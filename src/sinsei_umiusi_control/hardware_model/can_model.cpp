#include "sinsei_umiusi_control/hardware_model/can_model.hpp"

#include <rcpputils/tl_expected/expected.hpp>
#include <string>
#include <tuple>

#include "sinsei_umiusi_control/util/thruster_mode.hpp"

using namespace sinsei_umiusi_control::hardware_model;

auto CanModel::update_and_generate_command(
    cmd::main_power::Enabled && main_power_enabled,
    std::array<cmd::thruster::esc::Enabled, 4> && esc_enabled_flags,
    std::array<cmd::thruster::esc::DutyCycle, 4> && esc_duty_cycles,
    std::array<cmd::thruster::servo::Enabled, 4> && servo_enabled_flags,
    std::array<cmd::thruster::servo::Angle, 4> && servo_angles,
    cmd::led_tape::Color && led_tape_color) -> WriteCommand {
    this->loop_times++;

    // main_power_enabled
    if (this->last_main_power_enabled.value != main_power_enabled.value) {
        this->last_main_power_enabled = main_power_enabled;
        return this->last_main_power_enabled;
    }

    constexpr auto THRUSTERS_NUM = 4;        // 1 ~ 4
    constexpr auto THRUSTER_PACKET_NUM = 4;  // esc_enabled, servo_enabled, duty_cycle, angle
    constexpr auto THRUSTERS_TOTAL_PACKET_NUM = THRUSTERS_NUM * THRUSTER_PACKET_NUM;  // 16

    const auto period_led_tape_per_loop =
        this->period_led_tape_per_thrusters * THRUSTERS_TOTAL_PACKET_NUM;

    // `period_led_tape_per_loop`回に1回LEDテープのコマンドを送信する。
    // LEDテープのコマンドを送信しない場合はスラスターのコマンドを順番に送信する。
    const auto led = (this->loop_times % period_led_tape_per_loop) == 0;
    if (!led) {
        const auto thruster_index = this->loop_times % THRUSTERS_NUM;
        const auto thruster_id = thruster_index + 1;

        const auto packet_type =
            (this->loop_times % THRUSTERS_TOTAL_PACKET_NUM) / THRUSTER_PACKET_NUM;
        switch (packet_type) {
            case 0: {  // esc_enabled
                return std::forward_as_tuple(thruster_id, esc_enabled_flags[thruster_index]);
            }
            case 1: {  // esc_duty_cycle
                if (!esc_enabled_flags[thruster_index].value) {
                    break;  // ESCが無効の場合はデューティ比を送信しない
                }
                return std::forward_as_tuple(thruster_id, esc_duty_cycles[thruster_index]);
            }
            case 2: {  // servo_enabled
                return std::forward_as_tuple(thruster_id, servo_enabled_flags[thruster_index]);
            }
            case 3: {  // servo_angle
                if (!servo_enabled_flags[thruster_index].value) {
                    break;  // サーボが無効の場合は角度を送信しない
                }
                return std::forward_as_tuple(thruster_id, servo_angles[thruster_index]);
            }
            default: {
                break;  // unreachable
            }
        }
    }

    return led_tape_color;  // led_tape/color
}

auto CanModel::update_and_generate_command(
    cmd::main_power::Enabled && main_power_enabled_cmd,
    cmd::led_tape::Color && led_tape_color) -> WriteCommand {
    this->loop_times++;

    // main_power_enabled
    if (this->last_main_power_enabled.value != main_power_enabled_cmd.value) {
        this->last_main_power_enabled = main_power_enabled_cmd;
        return this->last_main_power_enabled;
    }
    return led_tape_color;  // led_tape/color
}

CanModel::CanModel(
    std::shared_ptr<interface::Can> can, std::array<int, 4> vesc_ids,
    size_t period_led_tape_per_thrusters, util::ThrusterMode thruster_mode)
: can(can),
  vesc_models{{
      can::VescModel(vesc_ids[0]),
      can::VescModel(vesc_ids[1]),
      can::VescModel(vesc_ids[2]),
      can::VescModel(vesc_ids[3]),
  }},
  last_main_power_enabled{false},
  period_led_tape_per_thrusters{period_led_tape_per_thrusters},
  thruster_mode(thruster_mode) {}

auto CanModel::on_init() -> tl::expected<void, std::string> {
    const auto res = this->can->init("can0");
    if (!res) {
        return tl::make_unexpected("Failed to initialize CAN interface: " + res.error());
    }
    return {};
}

auto CanModel::on_destroy() -> tl::expected<void, std::string> {
    // TODO: ここで念のためスラスターを停止しておく

    const auto res = this->can->close();
    if (!res) {
        return tl::make_unexpected("Failed to close CAN interface: " + res.error());
    }
    return {};
}

auto CanModel::on_read() const
    -> tl::expected<
        std::variant<
            std::tuple<size_t, state::thruster::esc::Rpm>,
            std::tuple<size_t, state::thruster::esc::Voltage>,
            std::tuple<size_t, state::thruster::esc::WaterLeaked>,
            state::main_power::BatteryCurrent, state::main_power::BatteryVoltage,
            state::main_power::Temperature, state::main_power::WaterLeaked>,
        std::string> {
    const auto frame = this->can->recv_frame();
    if (!frame) {
        return tl::make_unexpected("Failed to receive CAN frame: " + frame.error());
    }

    // フレームを各モデルに渡していく

    auto error_message = std::string("");

    // TODO: この位置に`can::MainPowerModel`の処理を追加する

    for (size_t i = 0; i < 4; ++i) {
        const auto vesc_id = std::to_string(i + 1);

        const auto packet_status_res = this->vesc_models[i].get_packet_status(frame.value());
        if (!packet_status_res) {
            error_message += "    VESC " + vesc_id + ": " + packet_status_res.error() + "\n";
            continue;
        }

        const auto & packet_status_opt = packet_status_res.value();
        if (!packet_status_opt) {
            // `Can::VescModel`では処理できないためスキップ
            continue;
        }

        // `thruster_mode`が`Direct`の時は、VESCからのフレームを無視する
        if (this->thruster_mode == util::ThrusterMode::Direct) {
            return tl::make_unexpected(
                "Received CAN frame from VESC " + vesc_id +
                " but thruster mode is Direct, so ignored.");
        }

        switch (packet_status_opt.value().index()) {
            case 0: {  // PacketStatus
                const auto & status = std::get<0>(packet_status_opt.value());
                constexpr double BLDC_POLE_PAIR = BLDC_POLES / 2.0;
                // ERPMを極対数で割ってRPMに変換
                return std::make_tuple(i, state::thruster::esc::Rpm{status.erpm / BLDC_POLE_PAIR});
            }
            case 4: {  // PacketStatus5
                const auto & status = std::get<4>(packet_status_opt.value());
                const auto volts_in = status.volts_in;
                return std::make_tuple(i, state::thruster::esc::Voltage{volts_in});
            }
            case 5: {  // PacketStatus6
                const auto & status = std::get<5>(packet_status_opt.value());
                // 浸水センサーはADC1に接続されている
                const auto water_leaked = status.adc1 < WATER_LEAKED_VOLTAGE_THRESHOLD;
                return std::make_tuple(i, state::thruster::esc::WaterLeaked{water_leaked});
            }
            default: {
                // 他のパケットは無視
                break;
            }
        }
    }

    // すべてのモデルでフレームを処理できなかった場合はエラー
    return tl::make_unexpected(
        "Failed to handle CAN frame \"" + std::to_string(frame.value().id) +
        "\" in all models: \n" + error_message);
}

auto CanModel::on_write(
    cmd::main_power::Enabled && main_power_enabled,
    std::array<cmd::thruster::esc::Enabled, 4> && esc_enabled_flags,
    std::array<cmd::thruster::esc::DutyCycle, 4> && esc_duty_cycles,
    std::array<cmd::thruster::servo::Enabled, 4> && servo_enabled_flags,
    std::array<cmd::thruster::servo::Angle, 4> && servo_angles,
    cmd::led_tape::Color && led_tape_color) -> tl::expected<void, std::string> {
    auto command = this->update_and_generate_command(
        std::forward<decltype(main_power_enabled)>(main_power_enabled),
        std::forward<decltype(esc_enabled_flags)>(esc_enabled_flags),
        std::forward<decltype(esc_duty_cycles)>(esc_duty_cycles),
        std::forward<decltype(servo_enabled_flags)>(servo_enabled_flags),
        std::forward<decltype(servo_angles)>(servo_angles),
        std::forward<decltype(led_tape_color)>(led_tape_color));

    auto frame = interface::CanFrame{};

    switch (command.index()) {
        case 0: {  // suc::cmd::main_power::Enabled
            auto & main_power_enabled = std::get<0>(command);

            // TODO: `main_power_enabled`の処理を実装する
            auto _ = main_power_enabled;
            return tl::make_unexpected("Not implemented for main power enabled command");
        }

        case 1: {  // std::tuple<ThrusterId, esc::Enabled>
            auto & [id, esc_enabled] = std::get<1>(command);
            if (id > vesc_models.size()) {
                return tl::make_unexpected("Invalid thruster ID: " + std::to_string(id));
            }

            // TODO: `esc_enabled`の処理を実装する
            auto _ = esc_enabled;
            return tl::make_unexpected("Not implemented for ESC enabled command");
        }

        case 2: {  // std::tuple<ThrusterId, ServoEnabled>
            auto & [id, servo_enabled] = std::get<2>(command);
            if (id > vesc_models.size()) {
                return tl::make_unexpected("Invalid thruster ID: " + std::to_string(id));
            }

            // TODO: `servo_enabled`の処理を実装する
            const auto _ = servo_enabled;
            return tl::make_unexpected("Not implemented for servo enabled command");
        }

        case 3: {  // std::tuple<ThrusterId, EscDutyCycle>
            auto & [id, esc_duty_cycle] = std::get<3>(command);
            if (id > vesc_models.size()) {
                return tl::make_unexpected("Invalid thruster ID: " + std::to_string(id));
            }

            auto duty_frame_res =
                vesc_models[id - 1].make_duty_frame(std::move(esc_duty_cycle.value));
            if (!duty_frame_res) {
                return tl::make_unexpected(
                    "Failed to create duty frame for thruster " + std::to_string(id) + ": " +
                    duty_frame_res.error());
            }
            frame = std::move(duty_frame_res.value());
            break;
        }

        case 4: {  // std::tuple<ThrusterId, ServoAngle>
            auto & [id, servo_angle] = std::get<4>(command);
            if (id > vesc_models.size()) {
                return tl::make_unexpected("Invalid thruster ID: " + std::to_string(id));
            }

            auto angle_frame_res =
                vesc_models[id - 1].make_servo_angle_frame(std::move(servo_angle.value));
            if (!angle_frame_res) {
                return tl::make_unexpected(
                    "Failed to create servo angle frame for thruster " + std::to_string(id) + ": " +
                    angle_frame_res.error());
            }
            frame = std::move(angle_frame_res.value());
            break;
        }

        case 5: {  // cmd::led_tape::Color
            auto & led_tape_color = std::get<5>(command);

            // TODO: `led_tape_color`の処理を実装する
            auto _ = led_tape_color;
            return tl::make_unexpected("Not implemented for LED tape color command");
        }
        default: {
            return tl::make_unexpected("Unknown command type in CanModel::on_write");
        }
    }
    const auto res = this->can->send_frame(std::move(frame));
    if (!res) {
        return tl::make_unexpected(
            "Failed to send CAN frame (command type id: " + std::to_string(command.index()) +
            "): " + res.error());
    }
    return {};
}

auto CanModel::on_write(
    cmd::main_power::Enabled /*main_power_enabled*/,
    cmd::led_tape::Color /*led_tape_color*/) -> tl::expected<void, std::string> {
    // TODO: `main_power_enabled`と`led_tape_color`の処理を実装する
    //return tl::make_unexpected("Not implemented");
    return {};
}
