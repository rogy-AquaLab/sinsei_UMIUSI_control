#include "sinsei_umiusi_control/hardware_model/can_model.hpp"

#include <rcpputils/tl_expected/expected.hpp>
#include <string>
#include <tuple>

#include "sinsei_umiusi_control/cmd/main_power.hpp"
#include "sinsei_umiusi_control/state/thruster.hpp"

using namespace sinsei_umiusi_control::hardware_model;

auto CanModel::update_and_generate_command(
    cmd::main_power::Enabled && main_power_enabled,
    std::array<cmd::thruster::EscEnabled, 4> && thruster_esc_enabled,
    std::array<cmd::thruster::ServoEnabled, 4> && thruster_servo_enabled,
    std::array<cmd::thruster::DutyCycle, 4> && thruster_duty_cycle,
    std::array<cmd::thruster::Angle, 4> && thruster_angle,
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
                return std::forward_as_tuple(thruster_id, thruster_esc_enabled[thruster_index]);
            }
            case 1: {  // servo_enabled
                return std::forward_as_tuple(thruster_id, thruster_servo_enabled[thruster_index]);
            }
            case 2: {  // esc_duty_cycle
                return std::forward_as_tuple(thruster_id, thruster_duty_cycle[thruster_index]);
            }
            case 3: {  // angle
                return std::forward_as_tuple(thruster_id, thruster_angle[thruster_index]);
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
    size_t period_led_tape_per_thrusters)
: can(can),
  vesc_models{{
      can::VescModel(vesc_ids[0]),
      can::VescModel(vesc_ids[1]),
      can::VescModel(vesc_ids[2]),
      can::VescModel(vesc_ids[3]),
  }},
  last_main_power_enabled{false},
  period_led_tape_per_thrusters{period_led_tape_per_thrusters} {}

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
            std::tuple<size_t, state::thruster::Rpm>, std::tuple<size_t, state::esc::WaterLeaked>,
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
        const auto rpm_res = this->vesc_models[i].get_rpm(frame.value());
        if (!rpm_res) {
            error_message += "    VESC " + std::to_string(i + 1) + ": " + rpm_res.error() + "\n";
            continue;
        }
        const auto & rpm_opt = rpm_res.value();
        if (rpm_opt) {
            return std::make_tuple(i, rpm_opt.value());
        }

        const auto water_leaked_res = this->vesc_models[i].get_water_leaked(frame.value());
        if (!water_leaked_res) {
            error_message +=
                "    VESC " + std::to_string(i + 1) + ": " + water_leaked_res.error() + "\n";
            continue;
        }
        const auto & water_leaked_opt = water_leaked_res.value();
        if (water_leaked_opt) {
            return std::make_tuple(i, water_leaked_opt.value());
        }
    }

    // すべてのモデルでフレームを処理できなかった場合はエラー
    return tl::make_unexpected(
        "Failed to handle CAN frame \"" + std::to_string(frame.value().id) +
        "\" in all models: \n" + error_message);
}

auto CanModel::on_write(
    cmd::main_power::Enabled && main_power_enabled,
    std::array<cmd::thruster::EscEnabled, 4> && thruster_esc_enabled,
    std::array<cmd::thruster::ServoEnabled, 4> && thruster_servo_enabled,
    std::array<cmd::thruster::DutyCycle, 4> && thruster_duty_cycle,
    std::array<cmd::thruster::Angle, 4> && thruster_angle,
    cmd::led_tape::Color && led_tape_color) -> tl::expected<void, std::string> {
    auto command = this->update_and_generate_command(
        std::forward<decltype(main_power_enabled)>(main_power_enabled),
        std::forward<decltype(thruster_esc_enabled)>(thruster_esc_enabled),
        std::forward<decltype(thruster_servo_enabled)>(thruster_servo_enabled),
        std::forward<decltype(thruster_duty_cycle)>(thruster_duty_cycle),
        std::forward<decltype(thruster_angle)>(thruster_angle),
        std::forward<decltype(led_tape_color)>(led_tape_color));

    auto frame = interface::CanFrame{};

    switch (command.index()) {
        case 0: {  // suc::cmd::main_power::Enabled
            auto & main_power_enabled = std::get<0>(command);

            // TODO: `main_power_enabled`の処理を実装する
            auto _ = main_power_enabled;
            return tl::make_unexpected("Not implemented for main power enabled command");
        }

        case 1: {  // std::tuple<ThrusterId, EscEnabled>
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