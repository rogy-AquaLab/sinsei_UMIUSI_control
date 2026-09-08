#include "sinsei_umiusi_control/hardware_model/can_model.hpp"

#include <rcpputils/tl_expected/expected.hpp>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "sinsei_umiusi_control/cmd/thruster/servo.hpp"

using namespace sinsei_umiusi_control::hardware_model;

auto CanModel::update_and_generate_command(
    cmd::main_power::Enabled main_power_enabled,
    const std::vector<ThrusterCommand> & thruster_commands,
    cmd::led_tape::Color led_tape_color) -> WriteCommand {
    this->loop_times++;

    // main_power_enabled
    if (this->last_main_power_enabled.value != main_power_enabled.value) {
        this->last_main_power_enabled = main_power_enabled;
        return this->last_main_power_enabled;
    }

    constexpr auto THRUSTER_PACKET_NUM = 4;  // esc_allowed, duty_cycle, servo_allowed, angle
    const auto thrusters_num = this->thrusters.size();
    const auto thrusters_total_packet_num = thrusters_num * THRUSTER_PACKET_NUM;

    const auto period_led_tape_per_loop =
        this->period_led_tape_per_thrusters * thrusters_total_packet_num;

    // `period_led_tape_per_loop`回に1回LEDテープのコマンドを送信する。
    // LEDテープのコマンドを送信しない場合はスラスターのコマンドを順番に送信する。
    const auto led = (this->loop_times % period_led_tape_per_loop) == 0;
    if (!led) {
        const auto thruster_index = this->loop_times % thrusters_num;

        const auto packet_type = (this->loop_times % thrusters_total_packet_num) / thrusters_num;
        const auto & thruster_command = thruster_commands[thruster_index];

        switch (packet_type) {
            case 0: {  // esc_allowed
                return std::make_tuple(thruster_index, thruster_command.esc_allowed);
            }
            case 1: {  // esc_duty_cycle
                if (!thruster_command.esc_allowed.value) {
                    break;  // ESCが無効の場合はデューティ比を送信しない
                }
                return std::make_tuple(thruster_index, thruster_command.esc_duty_cycle);
            }
            case 2: {  // servo_allowed
                return std::make_tuple(thruster_index, thruster_command.servo_allowed);
            }
            case 3: {  // servo_angle
                if (!thruster_command.servo_allowed.value) {
                    break;  // サーボが無効の場合は角度を送信しない
                }
                return std::make_tuple(thruster_index, thruster_command.servo_angle);
            }
            default: {
                break;  // unreachable
            }
        }
    }

    return led_tape_color;  // led_tape/color
}

CanModel::CanModel(
    std::shared_ptr<interface::Can> can, std::vector<ThrusterConfig> thruster_configs,
    size_t period_led_tape_per_thrusters)
: can(can),
  last_main_power_enabled{false},
  period_led_tape_per_thrusters{period_led_tape_per_thrusters} {
    this->thrusters.reserve(thruster_configs.size());
    for (auto & config : thruster_configs) {
        this->thrusters.push_back(
            Thruster{std::move(config.name), config.vesc_id, can::VescModel(config.vesc_id)});
    }
}

auto CanModel::validate_thrusters() const -> tl::expected<void, std::string> {
    if (this->thrusters.empty()) {
        return tl::make_unexpected("At least one thruster must be configured");
    }

    auto names = std::unordered_set<std::string>{};
    auto vesc_ids = std::unordered_set<can::VescModel::Id>{};
    for (const auto & thruster : this->thrusters) {
        if (thruster.name.empty()) {
            return tl::make_unexpected("Thruster name must not be empty");
        }
        if (!names.insert(thruster.name).second) {
            return tl::make_unexpected("Duplicate thruster name: " + thruster.name);
        }
        if (!vesc_ids.insert(thruster.vesc_id).second) {
            return tl::make_unexpected("Duplicate VESC ID: " + std::to_string(thruster.vesc_id));
        }
    }
    return {};
}

auto CanModel::on_init() -> tl::expected<void, std::string> {
    const auto validation_res = this->validate_thrusters();
    if (!validation_res) {
        return tl::make_unexpected("Invalid thruster configuration: " + validation_res.error());
    }

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
            std::tuple<std::string, state::thruster::esc::Rpm>,
            std::tuple<std::string, state::thruster::esc::Voltage>,
            std::tuple<std::string, state::thruster::esc::WaterLeaked>,
            state::main_power::BatteryCurrent, state::main_power::BatteryVoltage,
            state::main_power::Temperature, state::main_power::WaterLeaked>,
        std::string> {
    const auto frame_res = this->can->recv_frame();
    if (!frame_res) {
        return tl::make_unexpected("Failed to receive CAN frame: " + frame_res.error());
    }
    const auto & frame_opt = frame_res.value();
    if (!frame_opt) {
        return tl::make_unexpected(
            "CAN read timeout: no CAN frame received within the timeout period");
    }

    // フレームを各モデルに渡していく

    auto error_message = std::string("");

    // TODO: この位置に`can::MainPowerModel`の処理を追加する

    for (const auto & thruster : this->thrusters) {
        const auto description =
            "thruster '" + thruster.name + "' (VESC " + std::to_string(thruster.vesc_id) + ")";

        const auto packet_status_res = thruster.vesc_model.get_packet_status(frame_opt.value());
        if (!packet_status_res) {
            error_message += "    " + description + ": " + packet_status_res.error() + "\n";
            continue;
        }

        const auto & packet_status_opt = packet_status_res.value();
        if (!packet_status_opt) {
            // `Can::VescModel`では処理できないためスキップ
            continue;
        }

        switch (packet_status_opt.value().index()) {
            case 0: {  // PacketStatus
                const auto & status = std::get<0>(packet_status_opt.value());
                constexpr double BLDC_POLE_PAIR = BLDC_POLES / 2.0;
                // ERPMを極対数で割ってRPMに変換
                return std::make_tuple(
                    thruster.name, state::thruster::esc::Rpm{status.erpm / BLDC_POLE_PAIR});
            }
            case 4: {  // PacketStatus5
                const auto & status = std::get<4>(packet_status_opt.value());
                const auto volts_in = status.volts_in;
                return std::make_tuple(thruster.name, state::thruster::esc::Voltage{volts_in});
            }
            case 5: {  // PacketStatus6
                const auto & status = std::get<5>(packet_status_opt.value());
                // 浸水センサーはADC1に接続されている
                const auto water_leaked = status.adc1 < WATER_LEAKED_VOLTAGE_THRESHOLD;
                return std::make_tuple(
                    thruster.name, state::thruster::esc::WaterLeaked{water_leaked});
            }
            default: {
                return tl::make_unexpected(
                    "Unsupported VESC packet status variant received (" + description +
                    ", variant index: " + std::to_string(packet_status_opt.value().index()) + ")");
            }
        }
    }

    if (error_message.empty()) {
        return tl::make_unexpected(
            "Unhandled CAN frame: no registered model accepted frame id " +
            std::to_string(frame_opt.value().id));
    }

    return tl::make_unexpected(
        "Failed to handle CAN frame \"" + std::to_string(frame_opt.value().id) +
        "\" in all models: \n" + error_message);
}

auto CanModel::on_write(
    cmd::main_power::Enabled main_power_enabled,
    const std::vector<ThrusterCommand> & thruster_commands,
    cmd::led_tape::Color led_tape_color) -> tl::expected<void, std::string> {
    if (this->thrusters.empty()) {
        return tl::make_unexpected("No thrusters are configured");
    }

    if (thruster_commands.size() != this->thrusters.size()) {
        return tl::make_unexpected(
            "Thruster command count does not match configuration: expected " +
            std::to_string(this->thrusters.size()) + ", got " +
            std::to_string(thruster_commands.size()));
    }

    auto command =
        this->update_and_generate_command(main_power_enabled, thruster_commands, led_tape_color);

    auto frame = interface::CanFrame{};

    switch (command.index()) {
        case 0: {  // suc::cmd::main_power::Enabled
            auto & main_power_enabled = std::get<0>(command);

            // TODO: `main_power_enabled`の処理を実装する
            auto _ = main_power_enabled;
            return tl::make_unexpected("Not implemented for main power enabled command");
        }

        case 1: {  // std::tuple<ThrusterIndex, EscAllowed>
            auto & [index, esc_allowed] = std::get<1>(command);

            // TODO: `esc_allowed`の処理を実装する
            auto _ = esc_allowed;
            return tl::make_unexpected(
                "Not implemented for ESC allowed command (thruster: " +
                this->thrusters[index].name + ")");
        }

        case 2: {  // std::tuple<ThrusterIndex, ServoAllowed>
            auto & [index, servo_allowed] = std::get<2>(command);

            // TODO: `servo_allowed`の処理を実装する
            const auto _ = servo_allowed;
            return tl::make_unexpected(
                "Not implemented for servo allowed command (thruster: " +
                this->thrusters[index].name + ")");
        }

        case 3: {  // std::tuple<ThrusterIndex, EscDutyCycle>
            auto & [index, esc_duty_cycle] = std::get<3>(command);

            auto duty_frame_res =
                this->thrusters[index].vesc_model.make_duty_frame(esc_duty_cycle.value);
            if (!duty_frame_res) {
                return tl::make_unexpected(
                    "Failed to create duty frame for thruster '" + this->thrusters[index].name +
                    "': " + duty_frame_res.error());
            }
            frame = std::move(duty_frame_res.value());
            break;
        }

        case 4: {  // std::tuple<ThrusterIndex, ServoAngle>
            auto & [index, servo_angle] = std::get<4>(command);

            auto angle_frame_res =
                this->thrusters[index].vesc_model.make_servo_angle_frame(servo_angle.value);
            if (!angle_frame_res) {
                return tl::make_unexpected(
                    "Failed to create servo angle frame for thruster '" +
                    this->thrusters[index].name + "': " + angle_frame_res.error());
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
    const auto res = this->can->send_frame(frame);
    if (!res) {
        return tl::make_unexpected(
            "Failed to send CAN frame (command type id: " + std::to_string(command.index()) +
            "): " + res.error());
    }
    return {};
}
