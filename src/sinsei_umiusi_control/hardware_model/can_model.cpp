#include "sinsei_umiusi_control/hardware_model/can_model.hpp"

#include <rcpputils/tl_expected/expected.hpp>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace sinsei_umiusi_control::hardware_model;

auto CanModel::update_and_generate_command(
    cmd::main_power::Enabled main_power_enabled,
    const std::vector<ActuatorCommand> & actuator_commands,
    cmd::led_tape::Color led_tape_color) -> WriteCommand {
    this->loop_times++;

    // main_power_enabled
    if (this->last_main_power_enabled.value != main_power_enabled.value) {
        this->last_main_power_enabled = main_power_enabled;
        return this->last_main_power_enabled;
    }

    const auto actuators_num = this->actuators.size();
    const auto actuator_command_count_per_cycle = actuators_num * ACTUATOR_COMMAND_TYPE_COUNT;

    const auto period_led_tape_per_loop =
        this->period_led_tape_per_actuators * actuator_command_count_per_cycle;

    // `period_led_tape_per_loop`回に1回LEDテープのコマンドを送信する。
    // LEDテープのコマンドを送信しない場合はアクチュエータのコマンドを順番に送信する。
    const auto led = (this->loop_times % period_led_tape_per_loop) == 0;
    if (!led) {
        const auto actuator_index = this->loop_times % actuators_num;

        const auto command_type_index =
            (this->loop_times / actuators_num) % ACTUATOR_COMMAND_TYPE_COUNT;
        const auto & actuator = this->actuators[actuator_index];
        const auto & actuator_command = actuator_commands[actuator_index];

        switch (command_type_index) {
            case 0: {  // motor_allowed
                // allowedはVESCへ送るコマンドではなく、Duty出力のローカルな安全条件として扱う。
                break;
            }
            case 1: {  // motor_duty_cycle
                if (actuator.motor_type == MotorType::None) {
                    break;
                }
                // 無効化時にもDuty 0を送り、直前の出力がVESCに残らないようにする。
                return std::make_tuple(
                    actuator_index, MotorDutyCycle{
                                        actuator_command.motor_allowed.value
                                            ? actuator_command.motor_duty_cycle.value
                                            : 0.0});
            }
            case 2: {  // servo_allowed
                // allowedはVESCへ送るコマンドではなく、角度出力のローカルな安全条件として扱う。
                break;
            }
            case 3: {  // servo_angle
                if (!actuator.has_servo || !actuator_command.servo_allowed.value) {
                    break;  // サーボが無効の場合は角度を送信しない
                }
                return std::make_tuple(actuator_index, actuator_command.servo_angle);
            }
            default: {
                break;  // unreachable
            }
        }
    }

    if (led) {
        return led_tape_color;  // led_tape/color
    }
    return NoCommand{};
}

CanModel::CanModel(
    std::shared_ptr<interface::Can> can, std::vector<ActuatorConfig> actuator_configs,
    size_t period_led_tape_per_actuators)
: can(can),
  last_main_power_enabled{false},
  period_led_tape_per_actuators{period_led_tape_per_actuators} {
    this->actuators.reserve(actuator_configs.size());
    for (auto & config : actuator_configs) {
        this->actuators.push_back(Actuator{
            std::move(config.name), config.vesc_id, config.motor_type, config.has_servo,
            can::VescModel(config.vesc_id)});
    }
}

auto CanModel::validate_actuator_configs() const -> tl::expected<void, std::string> {
    if (this->actuators.empty()) {
        return tl::make_unexpected("At least one actuator must be configured");
    }

    auto names = std::unordered_set<std::string>{};
    auto vesc_ids = std::unordered_set<can::VescModel::Id>{};
    for (const auto & actuator : this->actuators) {
        if (actuator.name.empty()) {
            return tl::make_unexpected("Actuator name must not be empty");
        }
        if (!names.insert(actuator.name).second) {
            return tl::make_unexpected("Duplicate actuator name: " + actuator.name);
        }
        if (!vesc_ids.insert(actuator.vesc_id).second) {
            return tl::make_unexpected("Duplicate VESC ID: " + std::to_string(actuator.vesc_id));
        }
        if (actuator.motor_type == MotorType::None && !actuator.has_servo) {
            return tl::make_unexpected("Actuator must have a motor or servo: " + actuator.name);
        }
    }
    return {};
}

auto CanModel::on_init() -> tl::expected<void, std::string> {
    const auto validation_res = this->validate_actuator_configs();
    if (!validation_res) {
        return tl::make_unexpected("Invalid actuator configuration: " + validation_res.error());
    }
    if (this->period_led_tape_per_actuators <= 1) {
        return tl::make_unexpected("period_led_tape_per_actuators must be greater than 1");
    }

    const auto res = this->can->init("can0");
    if (!res) {
        return tl::make_unexpected("Failed to initialize CAN interface: " + res.error());
    }
    return {};
}

auto CanModel::on_destroy() -> tl::expected<void, std::string> {
    // TODO: ここで念のためアクチュエータを停止しておく

    const auto res = this->can->close();
    if (!res) {
        return tl::make_unexpected("Failed to close CAN interface: " + res.error());
    }
    return {};
}

auto CanModel::on_read() const -> tl::expected<ReadState, std::string> {
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

    for (const auto & actuator : this->actuators) {
        const auto description =
            "actuator '" + actuator.name + "' (VESC " + std::to_string(actuator.vesc_id) + ")";

        const auto packet_status_res = actuator.vesc_model.get_packet_status(frame_opt.value());
        if (!packet_status_res) {
            error_message += "    " + description + ": " + packet_status_res.error() + "\n";
            continue;
        }

        const auto & packet_status_opt = packet_status_res.value();
        if (!packet_status_opt) {
            // `Can::VescModel`では処理できないためスキップ
            continue;
        }

        // サーボ専用VESCのモーター状態は公開先が無いため、正常に受信した上で無視する。
        if (actuator.motor_type == MotorType::None) {
            return IgnoredUpdate{};
        }

        switch (packet_status_opt.value().index()) {
            case 0: {  // PacketStatus
                const auto & status = std::get<0>(packet_status_opt.value());
                const auto pole_pairs =
                    actuator.motor_type == MotorType::BLDC ? BLDC_POLES / 2.0 : 1.0;
                // BLDCはERPMを極対数で割り、DCはVESCの回転数をそのまま扱う。
                return std::make_tuple(
                    actuator.name, state::actuator::motor::Rpm{status.erpm / pole_pairs});
            }
            case 4: {  // PacketStatus5
                const auto & status = std::get<4>(packet_status_opt.value());
                const auto volts_in = status.volts_in;
                return std::make_tuple(actuator.name, state::actuator::motor::Voltage{volts_in});
            }
            case 5: {  // PacketStatus6
                const auto & status = std::get<5>(packet_status_opt.value());
                // 浸水センサーはADC1に接続されている
                const auto water_leaked = status.adc1 < WATER_LEAKED_VOLTAGE_THRESHOLD;
                return std::make_tuple(
                    actuator.name, state::actuator::motor::WaterLeaked{water_leaked});
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
    const std::vector<ActuatorCommand> & actuator_commands,
    cmd::led_tape::Color led_tape_color) -> tl::expected<void, std::string> {
    if (this->actuators.empty()) {
        return tl::make_unexpected("No actuators are configured");
    }

    if (actuator_commands.size() != this->actuators.size()) {
        return tl::make_unexpected(
            "Actuator command count does not match configuration: expected " +
            std::to_string(this->actuators.size()) + ", got " +
            std::to_string(actuator_commands.size()));
    }

    auto command =
        this->update_and_generate_command(main_power_enabled, actuator_commands, led_tape_color);

    auto frame = interface::CanFrame{};

    switch (command.index()) {
        case 0: {  // suc::cmd::main_power::Enabled
            auto & main_power_enabled = std::get<0>(command);

            // TODO: `main_power_enabled`の処理を実装する
            auto _ = main_power_enabled;
            return tl::make_unexpected("Not implemented for main power enabled command");
        }

        case 1: {  // std::tuple<ActuatorIndex, MotorDutyCycle>
            auto & [index, motor_duty_cycle] = std::get<1>(command);

            auto duty_frame_res =
                this->actuators[index].vesc_model.make_duty_frame(motor_duty_cycle.value);
            if (!duty_frame_res) {
                return tl::make_unexpected(
                    "Failed to create duty frame for actuator '" + this->actuators[index].name +
                    "': " + duty_frame_res.error());
            }
            frame = std::move(duty_frame_res.value());
            break;
        }

        case 2: {  // std::tuple<ActuatorIndex, ServoAngle>
            auto & [index, servo_angle] = std::get<2>(command);

            auto angle_frame_res =
                this->actuators[index].vesc_model.make_servo_angle_frame(servo_angle.value);
            if (!angle_frame_res) {
                return tl::make_unexpected(
                    "Failed to create servo angle frame for actuator '" +
                    this->actuators[index].name + "': " + angle_frame_res.error());
            }
            frame = std::move(angle_frame_res.value());
            break;
        }

        case 3: {  // cmd::led_tape::Color
            auto & led_tape_color = std::get<3>(command);

            // TODO: `led_tape_color`の処理を実装する
            auto _ = led_tape_color;
            return tl::make_unexpected("Not implemented for LED tape color command");
        }
        case 4: {  // NoCommand
            return {};
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
