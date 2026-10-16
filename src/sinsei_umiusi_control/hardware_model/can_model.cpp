#include "sinsei_umiusi_control/hardware_model/can_model.hpp"

#include <rcpputils/tl_expected/expected.hpp>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace sinsei_umiusi_control::hardware_model;

CanModel::CanModel(
    std::shared_ptr<interface::Can> can, std::vector<ActuatorConfig> actuator_configs,
    can::HarmonyBmsModel::Id harmony_bms_id)
: can(can), harmony_bms_model(harmony_bms_id) {
    this->actuators.reserve(actuator_configs.size());
    for (auto & config : actuator_configs) {
        this->actuators.push_back(Actuator{
            std::move(config.name), config.motor_type, config.has_servo,
            can::VescModel(config.vesc_id)});
    }
}

auto CanModel::validate_configuration() const -> tl::expected<void, std::string> {
    if (this->actuators.empty()) {
        return tl::make_unexpected("At least one actuator must be configured");
    }

    auto names = std::unordered_set<std::string>{};
    auto vesc_ids = std::unordered_set<can::VescModel::Id>{};
    for (const auto & actuator : this->actuators) {
        const auto vesc_id = actuator.vesc_model.get_id();
        if (actuator.name.empty()) {
            return tl::make_unexpected("Actuator name must not be empty");
        }
        if (!names.insert(actuator.name).second) {
            return tl::make_unexpected("Duplicate actuator name: " + actuator.name);
        }
        if (!vesc_ids.insert(vesc_id).second) {
            return tl::make_unexpected("Duplicate VESC ID: " + std::to_string(vesc_id));
        }
        if (actuator.motor_type == MotorType::None && !actuator.has_servo) {
            return tl::make_unexpected("Actuator must have a motor or servo: " + actuator.name);
        }
        if (vesc_id == this->harmony_bms_model.get_id()) {
            return tl::make_unexpected(
                "Harmony BMS ID conflicts with VESC ID: " + std::to_string(vesc_id));
        }
    }
    return {};
}

auto CanModel::on_init() -> tl::expected<void, std::string> {
    const auto validation_res = this->validate_configuration();
    if (!validation_res) {
        return tl::make_unexpected("Invalid CAN node configuration: " + validation_res.error());
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

auto CanModel::decode_frame(const interface::CanFrame & frame)
    -> tl::expected<std::optional<DecodedState>, std::string> {
    // CANフレームをデコードするため、各モデルを順番に試す

    // 受信途中のVESCのフレームは無視する
    if (can::vesc_protocol::is_transport_frame(frame)) {
        return std::nullopt;
    }

    auto error_message = std::string("");

    auto bms_packet_res = this->harmony_bms_model.decode(frame);
    if (!bms_packet_res) {
        error_message += "    Harmony BMS: " + bms_packet_res.error() + "\n";
    } else if (bms_packet_res.value()) {
        auto & bms_packet = bms_packet_res.value().value();
        if (std::holds_alternative<std::monostate>(bms_packet)) {
            // 返す値がないBMSのフレームは無視する
            return std::nullopt;
        }
        return std::optional<DecodedState>{DecodedState{std::move(bms_packet)}};
    }

    // TODO: STM32側の仕様確定後に`can::PowerDistributionModel`の処理を追加する

    for (const auto & actuator : this->actuators) {
        const auto vesc_id = actuator.vesc_model.get_id();
        const auto description = "'" + actuator.name + "' (VESC " + std::to_string(vesc_id) + ")";

        const auto packet_status_res = actuator.vesc_model.decode(frame);
        if (!packet_status_res) {
            error_message += "    " + description + ": " + packet_status_res.error() + "\n";
            continue;
        }

        const auto & packet_status_opt = packet_status_res.value();
        if (!packet_status_opt) {
            // `Can::VescModel`では処理できないためスキップ
            continue;
        }

        // サーボ専用VESCのモーター状態は公開先が無いため、生存確認のためにのみ使用する
        if (actuator.motor_type == MotorType::None) {
            return std::optional<DecodedState>{DecodedState{
                std::make_tuple(actuator.name, state::actuator::motor::Health{true})}};
        }

        switch (packet_status_opt.value().index()) {
            case 0: {  // PacketStatus
                const auto & status = std::get<0>(packet_status_opt.value());
                const auto pole_pairs =
                    actuator.motor_type == MotorType::BLDC ? BLDC_POLES / 2.0 : 1.0;
                // BLDCはERPMを極対数で割り、DCはVESCの回転数をそのまま扱う
                return std::optional<DecodedState>{DecodedState{std::make_tuple(
                    actuator.name, state::actuator::motor::Rpm{status.erpm / pole_pairs})}};
            }
            case 4: {  // PacketStatus5
                const auto & status = std::get<4>(packet_status_opt.value());
                const auto volts_in = status.volts_in;
                return std::optional<DecodedState>{DecodedState{
                    std::make_tuple(actuator.name, state::actuator::motor::Voltage{volts_in})}};
            }
            case 5: {  // PacketStatus6
                const auto & status = std::get<5>(packet_status_opt.value());
                // 浸水センサーはADC1に接続されている
                const auto water_leaked = status.adc1 < WATER_LEAKED_VOLTAGE_THRESHOLD;
                return std::optional<DecodedState>{DecodedState{std::make_tuple(
                    actuator.name, state::actuator::motor::WaterLeaked{water_leaked})}};
            }
            default: {
                // STATUS_2/3/4はVESCの生存確認のためにのみ使用
                return std::optional<DecodedState>{DecodedState{
                    std::make_tuple(actuator.name, state::actuator::motor::Health{true})}};
            }
        }
    }

    if (error_message.empty()) {
        return tl::make_unexpected(
            "Failed to decode CAN frame: no registered model accepted frame id " +
            std::to_string(frame.id));
    }

    return tl::make_unexpected(
        "Failed to decode CAN frame \"" + std::to_string(frame.id) + "\" in all models: \n" +
        error_message);
}

auto CanModel::on_read() -> tl::expected<ReadBatch, std::string> {
    const auto frames_res = this->can->recv_frames();
    if (!frames_res) {
        return tl::make_unexpected("Failed to receive CAN frames: " + frames_res.error());
    }

    auto read_batch = ReadBatch{};
    read_batch.states.reserve(frames_res.value().size());
    for (const auto & frame : frames_res.value()) {
        auto state_res = this->decode_frame(frame);
        if (state_res) {
            if (state_res.value()) {
                read_batch.states.push_back(std::move(state_res.value().value()));
            }
            continue;
        }

        if (!read_batch.error_message.empty()) {
            read_batch.error_message += "\n";
        }
        read_batch.error_message += state_res.error();
    }
    return read_batch;
}

auto CanModel::on_write(
    cmd::power_distribution::Enabled /*power_distribution_enabled*/,
    const std::vector<ActuatorCommand> & actuator_commands,
    cmd::led_tape::Color /*led_tape_color*/) -> tl::expected<void, std::string> {
    if (this->actuators.empty()) {
        return tl::make_unexpected("No actuators are configured");
    }

    if (actuator_commands.size() != this->actuators.size()) {
        return tl::make_unexpected(
            "Actuator command count does not match configuration: expected " +
            std::to_string(this->actuators.size()) + ", got " +
            std::to_string(actuator_commands.size()));
    }

    const auto phase = this->write_phase;
    this->write_phase = phase == WritePhase::Motor ? WritePhase::Servo : WritePhase::Motor;

    // TODO: STM32側の仕様確定後にpower_distribution_enabledとled_tape_colorを送信する
    // allowedはVESCへ送るコマンドではなく、出力のローカルな安全条件として扱う

    auto error_message = std::string("");

    for (size_t index = 0; index < this->actuators.size(); ++index) {
        const auto & actuator = this->actuators[index];
        const auto & command = actuator_commands[index];

        if (phase == WritePhase::Motor && actuator.motor_type == MotorType::None) {
            continue;
        }
        if (phase == WritePhase::Servo && (!actuator.has_servo || !command.servo_allowed.value)) {
            continue;
        }

        // 無効化時にもDuty 0を送り、直前の出力がVESCに残らないようにする
        const auto frame_res =
            phase == WritePhase::Motor
                ? actuator.vesc_model.make_duty_frame(
                      command.motor_allowed.value ? command.motor_duty_cycle.value : 0.0)
                : actuator.vesc_model.make_servo_angle_frame(command.servo_angle.value);
        if (!frame_res) {
            if (!error_message.empty()) {
                error_message += "\n";
            }
            error_message +=
                "Failed to create CAN frame for '" + actuator.name + "': " + frame_res.error();
            continue;
        }

        const auto send_res = this->can->send_frame(frame_res.value());
        if (!send_res) {
            if (!error_message.empty()) {
                error_message += "\n";
            }
            error_message +=
                "Failed to send CAN frame for '" + actuator.name + "': " + send_res.error();
        }
    }

    if (!error_message.empty()) {
        return tl::make_unexpected(error_message);
    }
    return {};
}
