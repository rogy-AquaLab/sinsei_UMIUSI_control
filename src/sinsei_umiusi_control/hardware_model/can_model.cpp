#include "sinsei_umiusi_control/hardware_model/can_model.hpp"

#include <rcpputils/tl_expected/expected.hpp>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "sinsei_umiusi_control/cmd/thruster/servo.hpp"

using namespace sinsei_umiusi_control::hardware_model;

CanModel::CanModel(
    std::shared_ptr<interface::Can> can, std::vector<ThrusterConfig> thruster_configs)
: can(can) {
    this->thrusters.reserve(thruster_configs.size());
    for (auto & config : thruster_configs) {
        this->thrusters.push_back(
            Thruster{std::move(config.name), config.vesc_id, can::VescModel(config.vesc_id)});
    }
}

auto CanModel::validate_thruster_configs() const -> tl::expected<void, std::string> {
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
    const auto validation_res = this->validate_thruster_configs();
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

auto CanModel::process_frame(const interface::CanFrame & frame) const
    -> tl::expected<StateUpdate, std::string> {
    // フレームを各モデルに渡していく

    auto error_message = std::string("");

    // TODO: この位置に`can::MainPowerModel`の処理を追加する

    for (const auto & thruster : this->thrusters) {
        const auto description =
            "'" + thruster.name + "' (VESC " + std::to_string(thruster.vesc_id) + ")";

        const auto packet_status_res = thruster.vesc_model.get_packet_status(frame);
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
                return StateUpdate{std::make_tuple(
                    thruster.name, state::thruster::esc::Rpm{status.erpm / BLDC_POLE_PAIR})};
            }
            case 4: {  // PacketStatus5
                const auto & status = std::get<4>(packet_status_opt.value());
                const auto volts_in = status.volts_in;
                return StateUpdate{
                    std::make_tuple(thruster.name, state::thruster::esc::Voltage{volts_in})};
            }
            case 5: {  // PacketStatus6
                const auto & status = std::get<5>(packet_status_opt.value());
                // 浸水センサーはADC1に接続されている
                const auto water_leaked = status.adc1 < WATER_LEAKED_VOLTAGE_THRESHOLD;
                return StateUpdate{std::make_tuple(
                    thruster.name, state::thruster::esc::WaterLeaked{water_leaked})};
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
            std::to_string(frame.id));
    }

    return tl::make_unexpected(
        "Failed to handle CAN frame \"" + std::to_string(frame.id) + "\" in all models: \n" +
        error_message);
}

auto CanModel::on_read() const -> tl::expected<ReadBatch, std::string> {
    const auto frames_res = this->can->recv_frames();
    if (!frames_res) {
        return tl::make_unexpected("Failed to receive CAN frames: " + frames_res.error());
    }

    auto read_batch = ReadBatch{};
    read_batch.updates.reserve(frames_res.value().size());
    for (const auto & frame : frames_res.value()) {
        auto update_res = this->process_frame(frame);
        if (update_res) {
            read_batch.updates.push_back(std::move(update_res.value()));
            continue;
        }

        if (!read_batch.error_message.empty()) {
            read_batch.error_message += "\n";
        }
        read_batch.error_message += update_res.error();
    }
    return read_batch;
}

auto CanModel::on_write(
    cmd::main_power::Enabled /*main_power_enabled*/,
    const std::vector<ThrusterCommand> & thruster_commands,
    cmd::led_tape::Color /*led_tape_color*/) -> tl::expected<void, std::string> {
    if (this->thrusters.empty()) {
        return tl::make_unexpected("No thrusters are configured");
    }

    if (thruster_commands.size() != this->thrusters.size()) {
        return tl::make_unexpected(
            "Thruster command count does not match configuration: expected " +
            std::to_string(this->thrusters.size()) + ", got " +
            std::to_string(thruster_commands.size()));
    }

    const auto phase = this->write_phase;
    this->write_phase = phase == WritePhase::Esc ? WritePhase::Servo : WritePhase::Esc;

    // TODO: main_power_enabledとled_tape_colorのCAN送信を実装する
    // TODO: esc_allowed/servo_allowedはLispBMが未実装

    auto error_message = std::string("");

    for (size_t index = 0; index < this->thrusters.size(); ++index) {
        const auto & thruster = this->thrusters[index];
        const auto & command = thruster_commands[index];

        if (phase == WritePhase::Esc && !command.esc_allowed.value) {
            continue;
        }
        if (phase == WritePhase::Servo && !command.servo_allowed.value) {
            continue;
        }

        const auto frame_res =
            phase == WritePhase::Esc
                ? thruster.vesc_model.make_duty_frame(command.esc_duty_cycle.value)
                : thruster.vesc_model.make_servo_angle_frame(command.servo_angle.value);
        if (!frame_res) {
            if (!error_message.empty()) {
                error_message += "\n";
            }
            error_message +=
                "Failed to create CAN frame for '" + thruster.name + "': " + frame_res.error();
            continue;
        }

        const auto send_res = this->can->send_frame(frame_res.value());
        if (!send_res) {
            if (!error_message.empty()) {
                error_message += "\n";
            }
            error_message +=
                "Failed to send CAN frame for '" + thruster.name + "': " + send_res.error();
        }
    }

    if (!error_message.empty()) {
        return tl::make_unexpected(error_message);
    }
    return {};
}
