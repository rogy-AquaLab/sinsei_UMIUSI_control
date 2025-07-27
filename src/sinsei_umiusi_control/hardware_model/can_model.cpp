#include "sinsei_umiusi_control/hardware_model/can_model.hpp"

#include <rcpputils/tl_expected/expected.hpp>
#include <string>

#include "sinsei_umiusi_control/state/thruster.hpp"

namespace suc = sinsei_umiusi_control;
namespace suchm = suc::hardware_model;

auto suchm::CanModel::update_and_generate_command(
    bool is_can_mode, cmd::main_power::Enabled main_power_enabled,
    std::array<cmd::thruster::EscEnabled, 4> thruster_esc_enabled,
    std::array<cmd::thruster::ServoEnabled, 4> thruster_servo_enabled,
    std::array<cmd::thruster::DutyCycle, 4> thruster_duty_cycle,
    std::array<cmd::thruster::Angle, 4> thruster_angle,
    cmd::led_tape::Color led_tape_color) -> std::optional<WriteCommand> {
    {  // main_power/enabled
        if (this->main_power_enabled.value != main_power_enabled.value) {
            this->main_power_enabled = std::move(main_power_enabled);
            return this->main_power_enabled;
        }
    }
    // CANモードの場合のみ、各スラスターのコマンドを扱う
    if (is_can_mode) {
        for (size_t i = 0; i < 4; ++i) {  // thrusterN/esc/enbled
            if (this->thruster_esc_enabled[i].value != thruster_esc_enabled[i].value) {
                this->thruster_esc_enabled[i] = std::move(thruster_esc_enabled[i]);
                return std::make_pair(i, std::move(this->thruster_esc_enabled[i]));
            }
        }
        for (size_t i = 0; i < 4; ++i) {  // thrusterN/servo/enabled
            if (this->thruster_servo_enabled[i].value != thruster_servo_enabled[i].value) {
                this->thruster_servo_enabled[i] = std::move(thruster_servo_enabled[i]);
                return std::make_pair(i, std::move(this->thruster_servo_enabled[i]));
            }
        }
        for (size_t i = 0; i < 4; ++i) {  // thrusterN/esc/duty_cycle
            if (this->thruster_duty_cycle[i].value != thruster_duty_cycle[i].value) {
                this->thruster_duty_cycle[i] = std::move(thruster_duty_cycle[i]);
                return std::make_pair(i, std::move(this->thruster_duty_cycle[i]));
            }
        }
        for (size_t i = 0; i < 4; ++i) {  // thrusterN/servo/angle
            if (this->thruster_angle[i].value != thruster_angle[i].value) {
                this->thruster_angle[i] = std::move(thruster_angle[i]);
                return std::make_pair(i, std::move(this->thruster_angle[i]));
            }
        }
    }
    {  // led_tape/color
        if ((this->led_tape_color.red != led_tape_color.red) ||
            (this->led_tape_color.green != led_tape_color.green) ||
            (this->led_tape_color.blue != led_tape_color.blue)) {
            this->led_tape_color = std::move(led_tape_color);
            return this->led_tape_color;
        }
    }
    return std::nullopt;
}

suchm::CanModel::CanModel(std::shared_ptr<suchm::interface::Can> can, std::array<int, 4> vesc_ids)
: can(std::move(can)),
  vesc_models{{
      can::VescModel(vesc_ids[0]),
      can::VescModel(vesc_ids[1]),
      can::VescModel(vesc_ids[2]),
      can::VescModel(vesc_ids[3]),
  }},
  main_power_enabled{false},
  thruster_esc_enabled{{{false}, {false}, {false}, {false}}},
  thruster_servo_enabled{{{false}, {false}, {false}, {false}}},
  thruster_duty_cycle{{{0.0}, {0.0}, {0.0}, {0.0}}},
  thruster_angle{{{0.0}, {0.0}, {0.0}, {0.0}}},
  led_tape_color{0u, 0u, 0u} {}

auto suchm::CanModel::on_init() -> tl::expected<void, std::string> {
    auto res = this->can->init("can0");
    if (!res) {
        return tl::make_unexpected("Failed to initialize CAN interface: " + res.error());
    }
    return {};
}

auto suchm::CanModel::on_destroy() -> tl::expected<void, std::string> {
    // TODO: ここで念のためスラスターを停止しておく

    auto res = this->can->close();
    if (!res) {
        return tl::make_unexpected("Failed to close CAN interface: " + res.error());
    }
    return {};
}

auto suchm::CanModel::on_read()
    -> tl::expected<
        std::variant<
            std::pair<size_t, suc::state::thruster::Rpm>,
            std::pair<size_t, suc::state::esc::WaterLeaked>, suc::state::main_power::BatteryCurrent,
            suc::state::main_power::BatteryVoltage, suc::state::main_power::Temperature,
            suc::state::main_power::WaterLeaked>,
        std::string> {
    auto frame = this->can->recv_frame();
    if (!frame) {
        return tl::make_unexpected("Failed to receive CAN frame: " + frame.error());
    }

    // フレームを各モデルに渡していく

    std::string error_message;

    // TODO: この位置に`can::MainPowerModel`の処理を追加する

    for (size_t i = 0; i < 4; ++i) {
        auto rpm_res = this->vesc_models[i].get_rpm(frame.value());
        if (!rpm_res) {
            error_message += "    VESC " + std::to_string(i + 1) + ": " + rpm_res.error() + "\n";
            continue;
        }
        auto rpm_opt = rpm_res.value();
        if (rpm_opt) {
            std::pair<size_t, suc::state::thruster::Rpm> rpm_pair{i, rpm_opt.value()};
            return rpm_pair;
        }

        auto water_leaked_res = this->vesc_models[i].get_water_leaked(frame.value());
        if (!water_leaked_res) {
            error_message +=
                "    VESC " + std::to_string(i + 1) + ": " + water_leaked_res.error() + "\n";
            continue;
        }
        auto water_leaked_opt = water_leaked_res.value();
        if (water_leaked_opt) {
            std::pair<size_t, suc::state::esc::WaterLeaked> water_leaked_pair{
                i, water_leaked_opt.value()};
            return water_leaked_pair;
        }
    }

    // すべてのモデルでフレームを処理できなかった場合はエラー
    return tl::make_unexpected(
        "Failed to handle CAN frame \"" + std::to_string(frame.value().id) +
        "\" in all models: \n" + error_message);
}

auto suchm::CanModel::on_write(
    cmd::main_power::Enabled && main_power_enabled,
    std::array<cmd::thruster::EscEnabled, 4> && thruster_esc_enabled,
    std::array<cmd::thruster::ServoEnabled, 4> && thruster_servo_enabled,
    std::array<cmd::thruster::DutyCycle, 4> && thruster_duty_cycle,
    std::array<cmd::thruster::Angle, 4> && thruster_angle,
    cmd::led_tape::Color && led_tape_color) -> tl::expected<void, std::string> {
    auto command_opt = this->update_and_generate_command(
        true, std::move(main_power_enabled), std::move(thruster_esc_enabled),
        std::move(thruster_servo_enabled), std::move(thruster_duty_cycle),
        std::move(thruster_angle), std::move(led_tape_color));
    if (!command_opt) {
        // No command to write
        return {};
    }
    auto && command = std::move(command_opt.value());

    auto && frame = suchm::interface::CanFrame{};

    switch (command.index()) {
        case 0: {  // suc::cmd::main_power::Enabled
            auto & main_power_enabled = std::get<0>(command);

            // TODO: `main_power_enabled`の処理を実装する
            auto _ = main_power_enabled;
            return tl::make_unexpected("Not implemented for main power enabled command");
        }

        case 1: {  // std::pair<ThrusterId, EscEnabled>
            auto & [id, esc_enabled] = std::get<1>(command);
            if (id > vesc_models.size()) {
                return tl::make_unexpected("Invalid thruster ID: " + std::to_string(id));
            }

            // TODO: `esc_enabled`の処理を実装する
            auto _ = esc_enabled;
            return tl::make_unexpected("Not implemented for ESC enabled command");
        }

        case 2: {  // std::pair<ThrusterId, ServoEnabled>
            auto & [id, servo_enabled] = std::get<2>(command);
            if (id > vesc_models.size()) {
                return tl::make_unexpected("Invalid thruster ID: " + std::to_string(id));
            }

            // TODO: `servo_enabled`の処理を実装する
            auto _ = servo_enabled;
            return tl::make_unexpected("Not implemented for servo enabled command");
        }

        case 3: {  // std::pair<ThrusterId, EscDutyCycle>
            auto & [id, esc_duty_cycle] = std::get<3>(command);
            if (id > vesc_models.size()) {
                return tl::make_unexpected("Invalid thruster ID: " + std::to_string(id));
            }

            auto duty_frame_res = vesc_models[id - 1].make_duty_frame(esc_duty_cycle.value);
            if (!duty_frame_res) {
                return tl::make_unexpected(
                    "Failed to create duty frame for thruster " + std::to_string(id) + ": " +
                    duty_frame_res.error());
            }
            frame = std::move(duty_frame_res.value());
            break;
        }

        case 4: {  // std::pair<ThrusterId, ServoAngle>
            auto & [id, servo_angle] = std::get<4>(command);
            if (id > vesc_models.size()) {
                return tl::make_unexpected("Invalid thruster ID: " + std::to_string(id));
            }

            auto angle_frame_res = vesc_models[id - 1].make_servo_angle_frame(servo_angle.value);
            if (!angle_frame_res) {
                return tl::make_unexpected(
                    "Failed to create servo angle frame for thruster " + std::to_string(id) + ": " +
                    angle_frame_res.error());
            }
            frame = std::move(angle_frame_res.value());
            break;
        }

        case 5: {  // suc::cmd::led_tape::Color
            auto & led_tape_color = std::get<5>(command);

            // TODO: `led_tape_color`の処理を実装する
            auto _ = led_tape_color;
            return tl::make_unexpected("Not implemented for LED tape color command");
        }
        default: {
            return tl::make_unexpected("Unknown command type in CanModel::on_write");
        }
    }
    auto res = this->can->send_frame(std::move(frame));
    if (!res) {
        return tl::make_unexpected(
            "Failed to send CAN frame (command type id: " + std::to_string(command.index()) +
            "): " + res.error());
    }
    return {};
}

auto suchm::CanModel::on_write(
    suc::cmd::main_power::Enabled /*main_power_enabled*/,
    suc::cmd::led_tape::Color /*led_tape_color*/) -> tl::expected<void, std::string> {
    // TODO: `main_power_enabled`と`led_tape_color`の処理を実装する
    //return tl::make_unexpected("Not implemented");
    return {};
}