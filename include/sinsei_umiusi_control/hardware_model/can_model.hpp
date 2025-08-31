#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP

#include <array>
#include <memory>
#include <rcpputils/tl_expected/expected.hpp>
#include <variant>

#include "sinsei_umiusi_control/cmd/led_tape.hpp"
#include "sinsei_umiusi_control/cmd/main_power.hpp"
#include "sinsei_umiusi_control/cmd/thruster.hpp"
#include "sinsei_umiusi_control/hardware_model/can/vesc_model.hpp"
#include "sinsei_umiusi_control/hardware_model/interface/can.hpp"
#include "sinsei_umiusi_control/state/esc.hpp"
#include "sinsei_umiusi_control/state/main_power.hpp"
#include "sinsei_umiusi_control/state/thruster.hpp"

namespace sinsei_umiusi_control::hardware_model {

class CanModel {
  public:
    using ThrusterId = size_t;
    using EscEnabled = cmd::thruster::EscEnabled;
    using ServoAngle = cmd::thruster::Angle;
    using EscDutyCycle = cmd::thruster::DutyCycle;
    using ServoEnabled = cmd::thruster::ServoEnabled;

  private:
    using WriteCommand = std::variant<
        cmd::main_power::Enabled, std::tuple<ThrusterId, EscEnabled>,
        std::tuple<ThrusterId, ServoEnabled>, std::tuple<ThrusterId, EscDutyCycle>,
        std::tuple<ThrusterId, ServoAngle>, cmd::led_tape::Color>;

    std::shared_ptr<interface::Can> can;

    std::array<can::VescModel, 4> vesc_models;

    // main_powerが更新されたときは必ずこれを送信する
    cmd::main_power::Enabled last_main_power_enabled;

    // `(% 16) / 4`: `EscEnabled`, `ServoEnabled`, `DutyCycle` or `Angle`
    // `% 4`:        `thruster ID - 1`
    size_t loop_times = 0;

    // スラスタのコマンドが何周するごとにLEDテープのコマンドを1回送信するか
    const size_t period_led_tape_per_thrusters;

    // Update the internal state and select a command to write.
    auto update_and_generate_command(
        cmd::main_power::Enabled && main_power_enabled,
        std::array<cmd::thruster::EscEnabled, 4> && thruster_esc_enabled,
        std::array<cmd::thruster::ServoEnabled, 4> && thruster_servo_enabled,
        std::array<cmd::thruster::DutyCycle, 4> && thruster_duty_cycle,
        std::array<cmd::thruster::Angle, 4> && thruster_angle,
        cmd::led_tape::Color && led_tape_color) -> WriteCommand;

    auto update_and_generate_command(
        cmd::main_power::Enabled && main_power_enabled,
        cmd::led_tape::Color && led_tape_color) -> WriteCommand;

  public:
    CanModel(
        std::shared_ptr<interface::Can> can, std::array<int, 4> vesc_ids,
        size_t period_led_tape_per_thrusters);
    auto on_init() -> tl::expected<void, std::string>;
    auto on_destroy() -> tl::expected<void, std::string>;
    auto on_read() const
        -> tl::expected<
            std::variant<
                std::tuple<size_t, state::thruster::Rpm>,
                std::tuple<size_t, state::esc::WaterLeaked>, state::main_power::BatteryCurrent,
                state::main_power::BatteryVoltage, state::main_power::Temperature,
                state::main_power::WaterLeaked>,
            std::string>;
    auto on_write(
        cmd::main_power::Enabled && main_power_enabled,
        std::array<cmd::thruster::EscEnabled, 4> && thruster_esc_enabled,
        std::array<cmd::thruster::ServoEnabled, 4> && thruster_servo_enabled,
        std::array<cmd::thruster::DutyCycle, 4> && thruster_duty_cycle,
        std::array<cmd::thruster::Angle, 4> && thruster_angle,
        cmd::led_tape::Color && led_tape_color) -> tl::expected<void, std::string>;

    auto on_write(cmd::main_power::Enabled main_power_enabled, cmd::led_tape::Color led_tape_color)
        -> tl::expected<void, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP
