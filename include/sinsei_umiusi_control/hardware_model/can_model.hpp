#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP

#include <array>
#include <memory>
#include <rcpputils/tl_expected/expected.hpp>
#include <variant>

#include "sinsei_umiusi_control/cmd/led_tape.hpp"
#include "sinsei_umiusi_control/cmd/main_power.hpp"
#include "sinsei_umiusi_control/cmd/thruster/esc.hpp"
#include "sinsei_umiusi_control/cmd/thruster/servo.hpp"
#include "sinsei_umiusi_control/hardware_model/can/vesc_model.hpp"
#include "sinsei_umiusi_control/hardware_model/interface/can.hpp"
#include "sinsei_umiusi_control/state/main_power.hpp"
#include "sinsei_umiusi_control/state/thruster/esc.hpp"
#include "sinsei_umiusi_control/state/thruster/servo.hpp"

namespace sinsei_umiusi_control::hardware_model {

class CanModel {
  public:
    using ThrusterId = size_t;

    using EscEnabled = cmd::thruster::esc::Enabled;
    using EscDutyCycle = cmd::thruster::esc::DutyCycle;
    using ServoEnabled = cmd::thruster::servo::Enabled;
    using ServoAngle = cmd::thruster::servo::Angle;

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

    // 使用するBLDCの極数
    static constexpr double BLDC_POLES = 14.0;

    // 浸水センサーの閾値（浸水とみなす最小電圧）
    // FIXME: 仮の値
    static constexpr double WATER_LEAKED_VOLTAGE_THRESHOLD = 2.0;

    // Update the internal state and select a command to write.
    auto update_and_generate_command(
        cmd::main_power::Enabled && main_power_enabled,
        std::array<cmd::thruster::esc::Enabled, 4> && esc_enabled_flags,
        std::array<cmd::thruster::esc::DutyCycle, 4> && esc_duty_cycles,
        std::array<cmd::thruster::servo::Enabled, 4> && servo_enabled_flags,
        std::array<cmd::thruster::servo::Angle, 4> && servo_angles,
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
                std::tuple<size_t, state::thruster::esc::Rpm>,
                std::tuple<size_t, state::thruster::esc::Voltage>,
                std::tuple<size_t, state::thruster::esc::WaterLeaked>,
                state::main_power::BatteryCurrent, state::main_power::BatteryVoltage,
                state::main_power::Temperature, state::main_power::WaterLeaked>,
            std::string>;
    auto on_write(
        cmd::main_power::Enabled && main_power_enabled,
        std::array<cmd::thruster::esc::Enabled, 4> && esc_enabled_flags,
        std::array<cmd::thruster::esc::DutyCycle, 4> && esc_duty_cycles,
        std::array<cmd::thruster::servo::Enabled, 4> && servo_enabled_flags,
        std::array<cmd::thruster::servo::Angle, 4> && servo_angles,
        cmd::led_tape::Color && led_tape_color) -> tl::expected<void, std::string>;

    auto on_write(cmd::main_power::Enabled main_power_enabled, cmd::led_tape::Color led_tape_color)
        -> tl::expected<void, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP
