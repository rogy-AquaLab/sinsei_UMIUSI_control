#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP

#include <memory>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>
#include <variant>
#include <vector>

#include "sinsei_umiusi_control/cmd/actuator/motor.hpp"
#include "sinsei_umiusi_control/cmd/actuator/servo.hpp"
#include "sinsei_umiusi_control/cmd/led_tape.hpp"
#include "sinsei_umiusi_control/cmd/main_power.hpp"
#include "sinsei_umiusi_control/hardware_model/can/vesc_model.hpp"
#include "sinsei_umiusi_control/hardware_model/interface/can.hpp"
#include "sinsei_umiusi_control/state/actuator/motor.hpp"
#include "sinsei_umiusi_control/state/main_power.hpp"

namespace sinsei_umiusi_control::hardware_model {

class CanModel {
  public:
    enum class MotorType { DC, BLDC, None };

    using MotorAllowed = cmd::actuator::motor::Allowed;
    using MotorDutyCycle = cmd::actuator::motor::DutyCycle;
    using ServoAllowed = cmd::actuator::servo::Allowed;
    using ServoAngle = cmd::actuator::servo::Angle;

    struct ActuatorConfig {
        std::string name;
        can::VescModel::Id vesc_id;
        MotorType motor_type;
        bool has_servo;
    };

    struct ActuatorCommand {
        MotorAllowed motor_allowed;
        MotorDutyCycle motor_duty_cycle;
        ServoAllowed servo_allowed;
        ServoAngle servo_angle;
    };

    // CANフレームは登録済みのVESC宛てだが、この構成では公開する状態が無い。
    struct IgnoredUpdate {};

    using ReadState = std::variant<
        std::tuple<std::string, state::actuator::motor::Rpm>,
        std::tuple<std::string, state::actuator::motor::Voltage>,
        std::tuple<std::string, state::actuator::motor::WaterLeaked>,
        state::main_power::BatteryCurrent, state::main_power::BatteryVoltage,
        state::main_power::Temperature, state::main_power::WaterLeaked, IgnoredUpdate>;

  private:
    using ActuatorIndex = size_t;

    struct NoCommand {};

    using WriteCommand = std::variant<
        cmd::main_power::Enabled, std::tuple<ActuatorIndex, MotorDutyCycle>,
        std::tuple<ActuatorIndex, ServoAngle>, cmd::led_tape::Color, NoCommand>;

    struct Actuator {
        std::string name;
        can::VescModel::Id vesc_id;
        MotorType motor_type;
        bool has_servo;
        can::VescModel vesc_model;
    };

    std::shared_ptr<interface::Can> can;

    std::vector<Actuator> actuators;

    // main_powerが更新されたときは必ずこれを送信する
    cmd::main_power::Enabled last_main_power_enabled;

    // 1アクチュエータあたりのコマンド数
    // motor_allowed, duty_cycle, servo_allowed, angle
    static constexpr size_t ACTUATOR_COMMAND_TYPE_COUNT = 4;

    // 1周期分の送信順を管理するカウンタ
    size_t loop_times = 0;

    // アクチュエータのコマンドが何周するごとにLEDテープのコマンドを1回送信するか
    const size_t period_led_tape_per_actuators;

    // 使用するBLDCの極数
    static constexpr double BLDC_POLES = 14.0;

    // 浸水センサーの閾値（浸水とみなす最小電圧）
    // FIXME: 仮の値
    static constexpr double WATER_LEAKED_VOLTAGE_THRESHOLD = 2.0;

    // 内部状態を更新し、送信するコマンドを選択する
    auto update_and_generate_command(
        cmd::main_power::Enabled main_power_enabled,
        const std::vector<ActuatorCommand> & actuator_commands,
        cmd::led_tape::Color led_tape_color) -> WriteCommand;

    auto validate_actuator_configs() const -> tl::expected<void, std::string>;

  public:
    CanModel(
        std::shared_ptr<interface::Can> can, std::vector<ActuatorConfig> actuator_configs,
        size_t period_led_tape_per_actuators);
    auto on_init() -> tl::expected<void, std::string>;
    auto on_destroy() -> tl::expected<void, std::string>;
    auto on_read() const -> tl::expected<ReadState, std::string>;
    // actuator_commandsはコンストラクタへ渡したactuator_configsと同じ順序で指定する
    auto on_write(
        cmd::main_power::Enabled main_power_enabled,
        const std::vector<ActuatorCommand> & actuator_commands,
        cmd::led_tape::Color led_tape_color) -> tl::expected<void, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP
