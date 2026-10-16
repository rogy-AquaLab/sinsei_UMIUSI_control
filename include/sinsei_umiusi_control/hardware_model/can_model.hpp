#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_MODEL_HPP

#include <memory>
#include <optional>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>
#include <tuple>
#include <variant>
#include <vector>

#include "sinsei_umiusi_control/cmd/actuator/motor.hpp"
#include "sinsei_umiusi_control/cmd/actuator/servo.hpp"
#include "sinsei_umiusi_control/cmd/led_tape.hpp"
#include "sinsei_umiusi_control/cmd/power_distribution.hpp"
#include "sinsei_umiusi_control/hardware_model/can/harmony_bms_model.hpp"
#include "sinsei_umiusi_control/hardware_model/can/vesc_model.hpp"
#include "sinsei_umiusi_control/hardware_model/interface/can.hpp"
#include "sinsei_umiusi_control/state/actuator/motor.hpp"

namespace sinsei_umiusi_control::hardware_model {

class CanModel {
  public:
    enum class MotorType { DC, BLDC, None };

    using MotorAllowed = cmd::actuator::motor::Allowed;
    using MotorDutyCycle = cmd::actuator::motor::DutyCycle;
    using ServoAllowed = cmd::actuator::servo::Allowed;
    using ServoAngle = cmd::actuator::servo::Angle;
    using DecodedState = std::variant<
        std::tuple<std::string, state::actuator::motor::Rpm>,
        std::tuple<std::string, state::actuator::motor::Voltage>,
        std::tuple<std::string, state::actuator::motor::WaterLeaked>,
        std::tuple<std::string, state::actuator::motor::Health>, can::HarmonyBmsModel::AnyPacket>;

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

    struct ReadBatch {
        std::vector<DecodedState> states;
        std::string error_message;
    };

  private:
    struct Actuator {
        std::string name;
        MotorType motor_type;
        bool has_servo;
        can::VescModel vesc_model;
    };

    enum class WritePhase {
        Motor,
        Servo,
    };

    std::shared_ptr<interface::Can> can;

    std::vector<Actuator> actuators;
    can::HarmonyBmsModel harmony_bms_model;

    // MotorとServoを交互に送信する
    WritePhase write_phase = WritePhase::Motor;

    // 使用するBLDCの極数
    static constexpr double BLDC_POLES = 14.0;

    // 浸水センサーの閾値（浸水とみなす最小電圧）
    // FIXME: 仮の値
    static constexpr double WATER_LEAKED_VOLTAGE_THRESHOLD = 2.0;

    auto validate_configuration() const -> tl::expected<void, std::string>;
    auto decode_frame(const interface::CanFrame & frame)
        -> tl::expected<std::optional<DecodedState>, std::string>;

  public:
    CanModel(
        std::shared_ptr<interface::Can> can, std::vector<ActuatorConfig> actuator_configs,
        can::HarmonyBmsModel::Id harmony_bms_id);
    auto on_init() -> tl::expected<void, std::string>;
    auto on_destroy() -> tl::expected<void, std::string>;
    auto on_read() -> tl::expected<ReadBatch, std::string>;
    // actuator_commandsはコンストラクタへ渡したactuator_configsと同じ順序で指定する
    auto on_write(
        cmd::power_distribution::Enabled power_distribution_enabled,
        const std::vector<ActuatorCommand> & actuator_commands,
        cmd::led_tape::Color led_tape_color) -> tl::expected<void, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_MODEL_HPP
