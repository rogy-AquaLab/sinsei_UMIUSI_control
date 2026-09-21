#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP

#include <memory>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>
#include <tuple>
#include <variant>
#include <vector>

#include "sinsei_umiusi_control/cmd/led_tape.hpp"
#include "sinsei_umiusi_control/cmd/main_power.hpp"
#include "sinsei_umiusi_control/cmd/thruster/esc.hpp"
#include "sinsei_umiusi_control/cmd/thruster/servo.hpp"
#include "sinsei_umiusi_control/hardware_model/can/vesc_model.hpp"
#include "sinsei_umiusi_control/hardware_model/interface/can.hpp"
#include "sinsei_umiusi_control/state/main_power.hpp"
#include "sinsei_umiusi_control/state/thruster/esc.hpp"

namespace sinsei_umiusi_control::hardware_model {

class CanModel {
  public:
    using EscAllowed = cmd::thruster::esc::Allowed;
    using EscDutyCycle = cmd::thruster::esc::DutyCycle;
    using ServoAllowed = cmd::thruster::servo::Allowed;
    using ServoAngle = cmd::thruster::servo::Angle;
    using StateUpdate = std::variant<
        std::tuple<std::string, state::thruster::esc::Rpm>,
        std::tuple<std::string, state::thruster::esc::Voltage>,
        std::tuple<std::string, state::thruster::esc::WaterLeaked>,
        state::main_power::BatteryCurrent, state::main_power::BatteryVoltage,
        state::main_power::Temperature, state::main_power::WaterLeaked>;

    struct ThrusterConfig {
        std::string name;
        can::VescModel::Id vesc_id;
    };

    struct ThrusterCommand {
        EscAllowed esc_allowed;
        EscDutyCycle esc_duty_cycle;
        ServoAllowed servo_allowed;
        ServoAngle servo_angle;
    };

    struct ReadBatch {
        std::vector<StateUpdate> updates;
        std::string error_message;
    };

  private:
    struct Thruster {
        std::string name;
        can::VescModel::Id vesc_id;
        can::VescModel vesc_model;
    };

    enum class WritePhase {
        Esc,
        Servo,
    };

    std::shared_ptr<interface::Can> can;

    std::vector<Thruster> thrusters;

    // ESCとServoを交互に送信する
    WritePhase write_phase = WritePhase::Esc;

    // 使用するBLDCの極数
    static constexpr double BLDC_POLES = 14.0;

    // 浸水センサーの閾値（浸水とみなす最小電圧）
    // FIXME: 仮の値
    static constexpr double WATER_LEAKED_VOLTAGE_THRESHOLD = 2.0;

    auto validate_thruster_configs() const -> tl::expected<void, std::string>;
    auto decode_frame(const interface::CanFrame & frame) const
        -> tl::expected<StateUpdate, std::string>;

  public:
    CanModel(std::shared_ptr<interface::Can> can, std::vector<ThrusterConfig> thruster_configs);
    auto on_init() -> tl::expected<void, std::string>;
    auto on_destroy() -> tl::expected<void, std::string>;
    auto on_read() const -> tl::expected<ReadBatch, std::string>;
    // thruster_commandsはコンストラクタへ渡したthruster_configsと同じ順序で指定する
    auto on_write(
        cmd::main_power::Enabled main_power_enabled,
        const std::vector<ThrusterCommand> & thruster_commands,
        cmd::led_tape::Color led_tape_color) -> tl::expected<void, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP
