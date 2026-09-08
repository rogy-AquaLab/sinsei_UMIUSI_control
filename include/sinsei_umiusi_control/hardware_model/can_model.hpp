#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP

#include <memory>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>
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

  private:
    using ThrusterIndex = size_t;

    using WriteCommand = std::variant<
        cmd::main_power::Enabled, std::tuple<ThrusterIndex, EscAllowed>,
        std::tuple<ThrusterIndex, ServoAllowed>, std::tuple<ThrusterIndex, EscDutyCycle>,
        std::tuple<ThrusterIndex, ServoAngle>, cmd::led_tape::Color>;

    struct Thruster {
        std::string name;
        can::VescModel::Id vesc_id;
        can::VescModel vesc_model;
    };

    std::shared_ptr<interface::Can> can;

    std::vector<Thruster> thrusters;

    // main_powerが更新されたときは必ずこれを送信する
    cmd::main_power::Enabled last_main_power_enabled;

    // 1スラスタあたりのコマンド数
    // esc_allowed, duty_cycle, servo_allowed, angle
    static constexpr size_t THRUSTER_COMMAND_TYPE_COUNT = 4;

    // 1周期分の送信順を管理するカウンタ
    size_t loop_times = 0;

    // スラスタのコマンドが何周するごとにLEDテープのコマンドを1回送信するか
    const size_t period_led_tape_per_thrusters;

    // 使用するBLDCの極数
    static constexpr double BLDC_POLES = 14.0;

    // 浸水センサーの閾値（浸水とみなす最小電圧）
    // FIXME: 仮の値
    static constexpr double WATER_LEAKED_VOLTAGE_THRESHOLD = 2.0;

    // 内部状態を更新し、送信するコマンドを選択する
    auto update_and_generate_command(
        cmd::main_power::Enabled main_power_enabled,
        const std::vector<ThrusterCommand> & thruster_commands,
        cmd::led_tape::Color led_tape_color) -> WriteCommand;

    auto validate_thruster_configs() const -> tl::expected<void, std::string>;

  public:
    CanModel(
        std::shared_ptr<interface::Can> can, std::vector<ThrusterConfig> thruster_configs,
        size_t period_led_tape_per_thrusters);
    auto on_init() -> tl::expected<void, std::string>;
    auto on_destroy() -> tl::expected<void, std::string>;
    auto on_read() const
        -> tl::expected<
            std::variant<
                std::tuple<std::string, state::thruster::esc::Rpm>,
                std::tuple<std::string, state::thruster::esc::Voltage>,
                std::tuple<std::string, state::thruster::esc::WaterLeaked>,
                state::main_power::BatteryCurrent, state::main_power::BatteryVoltage,
                state::main_power::Temperature, state::main_power::WaterLeaked>,
            std::string>;
    // thruster_commandsはコンストラクタへ渡したthruster_configsと同じ順序で指定する
    auto on_write(
        cmd::main_power::Enabled main_power_enabled,
        const std::vector<ThrusterCommand> & thruster_commands,
        cmd::led_tape::Color led_tape_color) -> tl::expected<void, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP
