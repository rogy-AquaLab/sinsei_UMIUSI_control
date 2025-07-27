#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP

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

namespace suc = sinsei_umiusi_control;

namespace sinsei_umiusi_control::hardware_model {

class CanModel {
  private:
    std::shared_ptr<interface::Can> can;

    std::array<can::VescModel, 4> vesc_models;

  public:
    CanModel(std::shared_ptr<interface::Can> can, std::array<int, 4> vesc_ids);
    auto on_init() -> tl::expected<void, std::string>;
    auto on_destroy() -> tl::expected<void, std::string>;
    auto on_read()
        -> tl::expected<
            std::variant<
                std::pair<size_t, suc::state::thruster::Rpm>,
                std::pair<size_t, suc::state::esc::WaterLeaked>,
                suc::state::main_power::BatteryCurrent, suc::state::main_power::BatteryVoltage,
                suc::state::main_power::Temperature, suc::state::main_power::WaterLeaked>,
            std::string>;
    auto on_write(
        std::array<suc::cmd::thruster::EscEnabled, 4> thruster_esc_enabled,
        std::array<suc::cmd::thruster::ServoEnabled, 4> thruster_servo_enabled,
        std::array<suc::cmd::thruster::Angle, 4> thruster_angle,
        std::array<suc::cmd::thruster::DutyCycle, 4> thruster_duty_cycle,
        suc::cmd::main_power::Enabled main_power_enabled,
        suc::cmd::led_tape::Color led_tape_color) -> tl::expected<void, std::string>;

    auto on_write(
        suc::cmd::main_power::Enabled main_power_enabled,
        suc::cmd::led_tape::Color led_tape_color) -> tl::expected<void, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP