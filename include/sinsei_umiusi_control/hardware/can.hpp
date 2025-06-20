#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_CAN_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_CAN_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "sinsei_umiusi_control/cmd/led_tape.hpp"
#include "sinsei_umiusi_control/cmd/main_power.hpp"
#include "sinsei_umiusi_control/cmd/thruster.hpp"
#include "sinsei_umiusi_control/state/main_power.hpp"
#include "sinsei_umiusi_control/state/thruster.hpp"

namespace suc = sinsei_umiusi_control;

namespace sinsei_umiusi_control::hardware {

class Can : public hardware_interface::SystemInterface {
  private:
    std::array<suc::cmd::thruster::EscEnabled, 4> thruster_esc_enabled;
    std::array<suc::cmd::thruster::ServoEnabled, 4> thruster_servo_enabled;
    std::array<suc::cmd::thruster::Angle, 4> thruster_angle;
    std::array<suc::cmd::thruster::Thrust, 4> thruster_thrust;
    suc::cmd::main_power::Enabled main_power_enabled;
    suc::cmd::led_tape::Color led_tape_color;

    std::array<suc::state::thruster::ServoCurrent, 4> thruster_servo_current;
    std::array<suc::state::thruster::Rpm, 4> thruster_rpm;
    suc::state::main_power::BatteryCurrent battery_current;
    suc::state::main_power::BatteryVoltage battery_voltage;
    suc::state::main_power::Temperature temperature;
    suc::state::main_power::WaterLeaked water_leaked;

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(Can)

    Can() = default;

    auto read(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
    auto write(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
};

}  // namespace sinsei_umiusi_control::hardware

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_CAN_HPP