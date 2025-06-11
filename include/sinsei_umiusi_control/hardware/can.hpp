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
    suc::cmd::thruster::Enabled thruster1_enabled;
    suc::cmd::thruster::Enabled thruster2_enabled;
    suc::cmd::thruster::Enabled thruster3_enabled;
    suc::cmd::thruster::Enabled thruster4_enabled;
    suc::cmd::thruster::Angle thruster1_angle;
    suc::cmd::thruster::Angle thruster2_angle;
    suc::cmd::thruster::Angle thruster3_angle;
    suc::cmd::thruster::Angle thruster4_angle;
    suc::cmd::thruster::Thrust thruster1_thrust;
    suc::cmd::thruster::Thrust thruster2_thrust;
    suc::cmd::thruster::Thrust thruster3_thrust;
    suc::cmd::thruster::Thrust thruster4_thrust;
    suc::cmd::main_power::Enabled main_power_enabled;
    suc::cmd::led_tape::Color led_tape_color;

    suc::state::thruster::ServoCurrent thruster1_servo_current;
    suc::state::thruster::ServoCurrent thruster2_servo_current;
    suc::state::thruster::ServoCurrent thruster3_servo_current;
    suc::state::thruster::ServoCurrent thruster4_servo_current;
    suc::state::thruster::Rpm thruster1_rpm;
    suc::state::thruster::Rpm thruster2_rpm;
    suc::state::thruster::Rpm thruster3_rpm;
    suc::state::thruster::Rpm thruster4_rpm;

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(Can)

    Can() = default;

    auto on_init(const hardware_interface::HardwareInfo & info)
        -> hardware_interface::CallbackReturn override;
    auto on_export_state_interfaces()
        -> std::vector<hardware_interface::StateInterface::ConstSharedPtr> override;
    auto on_export_command_interfaces()
        -> std::vector<hardware_interface::CommandInterface::SharedPtr> override;
    auto read(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
    auto write(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
};

}  // namespace sinsei_umiusi_control::hardware

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_CAN_HPP