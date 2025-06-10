#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_THRUSTER_DIRECT_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_THRUSTER_DIRECT_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "rclcpp/macros.hpp"

namespace sinsei_umiusi_control {

namespace hardware {

class ThrusterDirect : public hardware_interface::ActuatorInterface {
  private:
    double command;
    double state;

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ThrusterDirect)

    ThrusterDirect() = default;

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

}  // namespace hardware

}  // namespace sinsei_umiusi_control

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_THRUSTER_DIRECT_HPP