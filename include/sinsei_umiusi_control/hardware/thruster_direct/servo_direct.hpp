#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_THRUSTER_DIRECT_SERVO_DIRECT_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_THRUSTER_DIRECT_SERVO_DIRECT_HPP

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/macros.hpp>

#include "sinsei_umiusi_control/hardware_model/thruster_direct/servo_direct_model.hpp"

namespace sinsei_umiusi_control::hardware::thruster_direct {

using ID = int8_t;

class ServoDirect : public hardware_interface::SystemInterface {
  private:
    std::optional<hardware_model::thruster_direct::ServoDirectModel> model;
    ID id;  // 1 ~ 4

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ServoDirect)

    ServoDirect() = default;

    auto on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
        -> hardware_interface::CallbackReturn override;
    auto read(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
    auto write(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
};

}  // namespace sinsei_umiusi_control::hardware::thruster_direct

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_THRUSTER_DIRECT_SERVO_DIRECT_HPP
