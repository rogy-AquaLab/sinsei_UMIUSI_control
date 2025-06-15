#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_THRUSTER_DIRECT_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_THRUSTER_DIRECT_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "rclcpp/macros.hpp"
#include "sinsei_umiusi_control/cmd/thruster.hpp"

namespace suc = sinsei_umiusi_control;

namespace sinsei_umiusi_control::hardware {

class ThrusterDirect : public hardware_interface::ActuatorInterface {
  private:
    suc::cmd::thruster::Enabled enabled;
    suc::cmd::thruster::Angle angle;
    suc::cmd::thruster::Thrust thrust;

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ThrusterDirect)

    ThrusterDirect() = default;

    auto read(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
    auto write(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
};

}  // namespace sinsei_umiusi_control::hardware

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_THRUSTER_DIRECT_HPP