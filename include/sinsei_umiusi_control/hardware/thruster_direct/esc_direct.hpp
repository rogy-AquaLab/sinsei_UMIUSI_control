#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_THRUSTER_DIRECT_ESC_DIRECT_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_THRUSTER_DIRECT_ESC_DIRECT_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "sinsei_umiusi_control/cmd/thruster.hpp"

namespace suc = sinsei_umiusi_control;

namespace sinsei_umiusi_control::hardware::thruster_direct {

class EscDirect : public hardware_interface::SystemInterface {
  private:
    suc::cmd::thruster::EscEnabled enabled;
    suc::cmd::thruster::Thrust thrust;

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(EscDirect)

    EscDirect() = default;

    auto read(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
    auto write(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
};

}  // namespace sinsei_umiusi_control::hardware::thruster_direct

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_THRUSTER_DIRECT_ESC_DIRECT_HPP