#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_CAN_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_CAN_HPP

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/macros.hpp>

namespace sinsei_umiusi_control::hardware {

class Can : public hardware_interface::SystemInterface {
  private:
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