#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_IMU_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_IMU_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/sensor_interface.hpp"
#include "rclcpp/macros.hpp"

namespace sinsei_umiusi_control::hardware {

class Imu : public hardware_interface::SensorInterface {
  private:
    double command;
    double state;

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(Imu)

    Imu() = default;

    auto on_init(const hardware_interface::HardwareInfo & info)
        -> hardware_interface::CallbackReturn override;
    auto on_export_state_interfaces()
        -> std::vector<hardware_interface::StateInterface::ConstSharedPtr> override;
    auto read(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
};

}  // namespace sinsei_umiusi_control::hardware

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_IMU_HPP