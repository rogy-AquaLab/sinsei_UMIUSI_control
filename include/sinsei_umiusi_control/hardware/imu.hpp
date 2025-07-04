#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_IMU_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_IMU_HPP

#include <memory>

#include "hardware_interface/sensor_interface.hpp"
#include "rclcpp/macros.hpp"
#include "sinsei_umiusi_control/hardware_model/imu_model.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"
#include "sinsei_umiusi_control/util/pigpio.hpp"

namespace sinsei_umiusi_control::hardware {

class Imu : public hardware_interface::SensorInterface {
  private:
    std::optional<hardware_model::ImuModel> model;

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(Imu)

    Imu() = default;

    auto on_init(const hardware_interface::HardwareInfo & info)
        -> hardware_interface::CallbackReturn override;
    auto read(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
};

}  // namespace sinsei_umiusi_control::hardware

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_IMU_HPP