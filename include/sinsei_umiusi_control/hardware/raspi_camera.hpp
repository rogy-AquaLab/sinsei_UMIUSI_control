#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_RASPI_CAMERA_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_RASPI_CAMERA_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "rclcpp/macros.hpp"
#include "sinsei_umiusi_control/cmd/raspi_camera.hpp"
#include "sinsei_umiusi_control/state/raspi_camera.hpp"

namespace suc = sinsei_umiusi_control;

namespace sinsei_umiusi_control::hardware {

class RaspiCamera : public hardware_interface::ActuatorInterface {
  private:
    suc::cmd::raspi_camera::Config config;
    suc::state::raspi_camera::Image image;

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(RaspiCamera)

    RaspiCamera() = default;

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

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_RASPI_CAMERA_HPP