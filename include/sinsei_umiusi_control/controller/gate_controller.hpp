#ifndef SINSEI_UMIUSI_CONTROL_GATE_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_GATE_CONTROLLER_HPP

#include <memory>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sinsei_umiusi_control/cmd/app.hpp"
#include "sinsei_umiusi_control/cmd/headlights.hpp"
#include "sinsei_umiusi_control/cmd/indicator_led.hpp"
#include "sinsei_umiusi_control/cmd/led_tape.hpp"
#include "sinsei_umiusi_control/cmd/main_power.hpp"
#include "sinsei_umiusi_control/cmd/raspi_camera.hpp"
#include "sinsei_umiusi_control/cmd/thruster.hpp"
#include "sinsei_umiusi_control/cmd/usb_camera.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"
#include "sinsei_umiusi_control/state/main_power.hpp"
#include "sinsei_umiusi_control/state/raspi_camera.hpp"
#include "sinsei_umiusi_control/state/thruster.hpp"
#include "sinsei_umiusi_control/state/usb_camera.hpp"

namespace suc = sinsei_umiusi_control;

namespace sinsei_umiusi_control::controller {

class GateController : public controller_interface::ControllerInterface {
  private:
    // Command interfaces (out)
    std::unique_ptr<hardware_interface::LoanedCommandInterface> main_power_enabled;
    std::unique_ptr<hardware_interface::LoanedCommandInterface> led_tape_color;
    std::unique_ptr<hardware_interface::LoanedCommandInterface> indicator_led_enabled;
    std::unique_ptr<hardware_interface::LoanedCommandInterface> high_beam_enabled;
    std::unique_ptr<hardware_interface::LoanedCommandInterface> low_beam_enabled;
    std::unique_ptr<hardware_interface::LoanedCommandInterface> ir_enabled;
    std::unique_ptr<hardware_interface::LoanedCommandInterface> usb_camera_config;
    std::unique_ptr<hardware_interface::LoanedCommandInterface> raspi_camera_config;
    std::unique_ptr<hardware_interface::LoanedCommandInterface> thruster_enabled;
    std::unique_ptr<hardware_interface::LoanedCommandInterface> target_orientation;
    std::unique_ptr<hardware_interface::LoanedCommandInterface> target_velocity;

    // State interfaces (in)
    std::unique_ptr<hardware_interface::LoanedStateInterface> battery_current;
    std::unique_ptr<hardware_interface::LoanedStateInterface> battery_voltage;
    std::unique_ptr<hardware_interface::LoanedStateInterface> main_power_temperature;
    std::unique_ptr<hardware_interface::LoanedStateInterface> water_leaked;
    std::unique_ptr<hardware_interface::LoanedStateInterface> imu_orientation;
    std::unique_ptr<hardware_interface::LoanedStateInterface> imu_velocity;
    std::unique_ptr<hardware_interface::LoanedStateInterface> imu_temperature;
    std::unique_ptr<hardware_interface::LoanedStateInterface> usb_camera_image;
    std::unique_ptr<hardware_interface::LoanedStateInterface> raspi_camera_image;
    std::array<std::unique_ptr<hardware_interface::LoanedStateInterface>, 4> thruster_servo_current;
    std::array<std::unique_ptr<hardware_interface::LoanedStateInterface>, 4> thruster_rpm;

  public:
    GateController() = default;

    auto command_interface_configuration() const
        -> controller_interface::InterfaceConfiguration override;
    auto state_interface_configuration() const
        -> controller_interface::InterfaceConfiguration override;
    auto on_init() -> CallbackReturn override;
    auto on_configure(const rclcpp_lifecycle::State & previous_state) -> CallbackReturn override;
    auto on_activate(const rclcpp_lifecycle::State & previous_state) -> CallbackReturn override;
    auto on_deactivate(const rclcpp_lifecycle::State & previous_state) -> CallbackReturn override;
    auto update(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> controller_interface::return_type override;
};
}  // namespace sinsei_umiusi_control::controller

#endif  // SINSEI_UMIUSI_CONTROL_GATE_CONTROLLER_HPP