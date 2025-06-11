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
    suc::cmd::main_power::Enabled main_power_enabled;
    suc::cmd::led_tape::Color led_tape_color;
    suc::cmd::indicator_led::Enabled indicator_led_enabled;
    suc::cmd::headlights::HighBeamEnabled high_beam_enabled;
    suc::cmd::headlights::LowBeamEnabled low_beam_enabled;
    suc::cmd::headlights::IrEnabled ir_enabled;
    suc::cmd::usb_camera::Config usb_camera_config;
    suc::cmd::raspi_camera::Config raspi_camera_config;
    suc::cmd::thruster::Enabled thruster_enabled;
    suc::cmd::app::Orientation target_orientation;
    suc::cmd::app::Velocity target_velocity;

    suc::state::main_power::BatteryCurrent battery_current;
    suc::state::main_power::BatteryVoltage battery_voltage;
    suc::state::main_power::Temperature main_power_temperature;
    suc::state::main_power::WaterLeaked water_leaked;
    suc::state::imu::Orientation imu_orientation;
    suc::state::imu::Velocity imu_velocity;
    suc::state::imu::Temperature imu_temperature;
    suc::state::usb_camera::Image usb_camera_image;
    suc::state::raspi_camera::Image raspi_camera_image;
    suc::state::thruster::ServoCurrent thruster1_servo_current;
    suc::state::thruster::ServoCurrent thruster2_servo_current;
    suc::state::thruster::ServoCurrent thruster3_servo_current;
    suc::state::thruster::ServoCurrent thruster4_servo_current;
    suc::state::thruster::Rpm thruster1_rpm;
    suc::state::thruster::Rpm thruster2_rpm;
    suc::state::thruster::Rpm thruster3_rpm;
    suc::state::thruster::Rpm thruster4_rpm;

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