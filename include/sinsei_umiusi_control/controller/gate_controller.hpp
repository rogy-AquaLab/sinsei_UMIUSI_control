#ifndef SINSEI_UMIUSI_CONTROL_GATE_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_GATE_CONTROLLER_HPP

#include <optional>
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
#include "sinsei_umiusi_control/interface_access_helper.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"
#include "sinsei_umiusi_control/state/main_power.hpp"
#include "sinsei_umiusi_control/state/raspi_camera.hpp"
#include "sinsei_umiusi_control/state/thruster.hpp"
#include "sinsei_umiusi_control/state/usb_camera.hpp"
#include "std_msgs/msg/bool.hpp"

namespace suc = sinsei_umiusi_control;

namespace sinsei_umiusi_control::controller {

class GateController : public controller_interface::ControllerInterface {
  private:
    // TODO: 今後`indicator_led/indicator_led/enabled`以外の分も実装する
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr indicator_led_enabled_subscriber;

    // TODO: 今後`indicator_led/indicator_led/enabled`以外の分も実装する
    cmd::indicator_led::Enabled indicator_led_enabled_ref;

    static constexpr size_t cmd_size = 22;
    static constexpr const char * cmd_interface_names[cmd_size] = {
        "indicator_led/indicator_led/enabled",
        "main_power/main_power/enabled",
        "led_tape/led_tape/color",
        "headlights/headlights/high_beam_enabled",
        "headlights/headlights/low_beam_enabled",
        "headlights/headlights/ir_enabled",
        "usb_camera/usb_camera/config",
        "raspi_camera/raspi_camera/config",
        "thruster_controller1/servo_enabled/servo_enabled",
        "thruster_controller2/servo_enabled/servo_enabled",
        "thruster_controller3/servo_enabled/servo_enabled",
        "thruster_controller4/servo_enabled/servo_enabled",
        "thruster_controller1/esc_enabled/esc_enabled",
        "thruster_controller2/esc_enabled/esc_enabled",
        "thruster_controller3/esc_enabled/esc_enabled",
        "thruster_controller4/esc_enabled/esc_enabled",
        "app_controller/target_orientation.x/target_orientation.x",
        "app_controller/target_orientation.y/target_orientation.y",
        "app_controller/target_orientation.z/target_orientation.z",
        "app_controller/target_velocity.x/target_velocity.x",
        "app_controller/target_velocity.y/target_velocity.y",
        "app_controller/target_velocity.z/target_velocity.z",
    };
    static constexpr size_t state_size = 21;
    static constexpr const char * state_interface_names[state_size] = {
        "main_power/main_power/battery_current",
        "main_power/main_power/battery_voltage",
        "main_power/main_power/temperature",
        "main_power/main_power/water_leaked",
        "thruster_controller1/servo_current/servo_current",
        "thruster_controller2/servo_current/servo_current",
        "thruster_controller3/servo_current/servo_current",
        "thruster_controller4/servo_current/servo_current",
        "thruster_controller1/rpm/rpm",
        "thruster_controller2/rpm/rpm",
        "thruster_controller3/rpm/rpm",
        "thruster_controller4/rpm/rpm",
        "app_controller/imu_orientation.x/imu_orientation.x",
        "app_controller/imu_orientation.y/imu_orientation.y",
        "app_controller/imu_orientation.z/imu_orientation.z",
        "app_controller/imu_velocity.x/imu_velocity.x",
        "app_controller/imu_velocity.y/imu_velocity.y",
        "app_controller/imu_velocity.z/imu_velocity.z",
        "imu/imu/temperature",
        "usb_camera/usb_camera/image",
        "raspi_camera/raspi_camera/image",
    };

    std::unique_ptr<InterfaceAccessHelper<rclcpp_lifecycle::LifecycleNode, cmd_size, state_size>>
        interface_helper_;

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