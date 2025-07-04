#ifndef SINSEI_UMIUSI_CONTROL_GATE_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_GATE_CONTROLLER_HPP

#include <rclcpp/publisher.hpp>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sinsei_umiusi_control/cmd/app.hpp"
#include "sinsei_umiusi_control/cmd/headlights.hpp"
#include "sinsei_umiusi_control/cmd/indicator_led.hpp"
#include "sinsei_umiusi_control/cmd/led_tape.hpp"
#include "sinsei_umiusi_control/cmd/main_power.hpp"
#include "sinsei_umiusi_control/cmd/thruster.hpp"
#include "sinsei_umiusi_control/interface_access_helper.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"
#include "sinsei_umiusi_control/state/main_power.hpp"
#include "sinsei_umiusi_control/state/thruster.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"

namespace sinsei_umiusi_control::controller {

class GateController : public controller_interface::ControllerInterface {
  private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr indicator_led_enabled_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr main_power_enabled_subscriber;
    rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr led_tape_color_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr high_beam_enabled_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr low_beam_enabled_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ir_enabled_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo_enabled_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr esc_enabled_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr target_orientation_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr target_velocity_subscriber;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_current_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_voltage_publisher;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr main_temperature_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr water_leaked_publisher;
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4> servo_current_publisher;
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4> rpm_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imu_orientation_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imu_velocity_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr imu_temperature_publisher;

    cmd::indicator_led::Enabled indicator_led_enabled_ref;
    cmd::main_power::Enabled main_power_enabled_ref;
    cmd::led_tape::Color led_tape_color_ref;
    cmd::headlights::HighBeamEnabled high_beam_enabled_ref;
    cmd::headlights::LowBeamEnabled low_beam_enabled_ref;
    cmd::headlights::IrEnabled ir_enabled_ref;
    cmd::thruster::ServoEnabled servo_enabled_ref;
    cmd::thruster::EscEnabled esc_enabled_ref;
    cmd::app::Orientation target_orientation_ref;
    cmd::app::Velocity target_velocity_ref;

    state::main_power::BatteryCurrent battery_current_ref;
    state::main_power::BatteryVoltage battery_voltage_ref;
    state::main_power::Temperature main_temperature_ref;
    state::main_power::WaterLeaked water_leaked_ref;
    std::array<state::thruster::ServoCurrent, 4> servo_current_ref;
    std::array<state::thruster::Rpm, 4> rpm_ref;
    state::imu::Orientation imu_orientation_ref;
    state::imu::Velocity imu_velocity_ref;
    state::imu::Temperature imu_temperature_ref;

    static constexpr size_t CMD_SIZE = 20;
    static constexpr const char * CMD_INTERFACE_NAMES[CMD_SIZE] = {
        "indicator_led/enabled",
        "main_power/enabled",
        "led_tape/color",
        "headlights/high_beam_enabled",
        "headlights/low_beam_enabled",
        "headlights/ir_enabled",
        "thruster_controller1/servo_enabled",
        "thruster_controller2/servo_enabled",
        "thruster_controller3/servo_enabled",
        "thruster_controller4/servo_enabled",
        "thruster_controller1/esc_enabled",
        "thruster_controller2/esc_enabled",
        "thruster_controller3/esc_enabled",
        "thruster_controller4/esc_enabled",
        "app_controller/target_orientation.x",
        "app_controller/target_orientation.y",
        "app_controller/target_orientation.z",
        "app_controller/target_velocity.x",
        "app_controller/target_velocity.y",
        "app_controller/target_velocity.z",
    };
    static constexpr size_t STATE_SIZE = 19;
    static constexpr const char * STATE_INTERFACE_NAMES[STATE_SIZE] = {
        "main_power/battery_current",
        "main_power/battery_voltage",
        "main_power/temperature",
        "main_power/water_leaked",
        "thruster_controller1/servo_current",
        "thruster_controller2/servo_current",
        "thruster_controller3/servo_current",
        "thruster_controller4/servo_current",
        "thruster_controller1/rpm",
        "thruster_controller2/rpm",
        "thruster_controller3/rpm",
        "thruster_controller4/rpm",
        "app_controller/imu_orientation.x",
        "app_controller/imu_orientation.y",
        "app_controller/imu_orientation.z",
        "app_controller/imu_velocity.x",
        "app_controller/imu_velocity.y",
        "app_controller/imu_velocity.z",
        "imu/temperature",
    };

    std::unique_ptr<InterfaceAccessHelper<CMD_SIZE, STATE_SIZE>> interface_helper;

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