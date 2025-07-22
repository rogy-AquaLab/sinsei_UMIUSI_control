#ifndef SINSEI_UMIUSI_CONTROL_GATE_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_GATE_CONTROLLER_HPP

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>

#include "sinsei_umiusi_control/cmd/app.hpp"
#include "sinsei_umiusi_control/cmd/headlights.hpp"
#include "sinsei_umiusi_control/cmd/indicator_led.hpp"
#include "sinsei_umiusi_control/cmd/led_tape.hpp"
#include "sinsei_umiusi_control/cmd/main_power.hpp"
#include "sinsei_umiusi_control/cmd/thruster.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"
#include "sinsei_umiusi_control/state/main_power.hpp"
#include "sinsei_umiusi_control/state/thruster.hpp"
#include "sinsei_umiusi_control/util/interface_accessor.hpp"
#include "sinsei_umiusi_control/util/thruster_mode.hpp"

namespace sinsei_umiusi_control::controller {

class GateController : public controller_interface::ControllerInterface {
  private:
    // Command interfaces (out)
    struct Command {
        sinsei_umiusi_control::cmd::indicator_led::Enabled indicator_led_enabled_ref;
        sinsei_umiusi_control::cmd::main_power::Enabled main_power_enabled_ref;
        sinsei_umiusi_control::cmd::headlights::HighBeamEnabled high_beam_enabled_ref;
        sinsei_umiusi_control::cmd::headlights::LowBeamEnabled low_beam_enabled_ref;
        sinsei_umiusi_control::cmd::headlights::IrEnabled ir_enabled_ref;
        sinsei_umiusi_control::cmd::thruster::ServoEnabled servo_enabled_ref;
        sinsei_umiusi_control::cmd::thruster::EscEnabled esc_enabled_ref;
        sinsei_umiusi_control::cmd::led_tape::Color led_tape_color_ref;
        sinsei_umiusi_control::cmd::app::Orientation target_orientation_ref;
        sinsei_umiusi_control::cmd::app::Velocity target_velocity_ref;
    };
    Command cmd;

    // State interfaces (out)
    struct State {
        sinsei_umiusi_control::state::main_power::BatteryCurrent battery_current;
        sinsei_umiusi_control::state::main_power::BatteryVoltage battery_voltage;
        sinsei_umiusi_control::state::main_power::Temperature main_temperature;
        sinsei_umiusi_control::state::main_power::WaterLeaked water_leaked;
        sinsei_umiusi_control::state::imu::Temperature imu_temperature;
        sinsei_umiusi_control::state::imu::Orientation imu_orientation;
        sinsei_umiusi_control::state::imu::Velocity imu_velocity;
        std::array<sinsei_umiusi_control::state::thruster::Rpm, 4> rpm;
    };
    State state;

    // Subscribers for commands
    struct Subscribers {
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr indicator_led_enabled_subscriber;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr main_power_enabled_subscriber;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr high_beam_enabled_subscriber;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr low_beam_enabled_subscriber;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ir_enabled_subscriber;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo_enabled_subscriber;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr esc_enabled_subscriber;
        rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr led_tape_color_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr target_orientation_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr target_velocity_subscriber;
    };
    Subscribers sub;

    // Publishers for states
    struct Publishers {
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_current_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_voltage_publisher;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr main_temperature_publisher;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr water_leaked_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr imu_temperature_publisher;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imu_orientation_publisher;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imu_velocity_publisher;
        std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4> rpm_publisher;
    };
    Publishers pub;

    sinsei_umiusi_control::util::ThrusterMode thruster_mode;

    sinsei_umiusi_control::util::interface_accessor::InterfaceDataContainer command_interface_data;
    sinsei_umiusi_control::util::interface_accessor::InterfaceDataContainer state_interface_data;

  public:
    GateController() = default;

    auto command_interface_configuration() const
        -> controller_interface::InterfaceConfiguration override;
    auto state_interface_configuration() const
        -> controller_interface::InterfaceConfiguration override;
    auto on_init() -> CallbackReturn override;
    auto on_configure(const rclcpp_lifecycle::State & previous_state) -> CallbackReturn override;
    auto update(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> controller_interface::return_type override;
};
}  // namespace sinsei_umiusi_control::controller

#endif  // SINSEI_UMIUSI_CONTROL_GATE_CONTROLLER_HPP
