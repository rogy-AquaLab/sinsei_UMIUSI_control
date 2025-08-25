#ifndef SINSEI_UMIUSI_CONTROL_GATE_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_GATE_CONTROLLER_HPP

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/publisher.hpp>

#include "sinsei_umiusi_control/cmd/attitude.hpp"
#include "sinsei_umiusi_control/cmd/headlights.hpp"
#include "sinsei_umiusi_control/cmd/indicator_led.hpp"
#include "sinsei_umiusi_control/cmd/led_tape.hpp"
#include "sinsei_umiusi_control/cmd/main_power.hpp"
#include "sinsei_umiusi_control/cmd/thruster.hpp"
#include "sinsei_umiusi_control/msg/angle.hpp"
#include "sinsei_umiusi_control/msg/color.hpp"
#include "sinsei_umiusi_control/msg/current.hpp"
#include "sinsei_umiusi_control/msg/duty_cycle.hpp"
#include "sinsei_umiusi_control/msg/enabled.hpp"
#include "sinsei_umiusi_control/msg/orientation.hpp"
#include "sinsei_umiusi_control/msg/quaternion.hpp"
#include "sinsei_umiusi_control/msg/rpm.hpp"
#include "sinsei_umiusi_control/msg/temprature.hpp"
#include "sinsei_umiusi_control/msg/velocity.hpp"
#include "sinsei_umiusi_control/msg/voltage.hpp"
#include "sinsei_umiusi_control/msg/water_leaked.hpp"
#include "sinsei_umiusi_control/state/esc.hpp"
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

        // TODO: スラスタごとに分ける
        sinsei_umiusi_control::cmd::thruster::ServoEnabled servo_enabled_ref;
        sinsei_umiusi_control::cmd::thruster::EscEnabled esc_enabled_ref;

        sinsei_umiusi_control::cmd::led_tape::Color led_tape_color_ref;
        sinsei_umiusi_control::cmd::attitude::Orientation target_orientation_ref;
        sinsei_umiusi_control::cmd::attitude::Velocity target_velocity_ref;
    };
    Command cmd;

    // State interfaces (in)
    struct State {
        sinsei_umiusi_control::state::main_power::BatteryCurrent battery_current;
        sinsei_umiusi_control::state::main_power::BatteryVoltage battery_voltage;
        sinsei_umiusi_control::state::main_power::Temperature main_temperature;
        sinsei_umiusi_control::state::main_power::WaterLeaked water_leaked;
        sinsei_umiusi_control::state::imu::Temperature imu_temperature;
        sinsei_umiusi_control::state::imu::Quaternion imu_quaternion;
        sinsei_umiusi_control::state::imu::Velocity imu_velocity;
        std::array<sinsei_umiusi_control::state::thruster::Rpm, 4> rpm;
        std::array<sinsei_umiusi_control::state::thruster::EscEnabled, 4> esc_enabled;
        std::array<sinsei_umiusi_control::state::thruster::ServoEnabled, 4> servo_enabled;
        std::array<sinsei_umiusi_control::state::thruster::DutyCycle, 4> thruster_duty_cycles;
        std::array<sinsei_umiusi_control::state::thruster::Angle, 4> thruster_angles;
        std::array<sinsei_umiusi_control::state::esc::WaterLeaked, 4> esc_water_leaked;
    };
    State state;

    // Subscribers for commands
    struct Subscribers {
        rclcpp::Subscription<msg::Enabled>::SharedPtr indicator_led_enabled_subscriber;
        rclcpp::Subscription<msg::Enabled>::SharedPtr main_power_enabled_subscriber;
        rclcpp::Subscription<msg::Enabled>::SharedPtr high_beam_enabled_subscriber;
        rclcpp::Subscription<msg::Enabled>::SharedPtr low_beam_enabled_subscriber;
        rclcpp::Subscription<msg::Enabled>::SharedPtr ir_enabled_subscriber;
        rclcpp::Subscription<msg::Enabled>::SharedPtr servo_enabled_subscriber;
        rclcpp::Subscription<msg::Enabled>::SharedPtr esc_enabled_subscriber;
        rclcpp::Subscription<msg::Color>::SharedPtr led_tape_color_subscriber;
        rclcpp::Subscription<msg::Orientation>::SharedPtr target_orientation_subscriber;
        rclcpp::Subscription<msg::Velocity>::SharedPtr target_velocity_subscriber;
    };
    Subscribers sub;

    // Publishers for states
    struct Publishers {
        rclcpp::Publisher<msg::Current>::SharedPtr battery_current_publisher;
        rclcpp::Publisher<msg::Voltage>::SharedPtr battery_voltage_publisher;
        rclcpp::Publisher<msg::Temprature>::SharedPtr main_temperature_publisher;
        rclcpp::Publisher<msg::WaterLeaked>::SharedPtr water_leaked_publisher;
        rclcpp::Publisher<msg::Temprature>::SharedPtr imu_temperature_publisher;
        rclcpp::Publisher<msg::Quaternion>::SharedPtr imu_quaternion_publisher;
        rclcpp::Publisher<msg::Velocity>::SharedPtr imu_velocity_publisher;
        std::array<rclcpp::Publisher<msg::Rpm>::SharedPtr, 4> rpm_publisher;
        std::array<rclcpp::Publisher<msg::Enabled>::SharedPtr, 4> esc_enabled_publisher;
        std::array<rclcpp::Publisher<msg::Enabled>::SharedPtr, 4> servo_enabled_publisher;
        std::array<rclcpp::Publisher<msg::DutyCycle>::SharedPtr, 4> duty_cycles_publisher;
        std::array<rclcpp::Publisher<msg::Angle>::SharedPtr, 4> angle_publisher;
        std::array<rclcpp::Publisher<msg::WaterLeaked>::SharedPtr, 4> esc_water_leaked_publisher;
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
