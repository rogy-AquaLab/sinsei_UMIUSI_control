#ifndef SINSEI_UMIUSI_CONTROL_GATE_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_GATE_CONTROLLER_HPP

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include "sinsei_umiusi_control/cmd/attitude.hpp"
#include "sinsei_umiusi_control/cmd/headlights.hpp"
#include "sinsei_umiusi_control/cmd/indicator_led.hpp"
#include "sinsei_umiusi_control/cmd/led_tape.hpp"
#include "sinsei_umiusi_control/cmd/main_power.hpp"
#include "sinsei_umiusi_control/cmd/thruster/esc.hpp"
#include "sinsei_umiusi_control/cmd/thruster/servo.hpp"
#include "sinsei_umiusi_control/msg/esc_state.hpp"
#include "sinsei_umiusi_control/msg/headlights_output.hpp"
#include "sinsei_umiusi_control/msg/imu_state.hpp"
#include "sinsei_umiusi_control/msg/indicator_led_output.hpp"
#include "sinsei_umiusi_control/msg/led_tape_output.hpp"
#include "sinsei_umiusi_control/msg/main_power_output.hpp"
#include "sinsei_umiusi_control/msg/main_power_state.hpp"
#include "sinsei_umiusi_control/msg/target.hpp"
#include "sinsei_umiusi_control/msg/thruster_enabled_all.hpp"
#include "sinsei_umiusi_control/msg/thruster_state_all.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"
#include "sinsei_umiusi_control/state/main_power.hpp"
#include "sinsei_umiusi_control/state/thruster/esc.hpp"
#include "sinsei_umiusi_control/state/thruster/servo.hpp"
#include "sinsei_umiusi_control/util/interface_accessor.hpp"
#include "sinsei_umiusi_control/util/thruster_mode.hpp"

namespace sinsei_umiusi_control::controller {

class GateController : public controller_interface::ControllerInterface {
  public:
    struct Input {
        // State interfaces (in)
        struct State {
            sinsei_umiusi_control::state::main_power::BatteryVoltage main_power_battery_voltage;
            sinsei_umiusi_control::state::main_power::BatteryCurrent main_power_battery_current;
            sinsei_umiusi_control::state::main_power::Temperature main_temperature;
            sinsei_umiusi_control::state::main_power::WaterLeaked water_leaked;
            sinsei_umiusi_control::state::imu::Temperature imu_temperature;
            sinsei_umiusi_control::state::imu::Quaternion imu_quaternion;
            sinsei_umiusi_control::state::imu::Velocity imu_velocity;
            std::array<sinsei_umiusi_control::state::thruster::esc::Enabled, 4> esc_enabled_flags;
            std::array<sinsei_umiusi_control::state::thruster::esc::DutyCycle, 4> esc_duty_cycles;
            std::array<sinsei_umiusi_control::state::thruster::esc::Rpm, 4> esc_rpms;
            std::array<sinsei_umiusi_control::state::thruster::esc::WaterLeaked, 4>
                esc_water_leaked_flags;
            std::array<sinsei_umiusi_control::state::thruster::servo::Enabled, 4>
                servo_enabled_flags;
            std::array<sinsei_umiusi_control::state::thruster::servo::Angle, 4> servo_angles;
        };
        // Subscribers for commands
        struct Subscribers {
            rclcpp::Subscription<msg::IndicatorLedOutput>::SharedPtr
                indicator_led_output_subscriber;
            rclcpp::Subscription<msg::MainPowerOutput>::SharedPtr main_power_output_subscriber;
            rclcpp::Subscription<msg::HeadlightsOutput>::SharedPtr headlights_output_subscriber;
            rclcpp::Subscription<msg::ThrusterEnabledAll>::SharedPtr
                thruster_enabled_all_subscriber;
            rclcpp::Subscription<msg::LedTapeOutput>::SharedPtr led_tape_output_subscriber;
            rclcpp::Subscription<msg::Target>::SharedPtr target_subscriber;
        };
        State state;
        Subscribers sub;
    };
    struct Output {
        // Command interfaces (out)
        struct Command {
            sinsei_umiusi_control::cmd::indicator_led::Enabled indicator_led_enabled_ref;
            sinsei_umiusi_control::cmd::main_power::Enabled main_power_enabled_ref;
            sinsei_umiusi_control::cmd::headlights::HighBeamEnabled high_beam_enabled_ref;
            sinsei_umiusi_control::cmd::headlights::LowBeamEnabled low_beam_enabled_ref;
            sinsei_umiusi_control::cmd::headlights::IrEnabled ir_enabled_ref;

            std::array<sinsei_umiusi_control::cmd::thruster::esc::Enabled, 4> esc_enabled_ref;
            std::array<sinsei_umiusi_control::cmd::thruster::servo::Enabled, 4> servo_enabled_ref;

            sinsei_umiusi_control::cmd::led_tape::Color led_tape_color_ref;
            sinsei_umiusi_control::cmd::attitude::Orientation target_orientation_ref;
            sinsei_umiusi_control::cmd::attitude::Velocity target_velocity_ref;
        };
        // Publishers for states
        struct Publishers {
            rclcpp::Publisher<msg::MainPowerState>::SharedPtr main_power_state_publisher;
            rclcpp::Publisher<msg::ImuState>::SharedPtr imu_state_publisher;
            rclcpp::Publisher<msg::ThrusterStateAll>::SharedPtr thruster_state_all_publisher;
            std::array<rclcpp::Publisher<msg::EscState>::SharedPtr, 4> esc_state_publishers;
        };
        Command cmd;
        Publishers pub;
    };

  private:
    Input input;
    Output output;

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
