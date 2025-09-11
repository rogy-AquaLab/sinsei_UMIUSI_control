#ifndef SINSEI_UMIUSI_CONTROL_THRUSTER_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_THRUSTER_CONTROLLER_HPP

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/hardware_interface/handle.hpp>
#include <rclcpp/subscription.hpp>
#include <vector>

#include "sinsei_umiusi_control/cmd/thruster/esc.hpp"
#include "sinsei_umiusi_control/cmd/thruster/servo.hpp"
#include "sinsei_umiusi_control/msg/thruster_output.hpp"
#include "sinsei_umiusi_control/msg/thruster_output_all.hpp"
#include "sinsei_umiusi_control/state/thruster/esc.hpp"
#include "sinsei_umiusi_control/state/thruster/servo.hpp"
#include "sinsei_umiusi_control/util/interface_accessor.hpp"
#include "sinsei_umiusi_control/util/thruster_mode.hpp"

namespace sinsei_umiusi_control::controller {

class ThrusterController : public controller_interface::ChainableControllerInterface {
  public:
    struct Input {
        // Command interfaces (in)
        struct Command {
            cmd::thruster::esc::Enabled esc_enabled;
            cmd::thruster::esc::DutyCycle esc_duty_cycle;
            cmd::thruster::servo::Enabled servo_enabled;
            cmd::thruster::servo::Angle servo_angle;
        };
        // State interfaces (in)
        struct State {
            state::thruster::esc::Rpm esc_rpm;
            state::thruster::esc::Voltage esc_voltage;
            state::thruster::esc::WaterLeaked esc_water_leaked;
        };
        // Subscribers for command inputs
        struct Subscriber {
            rclcpp::Subscription<msg::ThrusterOutput>::SharedPtr thruster_output;
            rclcpp::Subscription<msg::ThrusterOutputAll>::SharedPtr thruster_output_all;
        };
        Command cmd;
        State state;
        Subscriber sub;
    };
    struct Output {
        // Command interfaces (out)
        struct Command {
            cmd::thruster::esc::Enabled esc_enabled;
            cmd::thruster::esc::DutyCycle esc_duty_cycle;
            cmd::thruster::servo::Enabled servo_enabled;
            cmd::thruster::servo::Angle servo_angle;
        };
        // State interfaces (out)
        struct State {
            state::thruster::esc::Enabled esc_enabled;
            state::thruster::esc::DutyCycle esc_duty_cycle;
            state::thruster::servo::Enabled servo_enabled;
            state::thruster::servo::Angle servo_angle;
        };
        Command cmd;
        State state;
    };

  private:
    Input input;
    Output output;

    util::interface_accessor::InterfaceDataContainer command_interface_data;
    util::interface_accessor::InterfaceDataContainer state_interface_data;
    util::interface_accessor::InterfaceDataContainer ref_interface_data;

    // Thruster mode (CAN or Direct)
    util::ThrusterMode mode;
    // Thruster hardware component ID (1~4)
    uint8_t id;
    // Thruster direction (true for forward, false for reverse)
    bool is_forward;
    // Maximum duty cycle (0.0 to 1.0)
    double max_duty;

  public:
    ThrusterController() = default;

    auto command_interface_configuration() const
        -> controller_interface::InterfaceConfiguration override;
    auto state_interface_configuration() const
        -> controller_interface::InterfaceConfiguration override;
    auto on_init() -> CallbackReturn override;
    auto on_configure(const rclcpp_lifecycle::State & previous_state) -> CallbackReturn override;
    auto on_export_reference_interfaces()
        -> std::vector<hardware_interface::CommandInterface> override;
    auto on_export_state_interfaces() -> std::vector<hardware_interface::StateInterface> override;
    auto on_set_chained_mode(bool chained_mode) -> bool override;
    auto update_reference_from_subscribers(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) -> controller_interface::return_type override;
    auto update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> controller_interface::return_type override;
};
}  // namespace sinsei_umiusi_control::controller

#endif  // SINSEI_UMIUSI_CONTROL_THRUSTER_CONTROLLER_HPP
