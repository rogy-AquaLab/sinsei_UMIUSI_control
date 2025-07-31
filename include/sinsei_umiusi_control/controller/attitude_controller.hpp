#ifndef SINSEI_UMIUSI_CONTROL_ATTITUDE_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_ATTITUDE_CONTROLLER_HPP

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/hardware_interface/handle.hpp>
#include <vector>

#include "sinsei_umiusi_control/cmd/attitude.hpp"
#include "sinsei_umiusi_control/cmd/thruster.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"
#include "sinsei_umiusi_control/state/thruster.hpp"
#include "sinsei_umiusi_control/util/interface_accessor.hpp"
#include "sinsei_umiusi_control/util/thruster_mode.hpp"

namespace sinsei_umiusi_control::controller {

class AttitudeController : public controller_interface::ChainableControllerInterface {
  private:
    struct Input {
        struct Command {  // Command interfaces (in)
            cmd::attitude::Orientation target_orientation;
            cmd::attitude::Velocity target_velocity;
        };
        struct State {  // State interfaces (out)
            state::imu::Quaternion imu_quaternion;
            state::imu::Velocity imu_velocity;
        };
        Command cmd;
        State state;
    };

    struct Output {
        struct Command {  // Command interfaces (out)
            std::array<cmd::thruster::Angle, 4> thruster_angles;
            std::array<cmd::thruster::DutyCycle, 4> thruster_duty_cycles;
        };
        struct State {  // State interfaces (in)
            std::array<state::thruster::Rpm, 4> thruster_rpms;
        };
        Command cmd;
        State state;
    };

    Input input;
    Output output;

    util::ThrusterMode thruster_mode;

    auto compute_outputs(const rclcpp::Time & time, const rclcpp::Duration & period) -> void;

    util::interface_accessor::InterfaceDataContainer command_interface_data;
    util::interface_accessor::InterfaceDataContainer state_interface_data;
    util::interface_accessor::InterfaceDataContainer ref_interface_data;

  public:
    AttitudeController() = default;

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

#endif  // SINSEI_UMIUSI_CONTROL_ATTITUDE_CONTROLLER_HPP
