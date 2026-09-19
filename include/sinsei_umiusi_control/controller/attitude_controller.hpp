#ifndef SINSEI_UMIUSI_CONTROL_ATTITUDE_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_ATTITUDE_CONTROLLER_HPP

#include <array>
#include <controller_interface/chainable_controller_interface.hpp>
#include <memory>
#include <optional>
#include <vector>

#include "sinsei_umiusi_control/cmd/attitude.hpp"
#include "sinsei_umiusi_control/cmd/thruster/esc.hpp"
#include "sinsei_umiusi_control/cmd/thruster/servo.hpp"
#include "sinsei_umiusi_control/controller/logic/logic_interface.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"
#include "sinsei_umiusi_control/state/thruster/esc.hpp"
#include "sinsei_umiusi_control/state/thruster/servo.hpp"
#include "sinsei_umiusi_control/util/interface_accessor.hpp"

namespace sinsei_umiusi_control::controller {

class AttitudeController : public controller_interface::ChainableControllerInterface {
  public:
    struct Input {
        // Command interfaces (in)
        struct Command {
            cmd::attitude::AttitudeTarget target_attitude;
            cmd::attitude::Velocity target_velocity;
        };
        // State interfaces (in)
        struct State {
            state::imu::Quaternion imu_quaternion;
            state::imu::Acceleration imu_acceleration;
            state::imu::AngularVelocity imu_angular_velocity;
            std::array<state::thruster::esc::Rpm, 4> esc_rpms;
            std::array<state::thruster::servo::CommandedAngle, 4> servo_commanded_angles;
            std::array<std::optional<state::thruster::servo::EstimatedAngle>, 4>
                servo_estimated_angles;
            std::array<state::thruster::servo::MaxAngularVelocity, 4> servo_max_angular_velocities;
        };
        Command cmd;
        State state;
    };

    struct Output {
        // Command interfaces (out)
        struct Command {
            std::array<cmd::thruster::esc::Thrust, 4> esc_thrusts;
            std::array<cmd::thruster::servo::Angle, 4> servo_angles;
        };
        Command cmd;
    };

    using Logic = logic::LogicInterface<Input, Output>;

  private:
    Input input;
    Output output;

    std::unique_ptr<Logic> logic;

    std::array<state::thruster::servo::EstimatedAngle, 4> servo_estimated_angle_interfaces;

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
