#ifndef SINSEI_UMIUSI_CONTROL_APP_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_APP_CONTROLLER_HPP

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/hardware_interface/handle.hpp>
#include <vector>

#include "sinsei_umiusi_control/cmd/app.hpp"
#include "sinsei_umiusi_control/cmd/thruster.hpp"
#include "sinsei_umiusi_control/interface_access_helper.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"

namespace sinsei_umiusi_control::controller {

class AppController : public controller_interface::ChainableControllerInterface {
  private:
    // Command interfaces (in)
    sinsei_umiusi_control::cmd::app::Orientation target_orientation;
    sinsei_umiusi_control::cmd::app::Velocity target_velocity;

    // State interfaces (out)
    sinsei_umiusi_control::state::imu::Orientation imu_orientation;
    sinsei_umiusi_control::state::imu::Velocity imu_velocity;

    std::array<sinsei_umiusi_control::cmd::thruster::Angle, 4> thruster_angles;
    std::array<sinsei_umiusi_control::cmd::thruster::Thrust, 4> thruster_thrusts;

    auto compute_outputs() -> void;

    static constexpr size_t CMD_SIZE = 8;
    static constexpr const char * CMD_INTERFACE_NAMES[CMD_SIZE] = {
        "thruster_controller1/angle",  "thruster_controller1/thrust", "thruster_controller2/angle",
        "thruster_controller2/thrust", "thruster_controller3/angle",  "thruster_controller3/thrust",
        "thruster_controller4/angle",  "thruster_controller4/thrust",
    };
    static constexpr size_t STATE_SIZE = 6;
    static constexpr const char * STATE_INTERFACE_NAMES[STATE_SIZE] = {
        "imu/orientation_raw.x", "imu/orientation_raw.y", "imu/orientation_raw.z",
        "imu/velocity_raw.x",    "imu/velocity_raw.y",    "imu/velocity_raw.z",
    };

    std::unique_ptr<InterfaceAccessHelper<CMD_SIZE, STATE_SIZE>> interface_helper;

  public:
    AppController() = default;

    auto command_interface_configuration() const
        -> controller_interface::InterfaceConfiguration override;
    auto state_interface_configuration() const
        -> controller_interface::InterfaceConfiguration override;
    auto on_init() -> CallbackReturn override;
    auto on_configure(const rclcpp_lifecycle::State & previous_state) -> CallbackReturn override;
    auto on_activate(const rclcpp_lifecycle::State & previous_state) -> CallbackReturn override;
    auto on_deactivate(const rclcpp_lifecycle::State & previous_state) -> CallbackReturn override;
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

#endif  // SINSEI_UMIUSI_CONTROL_APP_CONTROLLER_HPP