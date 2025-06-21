#ifndef SINSEI_UMIUSI_CONTROL_APP_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_APP_CONTROLLER_HPP

#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/hardware_interface/handle.hpp"
#include "sinsei_umiusi_control/cmd/app.hpp"
#include "sinsei_umiusi_control/cmd/thruster.hpp"
#include "sinsei_umiusi_control/interface_access_helper.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"

namespace suc = sinsei_umiusi_control;

namespace sinsei_umiusi_control::controller {

class AppController : public controller_interface::ChainableControllerInterface {
  private:
    // Command interfaces (in)
    suc::cmd::app::Orientation target_orientation;
    suc::cmd::app::Velocity target_velocity;

    // State interfaces (out)
    suc::state::imu::Orientation imu_orientation;
    suc::state::imu::Velocity imu_velocity;

    static constexpr size_t cmd_size = 8;
    static constexpr const char * cmd_interface_names[cmd_size] = {
        "thruster_controller1/angle/angle", "thruster_controller1/thrust/thrust",
        "thruster_controller2/angle/angle", "thruster_controller2/thrust/thrust",
        "thruster_controller3/angle/angle", "thruster_controller3/thrust/thrust",
        "thruster_controller4/angle/angle", "thruster_controller4/thrust/thrust",
    };
    static constexpr size_t state_size = 6;
    static constexpr const char * state_interface_names[state_size] = {
        "imu/imu/orientation_raw.x", "imu/imu/orientation_raw.y", "imu/imu/orientation_raw.z",
        "imu/imu/velocity_raw.x",    "imu/imu/velocity_raw.y",    "imu/imu/velocity_raw.z",
    };

    std::unique_ptr<InterfaceAccessHelper<rclcpp_lifecycle::LifecycleNode, cmd_size, state_size>>
        interface_helper_;

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