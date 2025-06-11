#ifndef SINSEI_UMIUSI_CONTROL_APP_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_APP_CONTROLLER_HPP

#include <memory>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sinsei_umiusi_control/cmd/app.hpp"
#include "sinsei_umiusi_control/cmd/thruster.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"

namespace suc = sinsei_umiusi_control;

namespace sinsei_umiusi_control::controller {

class AppController : public controller_interface::ChainableControllerInterface {
  private:
    suc::cmd::app::Orientation target_orientation;
    suc::cmd::app::Velocity target_velocity;
    suc::cmd::thruster::Angle thruster1_angle;
    suc::cmd::thruster::Angle thruster2_angle;
    suc::cmd::thruster::Angle thruster3_angle;
    suc::cmd::thruster::Angle thruster4_angle;
    suc::cmd::thruster::Thrust thruster1_thrust;
    suc::cmd::thruster::Thrust thruster2_thrust;
    suc::cmd::thruster::Thrust thruster3_thrust;
    suc::cmd::thruster::Thrust thruster4_thrust;
    suc::state::imu::Orientation imu_orientation;
    suc::state::imu::Velocity imu_velocity;

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