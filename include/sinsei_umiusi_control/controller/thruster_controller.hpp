#ifndef SINSEI_UMIUSI_CONTROL_THRUSTER_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_THRUSTER_CONTROLLER_HPP

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/hardware_interface/handle.hpp>
#include <vector>

#include "sinsei_umiusi_control/cmd/thruster.hpp"
#include "sinsei_umiusi_control/state/thruster.hpp"
#include "sinsei_umiusi_control/util/interface_accessor.hpp"
#include "sinsei_umiusi_control/util/thruster_mode.hpp"

namespace sinsei_umiusi_control::controller {

class ThrusterController : public controller_interface::ChainableControllerInterface {
  private:
    // Command interfaces (in)
    sinsei_umiusi_control::cmd::thruster::ServoEnabled servo_enabled;
    sinsei_umiusi_control::cmd::thruster::EscEnabled esc_enabled;
    sinsei_umiusi_control::cmd::thruster::Angle angle;
    sinsei_umiusi_control::cmd::thruster::DutyCycle duty_cycle;

    // State interfaces (out)
    sinsei_umiusi_control::state::thruster::Rpm rpm;

    sinsei_umiusi_control::util::interface_accessor::InterfaceDataContainer command_interface_data;
    sinsei_umiusi_control::util::interface_accessor::InterfaceDataContainer state_interface_data;
    sinsei_umiusi_control::util::interface_accessor::InterfaceDataContainer ref_interface_data;

    // Thruster ID (1~4)
    uint8_t id;

    util::ThrusterMode mode;

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