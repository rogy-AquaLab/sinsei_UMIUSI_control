#ifndef SINSEI_UMIUSI_CONTROL_THRUSTER_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_THRUSTER_CONTROLLER_HPP

#include <optional>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sinsei_umiusi_control/cmd/thruster.hpp"
#include "sinsei_umiusi_control/state/thruster.hpp"

namespace suc = sinsei_umiusi_control;

namespace sinsei_umiusi_control::controller {

class ThrusterController : public controller_interface::ChainableControllerInterface {
  private:
    // Command interfaces (in)
    suc::cmd::thruster::ServoEnabled servo_enabled;
    suc::cmd::thruster::EscEnabled esc_enabled;
    suc::cmd::thruster::Angle angle;
    suc::cmd::thruster::Thrust thrust;

    // Command interfaces (out)
    std::optional<hardware_interface::LoanedCommandInterface> servo_enabled_raw;
    std::optional<hardware_interface::LoanedCommandInterface> esc_enabled_raw;

    // State interfaces (in)
    std::optional<hardware_interface::LoanedStateInterface> servo_current_raw;
    std::optional<hardware_interface::LoanedStateInterface> rpm_raw;

    // State interfaces (out)
    suc::state::thruster::ServoCurrent servo_current;
    suc::state::thruster::Rpm rpm;

    // Thruster ID (1~4)
    uint8_t id;

  public:
    ThrusterController() = default;

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

#endif  // SINSEI_UMIUSI_CONTROL_THRUSTER_CONTROLLER_HPP