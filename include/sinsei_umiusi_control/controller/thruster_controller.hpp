#ifndef SINSEI_UMIUSI_CONTROL_THRUSTER_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_THRUSTER_CONTROLLER_HPP

#include <controller_interface/chainable_controller_interface.hpp>
#include <cstddef>
#include <hardware_interface/hardware_interface/handle.hpp>
#include <vector>

#include "sinsei_umiusi_control/cmd/thruster.hpp"
#include "sinsei_umiusi_control/interface_access_helper.hpp"
#include "sinsei_umiusi_control/state/thruster.hpp"
#include "sinsei_umiusi_control/util/thruster_mode.hpp"

namespace sinsei_umiusi_control::controller {

class ThrusterController : public controller_interface::ChainableControllerInterface {
  private:
    // Command interfaces (in)
    sinsei_umiusi_control::cmd::thruster::ServoEnabled servo_enabled;
    sinsei_umiusi_control::cmd::thruster::EscEnabled esc_enabled;
    sinsei_umiusi_control::cmd::thruster::Angle angle;
    sinsei_umiusi_control::cmd::thruster::Thrust thrust;

    // State interfaces (out)
    sinsei_umiusi_control::state::thruster::Rpm rpm;

    // 正式なinterface名は前に`thruster(_direct)(1-4)/`がつくが、リストを静的にするため特別に省略している
    static constexpr size_t CAN_CMD_SIZE = 4;
    static constexpr const char * CAN_CMD_INTERFACE_NAMES[CAN_CMD_SIZE] = {
        "servo/enabled_raw",
        "servo/angle_raw",
        "esc/enabled_raw",
        "esc/thrust_raw",
    };
    static constexpr size_t CAN_STATE_SIZE = 1;
    static constexpr const char * CAN_STATE_INTERFACE_NAMES[CAN_STATE_SIZE] = {
        "esc/rpm_raw",
    };
    static constexpr size_t DIRECT_CMD_SIZE = 4;
    static constexpr const char * DIRECT_CMD_INTERFACE_NAMES[DIRECT_CMD_SIZE] = {
        "servo_direct/enabled_raw",
        "servo_direct/angle_raw",
        "esc_direct/enabled_raw",
        "esc_direct/thrust_raw",
    };
    // `thruster_mode`が`direct`のときState Interfaceは存在しないが、配列のサイズは0にできないので1としている
    static constexpr size_t DIRECT_STATE_SIZE = 1;
    static constexpr const char * DIRECT_STATE_INTERFACE_NAMES[DIRECT_STATE_SIZE] = {};

    std::unique_ptr<InterfaceAccessHelper<CAN_CMD_SIZE, CAN_STATE_SIZE>> can_interface_helper;
    std::unique_ptr<InterfaceAccessHelper<DIRECT_CMD_SIZE, DIRECT_STATE_SIZE>>
        direct_interface_helper;

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