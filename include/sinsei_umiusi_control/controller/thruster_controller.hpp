#ifndef SINSEI_UMIUSI_CONTROL_THRUSTER_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_THRUSTER_CONTROLLER_HPP

#include <cstddef>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/hardware_interface/handle.hpp"
#include "sinsei_umiusi_control/cmd/thruster.hpp"
#include "sinsei_umiusi_control/interface_access_helper.hpp"
#include "sinsei_umiusi_control/state/thruster.hpp"

namespace suc = sinsei_umiusi_control;

namespace sinsei_umiusi_control::controller {

enum ThrusterMode {
    Can,
    Direct,
};

class ThrusterController : public controller_interface::ChainableControllerInterface {
  private:
    // Command interfaces (in)
    suc::cmd::thruster::ServoEnabled servo_enabled;
    suc::cmd::thruster::EscEnabled esc_enabled;
    suc::cmd::thruster::Angle angle;
    suc::cmd::thruster::Thrust thrust;

    // State interfaces (out)
    suc::state::thruster::ServoCurrent servo_current;
    suc::state::thruster::Rpm rpm;

    // 正式なinterface名は前に`thruster(_direct)(1-4)/`がつくが、リストを静的にするため特別に省略している
    static constexpr size_t can_cmd_size = 4;
    static constexpr const char * can_cmd_interface_names[can_cmd_size] = {
        "servo/servo/enabled_raw",
        "servo/servo/angle_raw",
        "esc/esc/enabled_raw",
        "esc/esc/thrust_raw",
    };
    static constexpr size_t can_state_size = 2;
    static constexpr const char * can_state_interface_names[can_state_size] = {
        "servo/servo/servo_current_raw",
        "esc/esc/rpm_raw",
    };
    static constexpr size_t direct_cmd_size = 4;
    static constexpr const char * direct_cmd_interface_names[direct_cmd_size] = {
        "servo_direct/servo_direct/enabled_raw",
        "servo_direct/servo_direct/angle_raw",
        "esc_direct/esc_direct/enabled_raw",
        "esc_direct/esc_direct/thrust_raw",
    };
    // `thruster_mode`が`direct`のときState Interfaceは存在しないが、配列のサイズは0にできないので1としている
    static constexpr size_t direct_state_size = 1;
    static constexpr const char * direct_state_interface_names[direct_state_size] = {};

    std::unique_ptr<
        InterfaceAccessHelper<rclcpp_lifecycle::LifecycleNode, can_cmd_size, can_state_size>>
        can_interface_helper_;
    std::unique_ptr<
        InterfaceAccessHelper<rclcpp_lifecycle::LifecycleNode, direct_cmd_size, direct_state_size>>
        direct_interface_helper_;

    // Thruster ID (1~4)
    uint8_t id;

    ThrusterMode mode;

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