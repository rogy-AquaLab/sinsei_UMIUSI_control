#include "sinsei_umiusi_control/controller/app_controller.hpp"

#include <cstddef>
#include <rclcpp/logging.hpp>

#include "sinsei_umiusi_control/util/interface_accessor.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"

namespace succ = sinsei_umiusi_control::controller;
namespace suc_util = sinsei_umiusi_control::util;
namespace rlc = rclcpp_lifecycle;
namespace hif = hardware_interface;
namespace cif = controller_interface;

auto succ::AppController::command_interface_configuration() const -> cif::InterfaceConfiguration {
    auto cmd_names = std::vector<std::string>{};
    for (const auto & [name, _] : this->command_interface_data) {
        cmd_names.push_back(name);
    }

    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        cmd_names,
    };
}

auto succ::AppController::state_interface_configuration() const -> cif::InterfaceConfiguration {
    auto state_names = std::vector<std::string>{};
    for (const auto & [name, _] : this->state_interface_data) {
        state_names.push_back(name);
    }

    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        state_names,
    };
}

auto succ::AppController::on_init() -> cif::CallbackReturn {
    constexpr const std::string_view CMD_INTERFACE_NAMES[8] = {
        "thruster_controller1/angle", "thruster_controller1/duty_cycle",
        "thruster_controller2/angle", "thruster_controller2/duty_cycle",
        "thruster_controller3/angle", "thruster_controller3/duty_cycle",
        "thruster_controller4/angle", "thruster_controller4/duty_cycle",
    };
    for (size_t i = 0; i < 4; ++i) {
        this->command_interface_data.emplace_back(
            CMD_INTERFACE_NAMES[i * 2 + 0], util::to_interface_data_ptr(this->thruster_angles[i]));
        this->command_interface_data.emplace_back(
            CMD_INTERFACE_NAMES[i * 2 + 1],
            util::to_interface_data_ptr(this->thruster_duty_cycles[i]));
    }

    constexpr const std::string_view STATE_INTERFACE_NAMES[6] = {
        "imu/orientation.x", "imu/orientation.y", "imu/orientation.z",
        "imu/velocity.x",    "imu/velocity.y",    "imu/velocity.z",
    };
    for (size_t i = 0; i < 2; ++i) {
        this->state_interface_data.emplace_back(
            STATE_INTERFACE_NAMES[i * 3 + 0], util::to_interface_data_ptr(this->imu_orientation.x));
        this->state_interface_data.emplace_back(
            STATE_INTERFACE_NAMES[i * 3 + 1], util::to_interface_data_ptr(this->imu_orientation.y));
        this->state_interface_data.emplace_back(
            STATE_INTERFACE_NAMES[i * 3 + 2], util::to_interface_data_ptr(this->imu_orientation.z));
    }

    constexpr const std::string_view REF_INTERFACE_NAMES[6] = {
        "target_orientation.x", "target_orientation.y", "target_orientation.z",
        "target_velocity.x",    "target_velocity.y",    "target_velocity.z",
    };
    for (size_t i = 0; i < 2; ++i) {
        this->ref_interface_data.emplace_back(
            REF_INTERFACE_NAMES[i * 3 + 0],
            util::to_interface_data_ptr(this->target_orientation.x));
        this->ref_interface_data.emplace_back(
            REF_INTERFACE_NAMES[i * 3 + 1],
            util::to_interface_data_ptr(this->target_orientation.y));
        this->ref_interface_data.emplace_back(
            REF_INTERFACE_NAMES[i * 3 + 2],
            util::to_interface_data_ptr(this->target_orientation.z));
    }

    return cif::CallbackReturn::SUCCESS;
}

auto succ::AppController::on_export_reference_interfaces() -> std::vector<hif::CommandInterface> {
    // To avoid bug in ros2 control. `reference_interfaces_` is actually not used.
    this->reference_interfaces_.resize(this->ref_interface_data.size());

    auto interfaces = std::vector<hif::CommandInterface>{};
    for (auto & [name, data] : this->ref_interface_data) {
        interfaces.emplace_back(hif::CommandInterface(this->get_node()->get_name(), name, data));
    }
    return interfaces;
}

auto succ::AppController::on_export_state_interfaces() -> std::vector<hif::StateInterface> {
    auto interfaces = std::vector<hif::StateInterface>{};
    for (auto & [name, data] : this->state_interface_data) {
        interfaces.emplace_back(hif::StateInterface(this->get_node()->get_name(), name, data));
    }
    return interfaces;
}

auto succ::AppController::on_set_chained_mode(bool /*chained_mode*/) -> bool { return true; };

auto succ::AppController::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) -> cif::return_type {
    return cif::return_type::OK;
}

auto succ::AppController::update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) -> cif::return_type {
    // 状態を取得
    auto res = util::interface_accessor::get_states_from_loaned_interfaces(
        this->state_interfaces_, this->state_interface_data);
    if (!res) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_node()->get_logger(), *this->get_node()->get_clock(), DURATION,
            "Failed to get value of state interfaces");
    }

    // 姿勢制御の関数を呼び出す
    this->compute_outputs();

    // コマンドを送信
    res = util::interface_accessor::set_commands_to_loaned_interfaces(
        this->command_interfaces_, this->command_interface_data);
    if (!res) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_node()->get_logger(), *this->get_node()->get_clock(), DURATION,
            "Failed to set value for command interfaces");
    }

    return cif::return_type::OK;
}

auto succ::AppController::compute_outputs() -> void {
    // TODO: PID制御などの処理はここに記述する
    // 現在はダミー
    for (size_t i = 0; i < 4; ++i) {
        this->thruster_angles[i].value = 0.0;
        this->thruster_duty_cycles[i].value = 0.0;
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::AppController,
    controller_interface::ChainableControllerInterface)
