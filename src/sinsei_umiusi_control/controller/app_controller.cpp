#include "sinsei_umiusi_control/controller/app_controller.hpp"

#include <controller_interface/controller_interface_base.hpp>
#include <cstddef>
#include <rclcpp/logging.hpp>
#include <string>

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
    this->get_node()->declare_parameter("thruster_mode", "unknown");

    return cif::CallbackReturn::SUCCESS;
}

auto succ::AppController::on_configure(const rlc::State & /*previous_state*/)
    -> cif::CallbackReturn {
    const auto mode_str = this->get_node()->get_parameter("thruster_mode").as_string();
    const auto mode_res = util::get_mode_from_str(mode_str);
    if (!mode_res) {
        RCLCPP_ERROR(this->get_node()->get_logger(), "Invalid thruster mode: %s", mode_str.c_str());
        return cif::CallbackReturn::ERROR;
    }
    this->thruster_mode = mode_res.value();

    RCLCPP_INFO(this->get_node()->get_logger(), "Thruster MODE: %s", mode_str.c_str());

    for (size_t i = 0; i < 4; ++i) {
        const auto prefix = "thruster_controller" + std::to_string(i + 1) + "/";
        this->command_interface_data.emplace_back(
            prefix + "angle", util::to_interface_data_ptr(this->thruster_angles[i]));
        this->command_interface_data.emplace_back(
            prefix + "duty_cycle", util::to_interface_data_ptr(this->thruster_duty_cycles[i]));
    }

    if (this->thruster_mode == util::ThrusterMode::Can) {
        // `can`モードのときは、RPMを取得するためのインターフェースを追加する。
        for (size_t i = 0; i < 4; ++i) {
            const auto id_str = std::to_string(i + 1);
            const auto prefix = "thruster_controller" + id_str + "/thruster" + id_str + "/";
            this->state_interface_data.emplace_back(
                prefix + "esc/rpm", util::to_interface_data_ptr(this->thruster_rpms[i]));
        }
    }
    this->state_interface_data.emplace_back(
        "imu/orientation.x", util::to_interface_data_ptr(this->imu_orientation.x));
    this->state_interface_data.emplace_back(
        "imu/orientation.y", util::to_interface_data_ptr(this->imu_orientation.y));
    this->state_interface_data.emplace_back(
        "imu/orientation.z", util::to_interface_data_ptr(this->imu_orientation.z));
    this->state_interface_data.emplace_back(
        "imu/velocity.x", util::to_interface_data_ptr(this->imu_velocity.x));
    this->state_interface_data.emplace_back(
        "imu/velocity.y", util::to_interface_data_ptr(this->imu_velocity.y));
    this->state_interface_data.emplace_back(
        "imu/velocity.z", util::to_interface_data_ptr(this->imu_velocity.z));

    this->ref_interface_data.emplace_back(
        "target_orientation.x", util::to_interface_data_ptr(this->target_orientation.x));
    this->ref_interface_data.emplace_back(
        "target_orientation.y", util::to_interface_data_ptr(this->target_orientation.y));
    this->ref_interface_data.emplace_back(
        "target_orientation.z", util::to_interface_data_ptr(this->target_orientation.z));
    this->ref_interface_data.emplace_back(
        "target_velocity.x", util::to_interface_data_ptr(this->target_velocity.x));
    this->ref_interface_data.emplace_back(
        "target_velocity.y", util::to_interface_data_ptr(this->target_velocity.y));
    this->ref_interface_data.emplace_back(
        "target_velocity.z", util::to_interface_data_ptr(this->target_velocity.z));

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
