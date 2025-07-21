#include "sinsei_umiusi_control/controller/app_controller.hpp"

#include <cstddef>
#include <optional>
#include <rclcpp/logging.hpp>

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
            CMD_INTERFACE_NAMES[i * 2], util::to_interface_data_ptr(this->thruster_angles[i]));
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
            STATE_INTERFACE_NAMES[i * 3], util::to_interface_data_ptr(this->imu_orientation.x));
        this->state_interface_data.emplace_back(
            STATE_INTERFACE_NAMES[i * 3 + 1], util::to_interface_data_ptr(this->imu_orientation.y));
        this->state_interface_data.emplace_back(
            STATE_INTERFACE_NAMES[i * 3 + 2], util::to_interface_data_ptr(this->imu_orientation.z));
    }

    constexpr const std::string_view REFERENCE_INTERFACE_NAMES[6] = {
        "target_orientation.x", "target_orientation.y", "target_orientation.z",
        "target_velocity.x",    "target_velocity.y",    "target_velocity.z",
    };
    for (size_t i = 0; i < 2; ++i) {
        this->reference_interface_data.emplace_back(
            REFERENCE_INTERFACE_NAMES[i * 3],
            util::to_interface_data_ptr(this->target_orientation.x));
        this->reference_interface_data.emplace_back(
            REFERENCE_INTERFACE_NAMES[i * 3 + 1],
            util::to_interface_data_ptr(this->target_orientation.y));
        this->reference_interface_data.emplace_back(
            REFERENCE_INTERFACE_NAMES[i * 3 + 2],
            util::to_interface_data_ptr(this->target_orientation.z));
    }

    return cif::CallbackReturn::SUCCESS;
}

auto succ::AppController::on_configure(const rlc::State & /*pervious_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto succ::AppController::on_activate(const rlc::State & /*previous_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto succ::AppController::on_deactivate(const rlc::State & /*previous_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto succ::AppController::on_export_reference_interfaces() -> std::vector<hif::CommandInterface> {
    this->reference_interfaces_.resize(6);

    auto interfaces = std::vector<hif::CommandInterface>{};
    for (auto & [name, data] : this->reference_interface_data) {
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
    for (size_t i = 0; i < this->state_interface_data.size(); ++i) {
        auto & [name, data] = this->state_interface_data[i];
        auto res = this->state_interfaces_.at(i).get_optional();
        if (!res) {
            auto node = this->get_node();
            constexpr auto DURATION = 3000;  // ms
            RCLCPP_WARN_THROTTLE(
                node->get_logger(), *node->get_clock(), DURATION,
                "Failed to set value for state interface: %s", name.c_str());
        }
        *data = res.value();
    }

    // 姿勢制御の関数を呼び出す
    this->compute_outputs();

    // コマンドを送信
    for (size_t i = 0; i < this->command_interface_data.size(); ++i) {
        const auto & [name, data] = this->command_interface_data[i];
        auto res = this->command_interfaces_.at(i).set_value(*data);
        if (!res) {
            auto node = this->get_node();
            constexpr auto DURATION = 3000;  // ms
            RCLCPP_WARN_THROTTLE(
                node->get_logger(), *node->get_clock(), DURATION,
                "Failed to set value for command interface: %s", name.c_str());
        }
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