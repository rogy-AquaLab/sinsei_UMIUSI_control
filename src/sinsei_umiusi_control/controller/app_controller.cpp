#include "sinsei_umiusi_control/controller/app_controller.hpp"

#include "sinsei_umiusi_control/utility.hpp"

namespace succ = sinsei_umiusi_control::controller;
namespace suc_util = sinsei_umiusi_control::utility;
namespace rlc = rclcpp_lifecycle;
namespace hif = hardware_interface;
namespace cif = controller_interface;

auto succ::AppController::command_interface_configuration() const -> cif::InterfaceConfiguration {
    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        {},
    };
}

auto succ::AppController::state_interface_configuration() const -> cif::InterfaceConfiguration {
    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        {},
    };
}

auto succ::AppController::on_init() -> cif::CallbackReturn { return cif::CallbackReturn::SUCCESS; }

auto succ::AppController::on_configure(const rlc::State & /*pervious_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto succ::AppController::on_activate(const rlc::State & /*previous_state*/)
    -> cif::CallbackReturn {
    // `(Command|State)Interface`にアクセスしやすいよう、メンバ変数に所有権を移しておく。
    // ※ これ以降、`command_interfaces_`から当該の`Loaned(Command|State)Interface`を取得することはない。
    for (size_t i = 0; i < 4; i++) {
        auto angle_it_name = "thruster_controller" + std::to_string(i + 1) + "/angle";
        auto angle_it = suc_util::find_interface(angle_it_name, this->command_interfaces_);
        if (angle_it) {
            this->thruster_angle[i].emplace(std::move(*angle_it));
        } else {
            RCLCPP_ERROR(
                this->get_node()->get_logger(), "Failed to find command interface: %s",
                angle_it_name.c_str());
        }

        auto thrust_it_name = "thruster_controller" + std::to_string(i + 1) + "/thrust";
        auto thrust_it = suc_util::find_interface(thrust_it_name, this->command_interfaces_);
        if (thrust_it) {
            this->thruster_thrust[i].emplace(std::move(*thrust_it));
        } else {
            RCLCPP_ERROR(
                this->get_node()->get_logger(), "Failed to find command interface: %s",
                thrust_it_name.c_str());
        }
    }

    std::array<std::string, 3> axes = {"x", "y", "z"};

    for (size_t i = 0; i < 3; i++) {
        auto imu_orientation_name = "imu/imu/orientation_raw." + axes[i];
        auto imu_orientation_it =
            suc_util::find_interface(imu_orientation_name, this->state_interfaces_);
        if (imu_orientation_it) {
            this->imu_orientation_raw[i].emplace(std::move(*imu_orientation_it));
        } else {
            RCLCPP_ERROR(
                this->get_node()->get_logger(), "Failed to find state interface: %s",
                imu_orientation_name.c_str());
        }

        auto imu_velocity_name = "imu/imu/velocity_raw." + axes[i];
        auto imu_velocity_it = suc_util::find_interface(imu_velocity_name, this->state_interfaces_);
        if (imu_velocity_it) {
            this->imu_velocity_raw[i].emplace(std::move(*imu_velocity_it));
        } else {
            RCLCPP_ERROR(
                this->get_node()->get_logger(), "Failed to find state interface: %s",
                imu_velocity_name.c_str());
        }
    }
    return cif::CallbackReturn::SUCCESS;
}

auto succ::AppController::on_deactivate(const rlc::State & /*previous_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto succ::AppController::on_export_reference_interfaces() -> std::vector<hif::CommandInterface> {
    this->reference_interfaces_.resize(6);

    auto interfaces = std::vector<hif::CommandInterface>{};
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name() + std::string("/target_orientation.x"), "target_orientation.x",
        &this->target_orientation.x));
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name() + std::string("/target_orientation.y"), "target_orientation.y",
        &this->target_orientation.y));
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name() + std::string("/target_orientation.z"), "target_orientation.z",
        &this->target_orientation.z));
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name() + std::string("/target_velocity.x"), "target_velocity.x",
        &this->target_velocity.x));
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name() + std::string("/target_velocity.y"), "target_velocity.y",
        &this->target_velocity.y));
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name() + std::string("/target_velocity.z"), "target_velocity.z",
        &this->target_velocity.z));
    return interfaces;
}

auto succ::AppController::on_export_state_interfaces() -> std::vector<hif::StateInterface> {
    auto interfaces = std::vector<hif::StateInterface>{};
    interfaces.emplace_back(hif::StateInterface(
        this->get_node()->get_name() + std::string("/imu_orientation.x"), "imu_orientation.x",
        &this->imu_orientation.x));
    interfaces.emplace_back(hif::StateInterface(
        this->get_node()->get_name() + std::string("/imu_orientation.y"), "imu_orientation.y",
        &this->imu_orientation.y));
    interfaces.emplace_back(hif::StateInterface(
        this->get_node()->get_name() + std::string("/imu_orientation.z"), "imu_orientation.z",
        &this->imu_orientation.z));
    interfaces.emplace_back(hif::StateInterface(
        this->get_node()->get_name() + std::string("/imu_velocity.x"), "imu_velocity.x",
        &this->imu_velocity.x));
    interfaces.emplace_back(hif::StateInterface(
        this->get_node()->get_name() + std::string("/imu_velocity.y"), "imu_velocity.y",
        &this->imu_velocity.y));
    interfaces.emplace_back(hif::StateInterface(
        this->get_node()->get_name() + std::string("/imu_velocity.z"), "imu_velocity.z",
        &this->imu_velocity.z));
    return interfaces;
}

auto succ::AppController::on_set_chained_mode(bool /*chained_mode*/) -> bool { return true; };

auto succ::AppController::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) -> cif::return_type {
    return cif::return_type::OK;
}

auto succ::AppController::update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) -> cif::return_type {
    return cif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::AppController,
    controller_interface::ChainableControllerInterface)