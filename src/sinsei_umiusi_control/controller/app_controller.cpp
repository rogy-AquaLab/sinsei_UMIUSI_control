#include "sinsei_umiusi_control/controller/app_controller.hpp"

#include "sinsei_umiusi_control/util.hpp"

namespace succ = sinsei_umiusi_control::controller;
namespace suc_util = sinsei_umiusi_control::util;
namespace rlc = rclcpp_lifecycle;
namespace hif = hardware_interface;
namespace cif = controller_interface;

auto succ::AppController::command_interface_configuration() const -> cif::InterfaceConfiguration {
    std::vector<std::string> cmd_names(
        std::begin(this->cmd_interface_names), std::end(this->cmd_interface_names));

    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        cmd_names,
    };
}

auto succ::AppController::state_interface_configuration() const -> cif::InterfaceConfiguration {
    std::vector<std::string> state_names(
        std::begin(this->state_interface_names), std::end(this->state_interface_names));

    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        state_names,
    };
}

auto succ::AppController::on_init() -> cif::CallbackReturn {
    this->interface_helper_ =
        std::make_unique<InterfaceAccessHelper<rclcpp_lifecycle::LifecycleNode, 8, 6>>(
            this->get_node().get(), this->command_interfaces_, this->cmd_interface_names,
            this->state_interfaces_, this->state_interface_names);

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
    // TODO: メンバ変数を`target_orientation`と`target_velocity`のinterfaceにセットする

    constexpr auto orientation_x_index =
        suc_util::get_index("imu/imu/orientation_raw.x", state_interface_names);
    constexpr auto orientation_y_index =
        suc_util::get_index("imu/imu/orientation_raw.y", state_interface_names);
    constexpr auto orientation_z_index =
        suc_util::get_index("imu/imu/orientation_raw.z", state_interface_names);
    constexpr auto velocity_x_index =
        suc_util::get_index("imu/imu/velocity_raw.x", state_interface_names);
    constexpr auto velocity_y_index =
        suc_util::get_index("imu/imu/velocity_raw.y", state_interface_names);
    constexpr auto velocity_z_index =
        suc_util::get_index("imu/imu/velocity_raw.z", state_interface_names);

    this->interface_helper_->get_state_value(orientation_x_index, this->imu_orientation.x);
    this->interface_helper_->get_state_value(orientation_y_index, this->imu_orientation.y);
    this->interface_helper_->get_state_value(orientation_z_index, this->imu_orientation.z);
    this->interface_helper_->get_state_value(velocity_x_index, this->imu_velocity.x);
    this->interface_helper_->get_state_value(velocity_y_index, this->imu_velocity.y);
    this->interface_helper_->get_state_value(velocity_z_index, this->imu_velocity.z);

    return cif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::AppController,
    controller_interface::ChainableControllerInterface)