#include "sinsei_umiusi_control/app_controller.hpp"

namespace suc = sinsei_umiusi_control;
namespace rlc = rclcpp_lifecycle;
namespace hif = hardware_interface;
namespace cif = controller_interface;

auto suc::AppController::command_interface_configuration() const -> cif::InterfaceConfiguration {
    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::ALL,
        {},
    };
}

auto suc::AppController::state_interface_configuration() const -> cif::InterfaceConfiguration {
    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::ALL,
        {},
    };
}

auto suc::AppController::on_init() -> cif::CallbackReturn { return cif::CallbackReturn::SUCCESS; }

auto suc::AppController::on_configure(const rlc::State & /*pervious_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto suc::AppController::on_activate(const rlc::State & /*previous_state*/) -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto suc::AppController::on_deactivate(const rlc::State & /*previous_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto suc::AppController::on_export_reference_interfaces() -> std::vector<hif::CommandInterface> {
    auto interfaces = std::vector<hif::CommandInterface>{};
    return interfaces;
}

auto suc::AppController::on_export_state_interfaces() -> std::vector<hif::StateInterface> {
    auto interfaces = std::vector<hif::StateInterface>{};
    return interfaces;
}

auto suc::AppController::on_set_chained_mode(bool /*chained_mode*/) -> bool { return true; };

auto suc::AppController::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) -> cif::return_type {
    return cif::return_type::OK;
}

auto suc::AppController::update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) -> cif::return_type {
    return cif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::AppController, controller_interface::ChainableControllerInterface)