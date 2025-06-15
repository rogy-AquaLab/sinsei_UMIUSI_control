#include "sinsei_umiusi_control/controller/thruster_controller.hpp"

namespace succ = sinsei_umiusi_control::controller;
namespace rlc = rclcpp_lifecycle;
namespace hif = hardware_interface;
namespace cif = controller_interface;

auto succ::ThrusterController::command_interface_configuration() const
    -> cif::InterfaceConfiguration {
    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        {},
    };
}

auto succ::ThrusterController::state_interface_configuration() const
    -> cif::InterfaceConfiguration {
    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        {},
    };
}

auto succ::ThrusterController::on_init() -> cif::CallbackReturn {
    this->get_node()->declare_parameter("id", 1);
    this->id = this->get_node()->get_parameter("id").as_int();
    RCLCPP_INFO(get_node()->get_logger(), "Thruster ID: %ld", this->id);
    return cif::CallbackReturn::SUCCESS;
}

auto succ::ThrusterController::on_configure(const rlc::State & /*pervious_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto succ::ThrusterController::on_activate(const rlc::State & /*previous_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto succ::ThrusterController::on_deactivate(const rlc::State & /*previous_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto succ::ThrusterController::on_export_reference_interfaces()
    -> std::vector<hif::CommandInterface> {
    this->reference_interfaces_.resize(3);

    auto interfaces = std::vector<hif::CommandInterface>{};
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name() + std::string("/enabled"), "enabled",
        reinterpret_cast<double *>(&this->enabled)));
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name() + std::string("/angle"), "angle",
        reinterpret_cast<double *>(&this->angle)));
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name() + std::string("/thrust"), "thrust",
        reinterpret_cast<double *>(&this->thrust)));
    return interfaces;
}

auto succ::ThrusterController::on_export_state_interfaces() -> std::vector<hif::StateInterface> {
    auto interfaces = std::vector<hif::StateInterface>{};

    interfaces.emplace_back(hif::StateInterface(
        this->get_node()->get_name() + std::string("/servo_current"), "servo_current",
        reinterpret_cast<double *>(&this->servo_current)));
    interfaces.emplace_back(hif::StateInterface(
        this->get_node()->get_name() + std::string("/rpm"), "rpm",
        reinterpret_cast<double *>(&this->rpm)));
    return interfaces;
}

auto succ::ThrusterController::on_set_chained_mode(bool /*chained_mode*/) -> bool { return true; }

auto succ::ThrusterController::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) -> cif::return_type {
    return cif::return_type::OK;
}

auto succ::ThrusterController::update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) -> cif::return_type {
    return cif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::ThrusterController,
    controller_interface::ChainableControllerInterface)