#include "sinsei_umiusi_control/gate_controller.hpp"

namespace suc = sinsei_umiusi_control;
namespace rlc = rclcpp_lifecycle;
namespace hif = hardware_interface;
namespace cif = controller_interface;

auto suc::GateController::command_interface_configuration() const -> cif::InterfaceConfiguration {
    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::ALL,
        {},
    };
}

auto suc::GateController::state_interface_configuration() const -> cif::InterfaceConfiguration {
    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::ALL,
        {},
    };
}

auto suc::GateController::on_init() -> cif::CallbackReturn { return cif::CallbackReturn::SUCCESS; }

auto suc::GateController::on_configure(const rlc::State & /*pervious_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto suc::GateController::on_activate(const rlc::State & /*previous_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto suc::GateController::on_deactivate(const rlc::State & /*previous_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto suc::GateController::update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/
    ) -> cif::return_type {
    return cif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::GateController, controller_interface::ControllerInterface)