#include "sinsei_umiusi_control/controller/gate_controller.hpp"

#include <rclcpp/logging.hpp>

#include "sinsei_umiusi_control/util.hpp"

namespace succ = sinsei_umiusi_control::controller;
namespace suc_util = sinsei_umiusi_control::util;
namespace rlc = rclcpp_lifecycle;
namespace hif = hardware_interface;
namespace cif = controller_interface;

auto succ::GateController::command_interface_configuration() const -> cif::InterfaceConfiguration {
    std::vector<std::string> cmd_names(
        std::begin(this->cmd_interface_names), std::end(this->cmd_interface_names));

    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        cmd_names,
    };
}

auto succ::GateController::state_interface_configuration() const -> cif::InterfaceConfiguration {
    std::vector<std::string> state_names(
        std::begin(this->state_interface_names), std::end(this->state_interface_names));

    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        state_names,
    };
}

auto succ::GateController::on_init() -> cif::CallbackReturn {
    this->interface_helper_ = std::make_unique<
        InterfaceAccessHelper<rclcpp_lifecycle::LifecycleNode, cmd_size, state_size>>(
        this->get_node().get(), this->command_interfaces_, this->cmd_interface_names,
        this->state_interfaces_, this->state_interface_names);

    return cif::CallbackReturn::SUCCESS;
}

auto succ::GateController::on_configure(const rlc::State & /*pervious_state*/)
    -> cif::CallbackReturn {
    // TODO: 今後`indicator_led/indicator_led/enabled`以外の分も実装する
    this->indicator_led_enabled_subscriber =
        this->get_node()->create_subscription<std_msgs::msg::Bool>(
            "indicator_led_enabled", rclcpp::SystemDefaultsQoS(),
            [this](const std_msgs::msg::Bool::SharedPtr input) {
                this->indicator_led_enabled_ref.value = input->data;
            });
    return cif::CallbackReturn::SUCCESS;
}

auto succ::GateController::on_activate(const rlc::State & /*previous_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto succ::GateController::on_deactivate(const rlc::State & /*previous_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto succ::GateController::update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/
    ) -> cif::return_type {
    // TODO: 今後`indicator_led/indicator_led/enabled`以外の分も実装する
    constexpr auto indicator_led_enabled_index =
        suc_util::get_index("indicator_led/indicator_led/enabled", cmd_interface_names);

    this->interface_helper_->set_cmd_value(
        indicator_led_enabled_index, this->indicator_led_enabled_ref);

    return cif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::GateController, controller_interface::ControllerInterface)