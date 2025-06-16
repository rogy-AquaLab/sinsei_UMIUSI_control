#include "sinsei_umiusi_control/controller/gate_controller.hpp"

namespace succ = sinsei_umiusi_control::controller;
namespace rlc = rclcpp_lifecycle;
namespace hif = hardware_interface;
namespace cif = controller_interface;

auto succ::GateController::command_interface_configuration() const -> cif::InterfaceConfiguration {
    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::ALL,
        {},
    };
}

auto succ::GateController::state_interface_configuration() const -> cif::InterfaceConfiguration {
    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::ALL,
        {},
    };
}

auto succ::GateController::on_init() -> cif::CallbackReturn { return cif::CallbackReturn::SUCCESS; }

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
    // `CommandInterface`にアクセスしやすいよう、メンバ変数に所有権を移しておく。
    // ※ これ以降、`command_interfaces_`から当該の`Loaned(Command|State)Interface`を取得することはない。

    // TODO: 今後`indicator_led/indicator_led/enabled`以外の分も実装する
    auto it = std::find_if(
        this->command_interfaces_.begin(), this->command_interfaces_.end(),
        [&](const auto & ifc) { return ifc.get_name() == "indicator_led/indicator_led/enabled"; });
    if (it != this->command_interfaces_.end()) {
        this->indicator_led_enabled.emplace(std::move(*it));
    } else {
        RCLCPP_ERROR(
            this->get_node()->get_logger(),
            "Failed to find command interface: indicator_led/indicator_led/enabled");
    }
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
    if (!this->indicator_led_enabled) {
        RCLCPP_ERROR(
            this->get_node()->get_logger(),
            "Command interface not initialized: indicator_led/indicator_led/enabled");
        return cif::return_type::ERROR;
    }
    auto res = this->indicator_led_enabled->set_value(
        *reinterpret_cast<double *>(&this->indicator_led_enabled_ref));
    if (!res) {
        RCLCPP_WARN(
            this->get_node()->get_logger(), "Failed to set command interface value: %s",
            this->indicator_led_enabled->get_name().c_str());
    }
    return cif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::GateController, controller_interface::ControllerInterface)