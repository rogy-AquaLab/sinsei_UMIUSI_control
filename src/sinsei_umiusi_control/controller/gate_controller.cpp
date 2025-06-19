#include "sinsei_umiusi_control/controller/gate_controller.hpp"

namespace succ = sinsei_umiusi_control::controller;
namespace rlc = rclcpp_lifecycle;
namespace hif = hardware_interface;
namespace cif = controller_interface;

auto succ::GateController::command_interface_configuration() const -> cif::InterfaceConfiguration {
    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        {
            "indicator_led/indicator_led/enabled",
            "main_power/main_power/enabled",
            "led_tape/led_tape/color",
            "headlights/headlights/high_beam_enabled",
            "headlights/headlights/low_beam_enabled",
            "headlights/headlights/ir_enabled",
            "usb_camera/usb_camera/config",
            "raspi_camera/raspi_camera/config",
            "thruster_controller1/servo_enabled/servo_enabled",
            "thruster_controller2/servo_enabled/servo_enabled",
            "thruster_controller3/servo_enabled/servo_enabled",
            "thruster_controller4/servo_enabled/servo_enabled",
            "thruster_controller1/esc_enabled/esc_enabled",
            "thruster_controller2/esc_enabled/esc_enabled",
            "thruster_controller3/esc_enabled/esc_enabled",
            "thruster_controller4/esc_enabled/esc_enabled",
            "app_controller/target_orientation.x/target_orientation.x",
            "app_controller/target_orientation.y/target_orientation.y",
            "app_controller/target_orientation.z/target_orientation.z",
            "app_controller/target_velocity.x/target_velocity.x",
            "app_controller/target_velocity.y/target_velocity.y",
            "app_controller/target_velocity.z/target_velocity.z",
        },
    };
}

auto succ::GateController::state_interface_configuration() const -> cif::InterfaceConfiguration {
    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        {
            "main_power/main_power/battery_current",
            "main_power/main_power/battery_voltage",
            "main_power/main_power/temperature",
            "main_power/main_power/water_leaked",
            "thruster_controller1/servo_current/servo_current",
            "thruster_controller2/servo_current/servo_current",
            "thruster_controller3/servo_current/servo_current",
            "thruster_controller4/servo_current/servo_current",
            "thruster_controller1/rpm/rpm",
            "thruster_controller2/rpm/rpm",
            "thruster_controller3/rpm/rpm",
            "thruster_controller4/rpm/rpm",
            "app_controller/imu_orientation.x/imu_orientation.x",
            "app_controller/imu_orientation.y/imu_orientation.y",
            "app_controller/imu_orientation.z/imu_orientation.z",
            "app_controller/imu_velocity.x/imu_velocity.x",
            "app_controller/imu_velocity.y/imu_velocity.y",
            "app_controller/imu_velocity.z/imu_velocity.z",
            "imu/imu/temperature",
            "usb_camera/usb_camera/image",
            "raspi_camera/raspi_camera/image",
        },
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
    /*if (!this->indicator_led_enabled) {
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
    }*/
    return cif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::GateController, controller_interface::ControllerInterface)