#include "sinsei_umiusi_control/controller/gate_controller.hpp"

#include <rclcpp/logging.hpp>

#include "sinsei_umiusi_control/util/interface_accessor.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"

namespace succ = sinsei_umiusi_control::controller;
namespace suc_util = sinsei_umiusi_control::util;
namespace rlc = rclcpp_lifecycle;
namespace hif = hardware_interface;
namespace cif = controller_interface;

auto succ::GateController::command_interface_configuration() const -> cif::InterfaceConfiguration {
    auto cmd_names = std::vector<std::string>{};
    for (const auto & [name, _] : this->command_interface_data) {
        cmd_names.push_back(name);
    }

    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        cmd_names,
    };
}

auto succ::GateController::state_interface_configuration() const -> cif::InterfaceConfiguration {
    auto state_names = std::vector<std::string>{};
    for (const auto & [name, _] : this->state_interface_data) {
        state_names.push_back(name);
    }

    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        state_names,
    };
}

auto succ::GateController::on_init() -> cif::CallbackReturn {
    using util::to_interface_data_ptr;

    this->cmd = succ::GateController::Command{};
    this->state = succ::GateController::State{};

    {  // Command interface
        this->command_interface_data.emplace_back(
            "indicator_led/enabled", to_interface_data_ptr(this->cmd.indicator_led_enabled_ref));
        this->command_interface_data.emplace_back(
            "main_power/enabled", to_interface_data_ptr(this->cmd.main_power_enabled_ref));
        this->command_interface_data.emplace_back(
            "headlights/high_beam_enabled", to_interface_data_ptr(this->cmd.high_beam_enabled_ref));
        this->command_interface_data.emplace_back(
            "headlights/low_beam_enabled", to_interface_data_ptr(this->cmd.low_beam_enabled_ref));
        this->command_interface_data.emplace_back(
            "headlights/ir_enabled", to_interface_data_ptr(this->cmd.ir_enabled_ref));
        this->command_interface_data.emplace_back(
            "led_tape/color", to_interface_data_ptr(this->cmd.led_tape_color_ref));
        this->command_interface_data.emplace_back(
            "app_controller/target_orientation.x",
            to_interface_data_ptr(this->cmd.target_orientation_ref.x));
        this->command_interface_data.emplace_back(
            "app_controller/target_orientation.y",
            to_interface_data_ptr(this->cmd.target_orientation_ref.y));
        this->command_interface_data.emplace_back(
            "app_controller/target_orientation.z",
            to_interface_data_ptr(this->cmd.target_orientation_ref.z));
        this->command_interface_data.emplace_back(
            "app_controller/target_velocity.x",
            to_interface_data_ptr(this->cmd.target_velocity_ref.x));
        this->command_interface_data.emplace_back(
            "app_controller/target_velocity.y",
            to_interface_data_ptr(this->cmd.target_velocity_ref.y));
        this->command_interface_data.emplace_back(
            "app_controller/target_velocity.z",
            to_interface_data_ptr(this->cmd.target_velocity_ref.z));
        for (size_t i = 0; i < 4; ++i) {
            auto prefix = "thruster_controller" + std::to_string(i + 1) + "/";

            this->command_interface_data.emplace_back(
                prefix + "servo_enabled", to_interface_data_ptr(this->cmd.servo_enabled_ref));
            this->command_interface_data.emplace_back(
                prefix + "esc_enabled", to_interface_data_ptr(this->cmd.esc_enabled_ref));
        }

        // Subscribers
        const auto cmd_prefix = std::string("cmd/");
        const auto qos = rclcpp::SystemDefaultsQoS();
        this->sub.indicator_led_enabled_subscriber =
            this->get_node()->create_subscription<std_msgs::msg::Bool>(
                cmd_prefix + "indicator_led_enabled", qos,
                [this](const std_msgs::msg::Bool::SharedPtr input) {
                    this->cmd.indicator_led_enabled_ref.value = input->data;
                });
        this->sub.main_power_enabled_subscriber =
            this->get_node()->create_subscription<std_msgs::msg::Bool>(
                cmd_prefix + "main_power_enabled", qos,
                [this](const std_msgs::msg::Bool::SharedPtr input) {
                    this->cmd.main_power_enabled_ref.value = input->data;
                });
        this->sub.led_tape_color_subscriber =
            this->get_node()->create_subscription<std_msgs::msg::ColorRGBA>(
                cmd_prefix + "led_tape_color", qos,
                [this](const std_msgs::msg::ColorRGBA::SharedPtr input) {
                    // alphaは無視
                    this->cmd.led_tape_color_ref.red = static_cast<uint8_t>(input->r);
                    this->cmd.led_tape_color_ref.green = static_cast<uint8_t>(input->g);
                    this->cmd.led_tape_color_ref.blue = static_cast<uint8_t>(input->b);
                });
        this->sub.high_beam_enabled_subscriber =
            this->get_node()->create_subscription<std_msgs::msg::Bool>(
                cmd_prefix + "high_beam_enabled", qos,
                [this](const std_msgs::msg::Bool::SharedPtr input) {
                    this->cmd.high_beam_enabled_ref.value = input->data;
                });
        this->sub.low_beam_enabled_subscriber =
            this->get_node()->create_subscription<std_msgs::msg::Bool>(
                cmd_prefix + "low_beam_enabled", qos,
                [this](const std_msgs::msg::Bool::SharedPtr input) {
                    this->cmd.low_beam_enabled_ref.value = input->data;
                });
        this->sub.ir_enabled_subscriber =
            this->get_node()->create_subscription<std_msgs::msg::Bool>(
                cmd_prefix + "ir_enabled", qos, [this](const std_msgs::msg::Bool::SharedPtr input) {
                    this->cmd.ir_enabled_ref.value = input->data;
                });
        this->sub.servo_enabled_subscriber =
            this->get_node()->create_subscription<std_msgs::msg::Bool>(
                cmd_prefix + "servo_enabled", qos,
                [this](const std_msgs::msg::Bool::SharedPtr input) {
                    this->cmd.servo_enabled_ref.value = input->data;
                });
        this->sub.esc_enabled_subscriber =
            this->get_node()->create_subscription<std_msgs::msg::Bool>(
                cmd_prefix + "esc_enabled", qos,
                [this](const std_msgs::msg::Bool::SharedPtr input) {
                    this->cmd.esc_enabled_ref.value = input->data;
                });
        this->sub.target_orientation_subscriber =
            this->get_node()->create_subscription<geometry_msgs::msg::Vector3>(
                cmd_prefix + "target_orientation", qos,
                [this](const geometry_msgs::msg::Vector3::SharedPtr input) {
                    this->cmd.target_orientation_ref.x = input->x;
                    this->cmd.target_orientation_ref.y = input->y;
                    this->cmd.target_orientation_ref.z = input->z;
                });
        this->sub.target_velocity_subscriber =
            this->get_node()->create_subscription<geometry_msgs::msg::Vector3>(
                cmd_prefix + "target_velocity", qos,
                [this](const geometry_msgs::msg::Vector3::SharedPtr input) {
                    this->cmd.target_velocity_ref.x = input->x;
                    this->cmd.target_velocity_ref.y = input->y;
                    this->cmd.target_velocity_ref.z = input->z;
                });
    }
    {  // State interface
        this->state_interface_data.emplace_back(
            "main_power/battery_current", to_interface_data_ptr(this->state.battery_current.value));
        this->state_interface_data.emplace_back(
            "main_power/battery_voltage", to_interface_data_ptr(this->state.battery_voltage.value));
        this->state_interface_data.emplace_back(
            "main_power/temperature", to_interface_data_ptr(this->state.main_temperature.value));
        this->state_interface_data.emplace_back(
            "main_power/water_leaked", to_interface_data_ptr(this->state.water_leaked.value));
        this->state_interface_data.emplace_back(
            "imu/temperature", to_interface_data_ptr(this->state.imu_temperature.value));
        this->state_interface_data.emplace_back(
            "app_controller/imu/orientation.x",
            to_interface_data_ptr(this->state.imu_orientation.x));
        this->state_interface_data.emplace_back(
            "app_controller/imu/orientation.y",
            to_interface_data_ptr(this->state.imu_orientation.y));
        this->state_interface_data.emplace_back(
            "app_controller/imu/orientation.z",
            to_interface_data_ptr(this->state.imu_orientation.z));
        this->state_interface_data.emplace_back(
            "app_controller/imu/velocity.x", to_interface_data_ptr(this->state.imu_velocity.x));
        this->state_interface_data.emplace_back(
            "app_controller/imu/velocity.y", to_interface_data_ptr(this->state.imu_velocity.y));
        this->state_interface_data.emplace_back(
            "app_controller/imu/velocity.z", to_interface_data_ptr(this->state.imu_velocity.z));
        for (size_t i = 0; i < 4; ++i) {
            const auto id_str = std::to_string(i + 1);
            const auto prefix = "thruster_controller" + id_str + "/thruster" + id_str + "/";

            this->state_interface_data.emplace_back(
                prefix + "esc/rpm", to_interface_data_ptr(this->state.rpm[i].value));
        }

        // Publishers
        const auto state_prefix = std::string("state/");
        const auto qos = rclcpp::SystemDefaultsQoS();
        this->pub.battery_current_publisher =
            this->get_node()->create_publisher<std_msgs::msg::Float64>(
                state_prefix + "battery_current", qos);
        this->pub.battery_voltage_publisher =
            this->get_node()->create_publisher<std_msgs::msg::Float64>(
                state_prefix + "battery_voltage", qos);
        this->pub.main_temperature_publisher =
            this->get_node()->create_publisher<std_msgs::msg::Int8>(
                state_prefix + "main_temperature", qos);
        this->pub.water_leaked_publisher = this->get_node()->create_publisher<std_msgs::msg::Bool>(
            state_prefix + "water_leaked", qos);
        this->pub.imu_temperature_publisher =
            this->get_node()->create_publisher<std_msgs::msg::Float64>(
                state_prefix + "imu_temperature", qos);
        this->pub.imu_orientation_publisher =
            this->get_node()->create_publisher<geometry_msgs::msg::Vector3>(
                state_prefix + "imu_orientation", qos);
        this->pub.imu_velocity_publisher =
            this->get_node()->create_publisher<geometry_msgs::msg::Vector3>(
                state_prefix + "imu_velocity", qos);
        for (size_t i = 0; i < 4; ++i) {
            this->pub.rpm_publisher[i] = this->get_node()->create_publisher<std_msgs::msg::Float64>(
                state_prefix + "thruster_rpm_" + std::to_string(i + 1), qos);
        }
    }
    return cif::CallbackReturn::SUCCESS;
}

auto succ::GateController::update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/
    ) -> cif::return_type {
    // 状態を取得
    util::interface_accessor::get_states_from_loaned_interfaces(
        this->state_interfaces_, this->state_interface_data);

    // 状態をトピックに出力
    this->pub.battery_current_publisher->publish(
        std_msgs::msg::Float64().set__data(this->state.battery_current.value));
    this->pub.battery_voltage_publisher->publish(
        std_msgs::msg::Float64().set__data(this->state.battery_voltage.value));
    this->pub.main_temperature_publisher->publish(
        std_msgs::msg::Int8().set__data(this->state.main_temperature.value));
    this->pub.water_leaked_publisher->publish(
        std_msgs::msg::Bool().set__data(this->state.water_leaked.value));
    this->pub.imu_temperature_publisher->publish(
        std_msgs::msg::Float64().set__data(this->state.imu_temperature.value));
    this->pub.imu_orientation_publisher->publish(geometry_msgs::msg::Vector3()
                                                     .set__x(this->state.imu_orientation.x)
                                                     .set__y(this->state.imu_orientation.y)
                                                     .set__z(this->state.imu_orientation.z));
    this->pub.imu_velocity_publisher->publish(geometry_msgs::msg::Vector3()
                                                  .set__x(this->state.imu_velocity.x)
                                                  .set__y(this->state.imu_velocity.y)
                                                  .set__z(this->state.imu_velocity.z));
    for (size_t i = 0; i < 4; ++i) {
        this->pub.rpm_publisher[i]->publish(
            std_msgs::msg::Float64().set__data(this->state.rpm[i].value));
    }

    // コマンドを送信
    util::interface_accessor::set_commands_to_loaned_interfaces(
        this->command_interfaces_, this->command_interface_data);

    return cif::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::GateController, controller_interface::ControllerInterface)
