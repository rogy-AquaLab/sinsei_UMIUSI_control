#include "sinsei_umiusi_control/controller/alexandrite_controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <controller_interface/controller_interface_base.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/qos.hpp>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "sinsei_umiusi_control/util/serialization.hpp"

using namespace sinsei_umiusi_control::controller;

namespace {

constexpr auto ACTUATOR_NAMES = std::array<std::string_view, 5>{
    "thruster1", "thruster2", "thruster3", "crawler_left", "crawler_right"};
constexpr double RADIANS_TO_DEGREES = 180.0 / 3.14159265358979323846;
constexpr double MAX_SERVO_ANGLE_RADIANS = 3.14159265358979323846 / 2.0;

}  // namespace

auto AlexandriteController::command_interface_configuration() const
    -> controller_interface::InterfaceConfiguration {
    auto names = std::vector<std::string>{};
    names.reserve(this->command_interface_data.size());
    for (const auto & [name, _data, _size] : this->command_interface_data) {
        names.push_back(name);
    }
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

auto AlexandriteController::state_interface_configuration() const
    -> controller_interface::InterfaceConfiguration {
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::NONE, {}};
}

auto AlexandriteController::on_init() -> controller_interface::CallbackReturn {
    using rcl_interfaces::msg::FloatingPointRange;
    using rcl_interfaces::msg::ParameterDescriptor;

    this->get_node()->declare_parameter(
        "command_timeout", 0.5,
        ParameterDescriptor{}
            .set__description("Seconds before a stale actuator command is stopped")
            .set__type(rclcpp::PARAMETER_DOUBLE)
            .set__floating_point_range(
                {FloatingPointRange{}.set__from_value(0.01).set__to_value(60.0)})
            .set__read_only(true));
    this->get_node()->declare_parameter(
        "max_duty", 0.5,
        ParameterDescriptor{}
            .set__description("Absolute duty-cycle limit for every actuator")
            .set__type(rclcpp::PARAMETER_DOUBLE)
            .set__floating_point_range(
                {FloatingPointRange{}.set__from_value(0.0).set__to_value(1.0)})
            .set__read_only(true));
    this->get_node()->declare_parameter(
        "command_topic_prefix", this->command_topic_prefix,
        ParameterDescriptor{}
            .set__description("Prefix for the five direct actuator command topics")
            .set__type(rclcpp::PARAMETER_STRING)
            .set__read_only(true));

    this->stop_all_actuators();
    return controller_interface::CallbackReturn::SUCCESS;
}

auto AlexandriteController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    -> controller_interface::CallbackReturn {
    this->command_timeout_seconds = this->get_node()->get_parameter("command_timeout").as_double();
    this->max_duty = this->get_node()->get_parameter("max_duty").as_double();
    this->command_topic_prefix =
        this->get_node()->get_parameter("command_topic_prefix").as_string();

    this->command_interface_data.clear();
    this->clear_received_commands();
    for (size_t i = 0; i < ACTUATOR_COUNT; ++i) {
        const auto actuator_prefix = std::string(ACTUATOR_NAMES[i]) + "/";
        this->command_interface_data.emplace_back(
            actuator_prefix + "esc/allowed",
            util::to_interface_data_ptr(this->commands[i].motor_allowed),
            sizeof(this->commands[i].motor_allowed));
        this->command_interface_data.emplace_back(
            actuator_prefix + "esc/duty_cycle",
            util::to_interface_data_ptr(this->commands[i].motor_duty_cycle),
            sizeof(this->commands[i].motor_duty_cycle));
        if (i < THRUSTER_COUNT) {
            this->command_interface_data.emplace_back(
                actuator_prefix + "servo/allowed",
                util::to_interface_data_ptr(this->commands[i].servo_allowed),
                sizeof(this->commands[i].servo_allowed));
            this->command_interface_data.emplace_back(
                actuator_prefix + "servo/angle",
                util::to_interface_data_ptr(this->commands[i].servo_angle),
                sizeof(this->commands[i].servo_angle));
        }

        const auto topic = this->command_topic_prefix + "/" + std::string(ACTUATOR_NAMES[i]);
        this->subscriptions[i] =
            this->get_node()->create_subscription<sinsei_umiusi_msgs::msg::ThrusterOutput>(
                topic, rclcpp::SystemDefaultsQoS(),
                [this, i](const sinsei_umiusi_msgs::msg::ThrusterOutput::SharedPtr message) {
                    auto command = ReceivedCommand{};
                    command.received = true;
                    command.received_at = std::chrono::steady_clock::now();
                    command.motor_allowed = message->runnable.esc;
                    command.motor_duty_cycle =
                        std::clamp(message->duty_cycle, -this->max_duty, this->max_duty);
                    if (i < THRUSTER_COUNT) {
                        command.servo_allowed = message->runnable.servo;
                        command.servo_angle_degrees =
                            std::clamp(
                                message->angle, -MAX_SERVO_ANGLE_RADIANS, MAX_SERVO_ANGLE_RADIANS) *
                            RADIANS_TO_DEGREES;
                    }

                    const auto lock = std::lock_guard(this->received_commands_mutex);
                    this->received_commands[i] = command;
                });
    }

    this->stop_all_actuators();
    return controller_interface::CallbackReturn::SUCCESS;
}

auto AlexandriteController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    -> controller_interface::CallbackReturn {
    this->stop_all_actuators();
    this->clear_received_commands();
    util::interface_accessor::set_commands_to_loaned_interfaces(
        this->command_interfaces_, this->command_interface_data);
    return controller_interface::CallbackReturn::SUCCESS;
}

auto AlexandriteController::update(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/) -> controller_interface::return_type {
    auto received_commands = std::array<ReceivedCommand, ACTUATOR_COUNT>{};
    {
        const auto lock = std::lock_guard(this->received_commands_mutex);
        received_commands = this->received_commands;
    }

    const auto now = std::chrono::steady_clock::now();
    for (size_t i = 0; i < ACTUATOR_COUNT; ++i) {
        const auto age_seconds =
            std::chrono::duration<double>(now - received_commands[i].received_at).count();
        const auto fresh =
            received_commands[i].received && age_seconds <= this->command_timeout_seconds;
        this->commands[i].motor_allowed.value = fresh && received_commands[i].motor_allowed;
        this->commands[i].motor_duty_cycle.value =
            this->commands[i].motor_allowed.value ? received_commands[i].motor_duty_cycle : 0.0;
        if (i < THRUSTER_COUNT) {
            this->commands[i].servo_allowed.value = fresh && received_commands[i].servo_allowed;
            this->commands[i].servo_angle.value = received_commands[i].servo_angle_degrees;
        }
    }

    if (!util::interface_accessor::set_commands_to_loaned_interfaces(
            this->command_interfaces_, this->command_interface_data)) {
        RCLCPP_WARN_THROTTLE(
            this->get_node()->get_logger(), *this->get_node()->get_clock(), 3000,
            "Failed to set Alexandrite actuator command interfaces");
    }
    return controller_interface::return_type::OK;
}

auto AlexandriteController::stop_all_actuators() -> void {
    for (auto & command : this->commands) {
        command.motor_allowed.value = false;
        command.motor_duty_cycle.value = 0.0;
        command.servo_allowed.value = false;
        command.servo_angle.value = 0.0;
    }
}

auto AlexandriteController::clear_received_commands() -> void {
    const auto lock = std::lock_guard(this->received_commands_mutex);
    this->received_commands = {};
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::AlexandriteController,
    controller_interface::ControllerInterface)
