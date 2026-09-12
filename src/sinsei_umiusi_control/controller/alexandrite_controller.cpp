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

#include "sinsei_umiusi_control/controller/logic/attitude/feed_forward.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"

using namespace sinsei_umiusi_control::controller;

namespace {

constexpr auto ACTUATOR_NAMES = std::array<std::string_view, 5>{
    "thruster1", "thruster2", "thruster3", "crawler_left", "crawler_right"};
constexpr auto RADIANS_TO_DEGREES = 180.0 / 3.14159265358979323846;

auto azimuth_angle_degrees(const double horizontal, const double vertical) -> double {
    if (horizontal == 0.0 && vertical == 0.0) {
        return 0.0;
    }
    return std::atan(vertical / horizontal) * RADIANS_TO_DEGREES;
}

auto signed_magnitude(const double horizontal, const double vertical) -> double {
    const auto sign = horizontal < 0.0 ? -1.0 : 1.0;
    return sign * std::hypot(horizontal, vertical);
}

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
        "rear_thruster_reversed", false,
        ParameterDescriptor{}
            .set__description("Reverse the rear thruster duty direction")
            .set__type(rclcpp::PARAMETER_BOOL)
            .set__read_only(true));
    this->get_node()->declare_parameter(
        "target_topic", this->target_topic,
        ParameterDescriptor{}
            .set__description("UMIUSI-compatible target topic for the three thrusters")
            .set__type(rclcpp::PARAMETER_STRING)
            .set__read_only(true));
    this->get_node()->declare_parameter(
        "crawler_target_topic", this->crawler_target_topic,
        ParameterDescriptor{}
            .set__description("Separate target topic for differential crawler motion")
            .set__type(rclcpp::PARAMETER_STRING)
            .set__read_only(true));
    this->get_node()->declare_parameter(
        "runnable_topic", this->runnable_topic,
        ParameterDescriptor{}
            .set__description("UMIUSI-compatible actuator runnable topic")
            .set__type(rclcpp::PARAMETER_STRING)
            .set__read_only(true));

    this->stop_all_actuators();
    return controller_interface::CallbackReturn::SUCCESS;
}

auto AlexandriteController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    -> controller_interface::CallbackReturn {
    this->command_timeout_seconds = this->get_node()->get_parameter("command_timeout").as_double();
    this->max_duty = this->get_node()->get_parameter("max_duty").as_double();
    this->rear_thruster_reversed =
        this->get_node()->get_parameter("rear_thruster_reversed").as_bool();
    this->target_topic = this->get_node()->get_parameter("target_topic").as_string();
    this->crawler_target_topic =
        this->get_node()->get_parameter("crawler_target_topic").as_string();
    this->runnable_topic = this->get_node()->get_parameter("runnable_topic").as_string();

    this->command_interface_data.clear();
    this->clear_inputs();
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
    }

    const auto qos = rclcpp::SystemDefaultsQoS();
    this->target_subscription =
        this->get_node()->create_subscription<sinsei_umiusi_msgs::msg::Target>(
            this->target_topic, qos,
            [this](const sinsei_umiusi_msgs::msg::Target::SharedPtr message) {
                const auto lock = std::lock_guard(this->input_mutex);
                this->target = {true, std::chrono::steady_clock::now(), *message};
            });
    this->crawler_target_subscription =
        this->get_node()->create_subscription<sinsei_umiusi_msgs::msg::Target>(
            this->crawler_target_topic, qos,
            [this](const sinsei_umiusi_msgs::msg::Target::SharedPtr message) {
                const auto lock = std::lock_guard(this->input_mutex);
                this->crawler_target = {true, std::chrono::steady_clock::now(), *message};
            });
    this->runnable_subscription =
        this->get_node()->create_subscription<sinsei_umiusi_msgs::msg::ThrusterRunnableAll>(
            this->runnable_topic, qos,
            [this](const sinsei_umiusi_msgs::msg::ThrusterRunnableAll::SharedPtr message) {
                const auto lock = std::lock_guard(this->input_mutex);
                this->runnable = {true, std::chrono::steady_clock::now(), *message};
            });

    this->stop_all_actuators();
    return controller_interface::CallbackReturn::SUCCESS;
}

auto AlexandriteController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    -> controller_interface::CallbackReturn {
    this->stop_all_actuators();
    this->clear_inputs();
    util::interface_accessor::set_commands_to_loaned_interfaces(
        this->command_interfaces_, this->command_interface_data);
    return controller_interface::CallbackReturn::SUCCESS;
}

auto AlexandriteController::update(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/) -> controller_interface::return_type {
    auto target = TimedInput<sinsei_umiusi_msgs::msg::Target>{};
    auto crawler_target = TimedInput<sinsei_umiusi_msgs::msg::Target>{};
    auto runnable = TimedInput<sinsei_umiusi_msgs::msg::ThrusterRunnableAll>{};
    {
        const auto lock = std::lock_guard(this->input_mutex);
        target = this->target;
        crawler_target = this->crawler_target;
        runnable = this->runnable;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto is_fresh = [this, now](const auto & input) {
        return input.received && std::chrono::duration<double>(now - input.received_at).count() <=
                                     this->command_timeout_seconds;
    };
    const auto target_fresh = is_fresh(target);
    const auto crawler_target_fresh = is_fresh(crawler_target);
    const auto runnable_fresh = is_fresh(runnable);

    auto feed_forward_input = AttitudeController::Input{};
    feed_forward_input.cmd.target_orientation = cmd::attitude::Orientation{
        target.message.orientation.x, target.message.orientation.y, target.message.orientation.z};
    feed_forward_input.cmd.target_velocity = cmd::attitude::Velocity{
        target.message.velocity.x, target.message.velocity.y, target.message.velocity.z};
    const auto thruster_output =
        logic::attitude::FeedForward{}.update(0.0, 0.0, feed_forward_input);
    constexpr auto FRONT_THRUSTER_COUNT = size_t{2};
    constexpr auto UMIUSI_FRONT_INDICES = std::array<size_t, FRONT_THRUSTER_COUNT>{0, 3};
    const auto front_motor_runnable =
        std::array<bool, FRONT_THRUSTER_COUNT>{runnable.message.lf.esc, runnable.message.rf.esc};
    const auto front_servo_runnable = std::array<bool, FRONT_THRUSTER_COUNT>{
        runnable.message.lf.servo, runnable.message.rf.servo};

    for (size_t i = 0; i < FRONT_THRUSTER_COUNT; ++i) {
        const auto umiusi_index = UMIUSI_FRONT_INDICES[i];
        this->commands[i].motor_allowed.value =
            target_fresh && runnable_fresh && front_motor_runnable[i];
        this->commands[i].motor_duty_cycle.value =
            this->commands[i].motor_allowed.value
                ? std::clamp(
                      thruster_output.cmd.esc_thrusts[umiusi_index].value, -this->max_duty,
                      this->max_duty)
                : 0.0;
        this->commands[i].servo_allowed.value =
            target_fresh && runnable_fresh && front_servo_runnable[i];
        this->commands[i].servo_angle.value = thruster_output.cmd.servo_angles[umiusi_index].value;
    }

    // The rear thruster is at negative x and rotates in the yz plane. Its zero angle points in
    // +y, so +y thrust creates -yaw and +z thrust creates +pitch.
    constexpr auto REAR_THRUSTER_INDEX = size_t{2};
    const auto rear_horizontal = target.message.velocity.y - target.message.orientation.z;
    const auto rear_vertical = target.message.velocity.z + target.message.orientation.y;
    auto rear_duty = signed_magnitude(rear_horizontal, rear_vertical);
    if (this->rear_thruster_reversed) {
        rear_duty = -rear_duty;
    }
    this->commands[REAR_THRUSTER_INDEX].motor_allowed.value =
        target_fresh && runnable_fresh && runnable.message.lb.esc;
    this->commands[REAR_THRUSTER_INDEX].motor_duty_cycle.value =
        this->commands[REAR_THRUSTER_INDEX].motor_allowed.value
            ? std::clamp(rear_duty, -this->max_duty, this->max_duty)
            : 0.0;
    this->commands[REAR_THRUSTER_INDEX].servo_allowed.value =
        target_fresh && runnable_fresh && runnable.message.lb.servo;
    this->commands[REAR_THRUSTER_INDEX].servo_angle.value =
        azimuth_angle_degrees(rear_horizontal, rear_vertical);

    auto left_crawler_duty =
        crawler_target.message.velocity.x - crawler_target.message.orientation.z;
    auto right_crawler_duty =
        crawler_target.message.velocity.x + crawler_target.message.orientation.z;
    const auto crawler_max_abs =
        std::max(std::abs(left_crawler_duty), std::abs(right_crawler_duty));
    if (crawler_max_abs > this->max_duty && crawler_max_abs > 0.0) {
        const auto scale = this->max_duty / crawler_max_abs;
        left_crawler_duty *= scale;
        right_crawler_duty *= scale;
    }
    const auto crawler_allowed = crawler_target_fresh && runnable_fresh && runnable.message.rb.esc;
    this->commands[THRUSTER_COUNT].motor_allowed.value = crawler_allowed;
    this->commands[THRUSTER_COUNT].motor_duty_cycle.value =
        crawler_allowed ? left_crawler_duty : 0.0;
    this->commands[THRUSTER_COUNT + 1].motor_allowed.value = crawler_allowed;
    this->commands[THRUSTER_COUNT + 1].motor_duty_cycle.value =
        crawler_allowed ? right_crawler_duty : 0.0;

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

auto AlexandriteController::clear_inputs() -> void {
    const auto lock = std::lock_guard(this->input_mutex);
    this->target = {};
    this->crawler_target = {};
    this->runnable = {};
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::AlexandriteController,
    controller_interface::ControllerInterface)
