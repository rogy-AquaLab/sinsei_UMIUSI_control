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

#include "sinsei_umiusi_control/controller/logic/alexandrite/mixer.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"

using namespace sinsei_umiusi_control::controller;

namespace {

constexpr auto ACTUATOR_NAMES = std::array<std::string_view, 5>{
    "thruster1", "thruster2", "thruster3", "crawler_left", "crawler_right"};
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
    auto names = std::vector<std::string>{};
    names.reserve(this->state_interface_data.size());
    for (const auto & [name, _data, _size] : this->state_interface_data) {
        names.push_back(name);
    }
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::INDIVIDUAL, names};
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
            .set__description("Absolute duty-cycle limit for the thrusters")
            .set__type(rclcpp::PARAMETER_DOUBLE)
            .set__floating_point_range(
                {FloatingPointRange{}.set__from_value(0.0).set__to_value(1.0)})
            .set__read_only(true));
    this->get_node()->declare_parameter(
        "crawler_max_duty", 0.5,
        ParameterDescriptor{}
            .set__description("Crawler duty cycle for a full (1.0) crawler input")
            .set__type(rclcpp::PARAMETER_DOUBLE)
            .set__floating_point_range(
                {FloatingPointRange{}.set__from_value(0.0).set__to_value(1.0)})
            .set__read_only(true));
    const auto declare_length = [this](const char * name, const char * description) {
        this->get_node()->declare_parameter(
            name, 1.0,
            ParameterDescriptor{}
                .set__description(description)
                .set__type(rclcpp::PARAMETER_DOUBLE)
                .set__floating_point_range(
                    {FloatingPointRange{}.set__from_value(1e-6).set__to_value(1e6)})
                .set__read_only(true));
    };
    // 長さの比だけを使うため、単位は何でもよい
    declare_length(
        "front_thruster_x", "Longitudinal distance from the center to the front thrusters");
    declare_length("front_thruster_y", "Lateral distance from the center to the front thrusters");
    declare_length("rear_thruster_x", "Longitudinal distance from the center to the rear thruster");
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
    this->get_node()->declare_parameter(
        "state_topic_prefix", this->state_topic_prefix,
        ParameterDescriptor{}
            .set__description("Prefix of thruster and crawler state topics")
            .set__type(rclcpp::PARAMETER_STRING)
            .set__read_only(true));

    this->stop_all_actuators();
    return controller_interface::CallbackReturn::SUCCESS;
}

auto AlexandriteController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    -> controller_interface::CallbackReturn {
    this->command_timeout_seconds = this->get_node()->get_parameter("command_timeout").as_double();
    this->max_duty = this->get_node()->get_parameter("max_duty").as_double();
    this->crawler_max_duty = this->get_node()->get_parameter("crawler_max_duty").as_double();
    this->thruster_geometry = logic::alexandrite::Geometry{
        this->get_node()->get_parameter("front_thruster_x").as_double(),
        this->get_node()->get_parameter("front_thruster_y").as_double(),
        this->get_node()->get_parameter("rear_thruster_x").as_double(),
    };
    this->target_topic = this->get_node()->get_parameter("target_topic").as_string();
    this->crawler_target_topic =
        this->get_node()->get_parameter("crawler_target_topic").as_string();
    this->runnable_topic = this->get_node()->get_parameter("runnable_topic").as_string();
    this->state_topic_prefix = this->get_node()->get_parameter("state_topic_prefix").as_string();

    this->command_interface_data.clear();
    this->state_interface_data.clear();
    this->states = {};
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

        this->state_interface_data.emplace_back(
            actuator_prefix + "esc/rpm", util::to_interface_data_ptr(this->states[i].rpm),
            sizeof(this->states[i].rpm));
        this->state_interface_data.emplace_back(
            actuator_prefix + "esc/voltage", util::to_interface_data_ptr(this->states[i].voltage),
            sizeof(this->states[i].voltage));
        this->state_interface_data.emplace_back(
            actuator_prefix + "esc/water_leaked",
            util::to_interface_data_ptr(this->states[i].water_leaked),
            sizeof(this->states[i].water_leaked));
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

    // スラスタはUMIUSIと同じtopicに、指令と同じ対応（thruster1〜3 = lf, lb, rf）で出す
    this->thruster_state_all_publisher =
        this->get_node()->create_publisher<sinsei_umiusi_msgs::msg::ThrusterStateAll>(
            this->state_topic_prefix + "thruster_state_all", qos);
    // クローラーはThrusterStateAllに入らないため、別のtopicに出す
    for (size_t i = 0; i < CRAWLER_COUNT; ++i) {
        this->crawler_state_publishers[i] =
            this->get_node()->create_publisher<sinsei_umiusi_msgs::msg::ThrusterState>(
                this->state_topic_prefix + std::string(ACTUATOR_NAMES[THRUSTER_COUNT + i]), qos);
    }

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

    const auto thruster_outputs = logic::alexandrite::mix(
        logic::alexandrite::Wrench{
            target.message.velocity.x,
            target.message.velocity.y,
            target.message.velocity.z,
            target.message.orientation.x,
            target.message.orientation.y,
            target.message.orientation.z,
        },
        this->thruster_geometry, this->max_duty);
    // UMIUSIと同じく左前から反時計回りに数えるため、thruster1〜3 = 左前、後ろ、右前 = lf、lb、rf
    const auto thruster_runnable =
        std::array<sinsei_umiusi_msgs::msg::ThrusterRunnable, THRUSTER_COUNT>{
            runnable.message.lf, runnable.message.lb, runnable.message.rf};
    for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
        auto & command = this->commands[i];
        command.motor_allowed.value = target_fresh && runnable_fresh && thruster_runnable[i].esc;
        command.motor_duty_cycle.value =
            command.motor_allowed.value ? thruster_outputs[i].duty : 0.0;
        command.servo_allowed.value = target_fresh && runnable_fresh && thruster_runnable[i].servo;
        command.servo_angle.value = thruster_outputs[i].angle;
    }

    auto left_crawler_duty =
        crawler_target.message.velocity.x - crawler_target.message.orientation.z;
    auto right_crawler_duty =
        crawler_target.message.velocity.x + crawler_target.message.orientation.z;
    // 入力は-1〜1。差動計算で1を超えた分は、左右の比を保って1に収めてから上限を掛ける
    const auto crawler_max_abs =
        std::max(std::abs(left_crawler_duty), std::abs(right_crawler_duty));
    const auto crawler_scale = this->crawler_max_duty / std::max(crawler_max_abs, 1.0);
    left_crawler_duty *= crawler_scale;
    right_crawler_duty *= crawler_scale;
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

    this->publish_states();
    return controller_interface::return_type::OK;
}

auto AlexandriteController::make_thruster_state(const size_t index) const
    -> sinsei_umiusi_msgs::msg::ThrusterState {
    using sinsei_umiusi_msgs::msg::ThrusterMode;

    const auto & command = this->commands[index];
    const auto & state = this->states[index];
    const auto has_servo = index < THRUSTER_COUNT;
    const auto to_mode = [](const bool allowed) {
        return allowed ? ThrusterMode::RUNNABLE : ThrusterMode::STANDBY;
    };
    return sinsei_umiusi_msgs::msg::ThrusterState()
        .set__mode(
            ThrusterMode()
                .set__esc(to_mode(command.motor_allowed.value))
                .set__servo(
                    has_servo ? to_mode(command.servo_allowed.value) : ThrusterMode::DISABLED))
        .set__duty_cycle(command.motor_duty_cycle.value)
        .set__angle(has_servo ? command.servo_angle.value : 0.0)
        .set__rpm(state.rpm.value)
        .set__input_voltage(static_cast<float>(state.voltage.value))
        .set__water_leaked(state.water_leaked.value);
}

auto AlexandriteController::publish_states() -> void {
    if (!util::interface_accessor::get_states_from_loaned_interfaces(
            this->state_interfaces_, this->state_interface_data)) {
        RCLCPP_WARN_THROTTLE(
            this->get_node()->get_logger(), *this->get_node()->get_clock(), 3000,
            "Failed to get Alexandrite actuator state interfaces");
    }

    // Alexandriteには右後ろのスラスタが無いため、rbは常にDISABLEDにする
    const auto absent = sinsei_umiusi_msgs::msg::ThrusterState().set__mode(
        sinsei_umiusi_msgs::msg::ThrusterMode()
            .set__esc(sinsei_umiusi_msgs::msg::ThrusterMode::DISABLED)
            .set__servo(sinsei_umiusi_msgs::msg::ThrusterMode::DISABLED));
    this->thruster_state_all_publisher->publish(sinsei_umiusi_msgs::msg::ThrusterStateAll()
                                                    .set__lf(this->make_thruster_state(0))
                                                    .set__lb(this->make_thruster_state(1))
                                                    .set__rf(this->make_thruster_state(2))
                                                    .set__rb(absent));
    for (size_t i = 0; i < CRAWLER_COUNT; ++i) {
        this->crawler_state_publishers[i]->publish(this->make_thruster_state(THRUSTER_COUNT + i));
    }
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
