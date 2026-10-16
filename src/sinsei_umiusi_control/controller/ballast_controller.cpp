#include "sinsei_umiusi_control/controller/ballast_controller.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/qos.hpp>
#include <string>
#include <string_view>
#include <vector>

using namespace sinsei_umiusi_control::controller;

namespace {

constexpr auto BALLAST_NAMES = std::array<std::string_view, 2>{"ballast1", "ballast2"};

// command_interfaces_はこの順序で並ぶ
constexpr auto COMMAND_INTERFACE_NAMES = std::array<std::string_view, 2>{"target", "calibrate"};
constexpr std::size_t TARGET_COMMAND_INDEX = 0;
constexpr std::size_t CALIBRATE_COMMAND_INDEX = 1;

// state_interfaces_はこの順序で並ぶ
constexpr auto STATE_INTERFACE_NAMES =
    std::array<std::string_view, 4>{"position", "state", "fault_reason", "health"};
constexpr std::size_t POSITION_STATE_INDEX = 0;
constexpr std::size_t STATE_STATE_INDEX = 1;
constexpr std::size_t FAULT_REASON_STATE_INDEX = 2;
constexpr std::size_t HEALTH_STATE_INDEX = 3;

template <std::size_t N>
auto make_interface_names(const std::array<std::string_view, N> & interface_names)
    -> std::vector<std::string> {
    auto names = std::vector<std::string>{};
    names.reserve(BALLAST_NAMES.size() * interface_names.size());
    for (const auto & ballast_name : BALLAST_NAMES) {
        for (const auto & interface_name : interface_names) {
            names.push_back(std::string(ballast_name) + "/" + std::string(interface_name));
        }
    }
    return names;
}

auto to_uint8(const double value) -> uint8_t {
    if (!std::isfinite(value) || value < 0.0 || value > 255.0) {
        return 0;
    }
    return static_cast<uint8_t>(value);
}

}  // namespace

auto BallastController::command_interface_configuration() const
    -> controller_interface::InterfaceConfiguration {
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::INDIVIDUAL,
        make_interface_names(COMMAND_INTERFACE_NAMES)};
}

auto BallastController::state_interface_configuration() const
    -> controller_interface::InterfaceConfiguration {
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::INDIVIDUAL,
        make_interface_names(STATE_INTERFACE_NAMES)};
}

auto BallastController::on_init() -> controller_interface::CallbackReturn {
    using rcl_interfaces::msg::ParameterDescriptor;

    this->get_node()->declare_parameter(
        "command_topic_prefix", this->command_topic_prefix,
        ParameterDescriptor{}
            .set__description("Prefix of ballast command topics")
            .set__type(rclcpp::PARAMETER_STRING)
            .set__read_only(true));
    this->get_node()->declare_parameter(
        "state_topic_prefix", this->state_topic_prefix,
        ParameterDescriptor{}
            .set__description("Prefix of ballast state topics")
            .set__type(rclcpp::PARAMETER_STRING)
            .set__read_only(true));

    this->reset_inputs();
    return controller_interface::CallbackReturn::SUCCESS;
}

auto BallastController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    -> controller_interface::CallbackReturn {
    this->command_topic_prefix =
        this->get_node()->get_parameter("command_topic_prefix").as_string();
    this->state_topic_prefix = this->get_node()->get_parameter("state_topic_prefix").as_string();

    const auto qos = rclcpp::SystemDefaultsQoS();
    for (std::size_t i = 0; i < BALLAST_COUNT; ++i) {
        const auto ballast_name = std::string(BALLAST_NAMES[i]);
        const auto command_prefix = this->command_topic_prefix + ballast_name + "/";
        const auto state_prefix = this->state_topic_prefix + ballast_name + "/";
        auto & io = this->ios[i];

        io.target_subscription = this->get_node()->create_subscription<std_msgs::msg::Float64>(
            command_prefix + "target", qos,
            [this, i](const std_msgs::msg::Float64::SharedPtr message) {
                if (!std::isfinite(message->data) || message->data < 0.0 || message->data > 1.0) {
                    RCLCPP_WARN(
                        this->get_node()->get_logger(), "Ignored ballast target out of [0, 1]: %f",
                        message->data);
                    return;
                }
                const auto lock = std::lock_guard(this->input_mutex);
                this->inputs[i].target = message->data;
            });
        io.calibrate_subscription = this->get_node()->create_subscription<std_msgs::msg::Empty>(
            command_prefix + "calibrate", qos,
            [this, i](const std_msgs::msg::Empty::SharedPtr /*message*/) {
                const auto lock = std::lock_guard(this->input_mutex);
                this->inputs[i].calibrate_request_count += 1.0;
            });
        io.position_publisher = this->get_node()->create_publisher<std_msgs::msg::Float64>(
            state_prefix + "position", qos);
        io.state_publisher =
            this->get_node()->create_publisher<std_msgs::msg::UInt8>(state_prefix + "state", qos);
        io.fault_reason_publisher = this->get_node()->create_publisher<std_msgs::msg::UInt8>(
            state_prefix + "fault_reason", qos);
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

auto BallastController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    -> controller_interface::CallbackReturn {
    // 有効化前に受け取った目標で、意図せず動き出さないようにする
    this->reset_inputs();
    return controller_interface::CallbackReturn::SUCCESS;
}

auto BallastController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    -> controller_interface::return_type {
    auto inputs = decltype(this->inputs){};
    {
        const auto lock = std::lock_guard(this->input_mutex);
        inputs = this->inputs;
    }

    auto command_ok = true;
    for (std::size_t i = 0; i < BALLAST_COUNT; ++i) {
        const auto offset = i * COMMAND_INTERFACE_NAMES.size();
        command_ok &=
            this->command_interfaces_[offset + TARGET_COMMAND_INDEX].set_value(inputs[i].target);
        command_ok &= this->command_interfaces_[offset + CALIBRATE_COMMAND_INDEX].set_value(
            inputs[i].calibrate_request_count);
    }
    if (!command_ok) {
        RCLCPP_WARN_THROTTLE(
            this->get_node()->get_logger(), *this->get_node()->get_clock(), 3000,
            "Failed to set ballast command interfaces");
    }

    for (std::size_t i = 0; i < BALLAST_COUNT; ++i) {
        const auto offset = i * STATE_INTERFACE_NAMES.size();
        const auto get_state = [this, offset](const std::size_t index) {
            return this->state_interfaces_[offset + index].get_optional().value_or(
                std::numeric_limits<double>::quiet_NaN());
        };
        const auto & io = this->ios[i];

        // 基板から状態が届いていないときは、位置を不明として扱う
        const auto healthy = get_state(HEALTH_STATE_INDEX) > 0.5;
        io.position_publisher->publish(std_msgs::msg::Float64{}.set__data(
            healthy ? get_state(POSITION_STATE_INDEX) : std::numeric_limits<double>::quiet_NaN()));
        io.state_publisher->publish(
            std_msgs::msg::UInt8{}.set__data(to_uint8(get_state(STATE_STATE_INDEX))));
        io.fault_reason_publisher->publish(
            std_msgs::msg::UInt8{}.set__data(to_uint8(get_state(FAULT_REASON_STATE_INDEX))));
    }

    return controller_interface::return_type::OK;
}

auto BallastController::reset_inputs() -> void {
    const auto lock = std::lock_guard(this->input_mutex);
    for (auto & input : this->inputs) {
        input.target = std::numeric_limits<double>::quiet_NaN();
        // 要求回数はハードウェア側で増加を検出するため、0には戻さない
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::BallastController, controller_interface::ControllerInterface)
