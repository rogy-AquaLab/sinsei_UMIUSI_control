#include "sinsei_umiusi_control/controller/imu_controller.hpp"

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>

#include "sinsei_umiusi_control/util/serialization.hpp"

using namespace sinsei_umiusi_control::controller;

auto ImuController::command_interface_configuration() const
    -> controller_interface::InterfaceConfiguration {
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::NONE, {}};
}

auto ImuController::state_interface_configuration() const
    -> controller_interface::InterfaceConfiguration {
    auto state_names = std::vector<std::string>{};
    for (const auto & [name, _data, _size] : this->state_interface_data) {
        state_names.push_back(name);
    }

    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::INDIVIDUAL, state_names};
}

auto ImuController::on_init() -> controller_interface::CallbackReturn {
    using rcl_interfaces::msg::ParameterDescriptor;

    this->get_node()->declare_parameter(
        "state_topic_prefix", this->state_topic_prefix,
        ParameterDescriptor{}
            .set__description("Prefix of IMU state topics")
            .set__type(rclcpp::PARAMETER_STRING)
            .set__read_only(true));

    return controller_interface::CallbackReturn::SUCCESS;
}

auto ImuController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    -> controller_interface::CallbackReturn {
    this->state_topic_prefix = this->get_node()->get_parameter("state_topic_prefix").as_string();

    this->state = State{};

    using util::to_interface_data_ptr;
    auto & data = this->state_interface_data;
    auto & state = this->state;
    data.clear();
    data.emplace_back(
        "imu/quaternion.x", to_interface_data_ptr(state.quaternion.x), sizeof(state.quaternion.x));
    data.emplace_back(
        "imu/quaternion.y", to_interface_data_ptr(state.quaternion.y), sizeof(state.quaternion.y));
    data.emplace_back(
        "imu/quaternion.z", to_interface_data_ptr(state.quaternion.z), sizeof(state.quaternion.z));
    data.emplace_back(
        "imu/quaternion.w", to_interface_data_ptr(state.quaternion.w), sizeof(state.quaternion.w));
    data.emplace_back(
        "imu/acceleration.x", to_interface_data_ptr(state.acceleration.x),
        sizeof(state.acceleration.x));
    data.emplace_back(
        "imu/acceleration.y", to_interface_data_ptr(state.acceleration.y),
        sizeof(state.acceleration.y));
    data.emplace_back(
        "imu/acceleration.z", to_interface_data_ptr(state.acceleration.z),
        sizeof(state.acceleration.z));
    data.emplace_back(
        "imu/angular_velocity.x", to_interface_data_ptr(state.angular_velocity.x),
        sizeof(state.angular_velocity.x));
    data.emplace_back(
        "imu/angular_velocity.y", to_interface_data_ptr(state.angular_velocity.y),
        sizeof(state.angular_velocity.y));
    data.emplace_back(
        "imu/angular_velocity.z", to_interface_data_ptr(state.angular_velocity.z),
        sizeof(state.angular_velocity.z));
    data.emplace_back(
        "imu/temperature", to_interface_data_ptr(state.temperature), sizeof(state.temperature));

    // UMIUSIのGateControllerと同じtopicに出す
    const auto qos = rclcpp::SystemDefaultsQoS();
    this->imu_publisher = this->get_node()->create_publisher<sensor_msgs::msg::Imu>(
        this->state_topic_prefix + "imu", qos);
    this->temperature_publisher = this->get_node()->create_publisher<sensor_msgs::msg::Temperature>(
        this->state_topic_prefix + "imu_temperature", qos);

    return controller_interface::CallbackReturn::SUCCESS;
}

// 変換はGateController::updateと揃えている
auto ImuController::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
    -> controller_interface::return_type {
    if (!util::interface_accessor::get_states_from_loaned_interfaces(
            this->state_interfaces_, this->state_interface_data)) {
        RCLCPP_WARN_THROTTLE(
            this->get_node()->get_logger(), *this->get_node()->get_clock(), 3000,
            "Failed to get IMU state interfaces");
    }
    const auto & state = this->state;
    const auto header = std_msgs::msg::Header().set__stamp(time).set__frame_id("imu");

    this->imu_publisher->publish(sensor_msgs::msg::Imu()
                                     .set__header(header)
                                     .set__orientation(geometry_msgs::msg::Quaternion()
                                                           .set__x(state.quaternion.x)
                                                           .set__y(state.quaternion.y)
                                                           .set__z(state.quaternion.z)
                                                           .set__w(state.quaternion.w))
                                     .set__linear_acceleration(geometry_msgs::msg::Vector3()
                                                                   .set__x(state.acceleration.x)
                                                                   .set__y(state.acceleration.y)
                                                                   .set__z(state.acceleration.z))
                                     .set__angular_velocity(geometry_msgs::msg::Vector3()
                                                                .set__x(state.angular_velocity.x)
                                                                .set__y(state.angular_velocity.y)
                                                                .set__z(state.angular_velocity.z)));
    this->temperature_publisher->publish(
        sensor_msgs::msg::Temperature().set__header(header).set__temperature(
            state.temperature.value));

    return controller_interface::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::ImuController, controller_interface::ControllerInterface)
