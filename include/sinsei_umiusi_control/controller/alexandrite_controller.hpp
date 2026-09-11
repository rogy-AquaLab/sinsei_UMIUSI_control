#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_ALEXANDRITE_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_ALEXANDRITE_CONTROLLER_HPP

#include <array>
#include <chrono>
#include <controller_interface/controller_interface.hpp>
#include <cstddef>
#include <mutex>
#include <rclcpp/subscription.hpp>
#include <string>

#include "sinsei_umiusi_control/cmd/actuator/motor.hpp"
#include "sinsei_umiusi_control/cmd/actuator/servo.hpp"
#include "sinsei_umiusi_control/util/interface_accessor.hpp"
#include "sinsei_umiusi_msgs/msg/thruster_output.hpp"

namespace sinsei_umiusi_control::controller {

class AlexandriteController : public controller_interface::ControllerInterface {
  private:
    static constexpr size_t THRUSTER_COUNT = 3;
    static constexpr size_t CRAWLER_COUNT = 2;
    static constexpr size_t ACTUATOR_COUNT = THRUSTER_COUNT + CRAWLER_COUNT;

    struct ActuatorCommand {
        cmd::actuator::motor::Allowed motor_allowed;
        cmd::actuator::motor::DutyCycle motor_duty_cycle;
        cmd::actuator::servo::Allowed servo_allowed;
        cmd::actuator::servo::Angle servo_angle;
    };

    struct ReceivedCommand {
        bool received{false};
        std::chrono::steady_clock::time_point received_at{};
        bool motor_allowed{false};
        double motor_duty_cycle{0.0};
        bool servo_allowed{false};
        double servo_angle_degrees{0.0};
    };

    std::array<ActuatorCommand, ACTUATOR_COUNT> commands{};
    std::array<ReceivedCommand, ACTUATOR_COUNT> received_commands{};
    std::array<
        rclcpp::Subscription<sinsei_umiusi_msgs::msg::ThrusterOutput>::SharedPtr, ACTUATOR_COUNT>
        subscriptions{};
    std::mutex received_commands_mutex;

    util::interface_accessor::InterfaceDataContainer command_interface_data;

    double command_timeout_seconds{0.5};
    double max_duty{0.5};
    std::string command_topic_prefix{"cmd/direct/alexandrite_controller"};

    auto stop_all_actuators() -> void;
    auto clear_received_commands() -> void;

  public:
    AlexandriteController() = default;

    auto command_interface_configuration() const
        -> controller_interface::InterfaceConfiguration override;
    auto state_interface_configuration() const
        -> controller_interface::InterfaceConfiguration override;
    auto on_init() -> CallbackReturn override;
    auto on_configure(const rclcpp_lifecycle::State & previous_state) -> CallbackReturn override;
    auto on_deactivate(const rclcpp_lifecycle::State & previous_state) -> CallbackReturn override;
    auto update(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> controller_interface::return_type override;
};

}  // namespace sinsei_umiusi_control::controller

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_ALEXANDRITE_CONTROLLER_HPP
