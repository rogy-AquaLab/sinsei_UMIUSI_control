#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_IMU_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_IMU_CONTROLLER_HPP

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <string>

#include "sinsei_umiusi_control/state/imu.hpp"
#include "sinsei_umiusi_control/util/interface_accessor.hpp"

namespace sinsei_umiusi_control::controller {

// IMUの状態をtopicへ出す暫定コントローラ
// UMIUSIではGateControllerが出しているが、AlexandriteではGateControllerを使わないため分けている
class ImuController : public controller_interface::ControllerInterface {
  private:
    struct State {
        state::imu::Quaternion quaternion;
        state::imu::Acceleration acceleration;
        state::imu::AngularVelocity angular_velocity;
        state::imu::Temperature temperature;
    };

    State state{};
    util::interface_accessor::InterfaceDataContainer state_interface_data;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_publisher;

    std::string state_topic_prefix{"state/"};

  public:
    ImuController() = default;

    auto command_interface_configuration() const
        -> controller_interface::InterfaceConfiguration override;
    auto state_interface_configuration() const
        -> controller_interface::InterfaceConfiguration override;
    auto on_init() -> CallbackReturn override;
    auto on_configure(const rclcpp_lifecycle::State & previous_state) -> CallbackReturn override;
    auto update(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> controller_interface::return_type override;
};

}  // namespace sinsei_umiusi_control::controller

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_IMU_CONTROLLER_HPP
