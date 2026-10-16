#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_BALLAST_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_BALLAST_CONTROLLER_HPP

#include <array>
#include <controller_interface/controller_interface.hpp>
#include <cstddef>
#include <mutex>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <string>

namespace sinsei_umiusi_control::controller {

// SteppingMotorDriver基板で動かす2つのバラストをtopicから操作する暫定コントローラ
class BallastController : public controller_interface::ControllerInterface {
  private:
    static constexpr std::size_t BALLAST_COUNT = 2;

    struct BallastIo {
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_subscription;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr calibrate_subscription;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_publisher;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_publisher;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr fault_reason_publisher;
    };

    // topicから受け取った値。input_mutexで保護する
    struct BallastInput {
        // 行程に対する割合（0.0〜1.0）。NaNは目標なし
        double target;
        // ゼロ点の取り直しを要求された回数
        double calibrate_request_count;
    };

    std::array<BallastIo, BALLAST_COUNT> ios{};
    std::array<BallastInput, BALLAST_COUNT> inputs{};
    std::mutex input_mutex;

    std::string command_topic_prefix{"cmd/"};
    std::string state_topic_prefix{"state/"};

    auto reset_inputs() -> void;

  public:
    BallastController() = default;

    auto command_interface_configuration() const
        -> controller_interface::InterfaceConfiguration override;
    auto state_interface_configuration() const
        -> controller_interface::InterfaceConfiguration override;
    auto on_init() -> CallbackReturn override;
    auto on_configure(const rclcpp_lifecycle::State & previous_state) -> CallbackReturn override;
    auto on_activate(const rclcpp_lifecycle::State & previous_state) -> CallbackReturn override;
    auto update(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> controller_interface::return_type override;
};

}  // namespace sinsei_umiusi_control::controller

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_BALLAST_CONTROLLER_HPP
