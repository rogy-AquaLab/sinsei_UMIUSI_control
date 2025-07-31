#include "sinsei_umiusi_control/hardware/imu.hpp"

#include "sinsei_umiusi_control/hardware_model/impl/pigpio.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"

namespace suchw = sinsei_umiusi_control::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

auto suchw::Imu::on_init(const hif::HardwareInfo & info) -> hif::CallbackReturn {
    this->hif::SensorInterface::on_init(info);

    this->model.emplace(std::make_unique<sinsei_umiusi_control::hardware_model::impl::Pigpio>());

    auto res = this->model->on_init();
    if (!res) {
        RCLCPP_ERROR(this->get_logger(), "\n  Failed to initialize IMU: %s", res.error().c_str());
        // IMUの初期化に失敗した場合、モデルにnullを再代入する
        this->model.reset();
    }

    return hif::CallbackReturn::SUCCESS;
}

auto suchw::Imu::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hif::return_type {
    if (!this->model) {
        constexpr auto DURATION = 3000;
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  IMU model is not initialized");
        return hif::return_type::OK;
    }

    const auto res = this->model->on_read();
    if (!res) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Failed to read IMU data: %s",
            res.error().c_str());
    }

    const auto [quaternion, velocity, temperature] = res.value();

    this->set_state("imu/quaternion.x", quaternion.x);
    this->set_state("imu/quaternion.y", quaternion.y);
    this->set_state("imu/quaternion.z", quaternion.z);
    this->set_state("imu/quaternion.w", quaternion.w);
    // this->set_state("imu/velocity.x", velocity.x);
    // this->set_state("imu/velocity.y", velocity.y);
    // this->set_state("imu/velocity.z", velocity.z);
    this->set_state("imu/temperature", util::to_interface_data(temperature));

    return hif::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sinsei_umiusi_control::hardware::Imu, hardware_interface::SensorInterface)