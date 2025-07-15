#include "sinsei_umiusi_control/hardware/imu.hpp"

#include "sinsei_umiusi_control/util/new_type.hpp"
#include "sinsei_umiusi_control/util/pigpio.hpp"

namespace suchw = sinsei_umiusi_control::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

auto suchw::Imu::on_init(const hif::HardwareInfo & info) -> hif::CallbackReturn {
    this->hif::SensorInterface::on_init(info);

    this->model.emplace(std::make_unique<sinsei_umiusi_control::util::Pigpio>());

    auto res = this->model->begin();
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

    auto res = this->model->on_read();
    if (!res) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Failed to read IMU data: %s",
            res.error().c_str());
    }

    auto [orientation, velocity, temperature] = res.value();

    this->set_state("imu/orientation_raw.x", orientation.x);
    this->set_state("imu/orientation_raw.y", orientation.y);
    this->set_state("imu/orientation_raw.z", orientation.z);
    // this->set_state("imu/velocity_raw.x", velocity.x);
    // this->set_state("imu/velocity_raw.y", velocity.y);
    // this->set_state("imu/velocity_raw.z", velocity.z);
    this->set_state("imu/temperature", util::to_double(temperature));

    return hif::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sinsei_umiusi_control::hardware::Imu, hardware_interface::SensorInterface)