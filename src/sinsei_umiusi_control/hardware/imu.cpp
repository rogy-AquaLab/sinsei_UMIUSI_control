#include "sinsei_umiusi_control/hardware/imu.hpp"

#include <hardware_interface/sensor_interface.hpp>

namespace suchw = sinsei_umiusi_control::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

auto suchw::Imu::on_init(const hif::HardwareInfo & info) -> hif::CallbackReturn {
    this->hif::SensorInterface::on_init(info);

    // FIXME: ピン番号はパラメーターなどで設定できるようにする
    // FIXME: Pigpioクラスの初期化にはピン番号が必要なため、適当な値を使用
    this->model.emplace(
        std::make_unique<sinsei_umiusi_control::util::Pigpio>(999), *this->get_clock());

    return hif::CallbackReturn::SUCCESS;
}

auto suchw::Imu::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hif::return_type {
    return hif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sinsei_umiusi_control::hardware::Imu, hardware_interface::SensorInterface)