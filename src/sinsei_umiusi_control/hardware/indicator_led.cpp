#include "sinsei_umiusi_control/hardware/indicator_led.hpp"

#include "sinsei_umiusi_control/util/pigpio.hpp"

namespace suchw = sinsei_umiusi_control::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

auto suchw::IndicatorLed::on_init(const hif::HardwareInfo & info) -> hif::CallbackReturn {
    this->hif::SystemInterface::on_init(info);

    auto led_pin = std::make_unique<sinsei_umiusi_control::util::Pigpio>();

    // FIXME: ピン番号はパラメーターなどで設定できるようにする
    this->model.emplace(std::move(led_pin), 24);

    // TODO: エラー処理を追加
    this->model->on_init();

    return hif::CallbackReturn::SUCCESS;
}

auto suchw::IndicatorLed::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hif::return_type {
    if (this->model.has_value()) this->model->on_read();
    return hif::return_type::OK;
}

auto suchw::IndicatorLed::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    -> hif::return_type {
    double enabled_raw = get_command("indicator_led/enabled");
    auto enabled =
        *reinterpret_cast<sinsei_umiusi_control::cmd::indicator_led::Enabled *>(&enabled_raw);
    if (this->model.has_value()) this->model->on_write(enabled);

    return hif::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::hardware::IndicatorLed, hardware_interface::SystemInterface)