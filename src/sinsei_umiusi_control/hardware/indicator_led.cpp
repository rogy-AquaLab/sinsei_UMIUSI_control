#include "sinsei_umiusi_control/hardware/indicator_led.hpp"

#include "sinsei_umiusi_control/util/pigpio.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"

namespace suchw = sinsei_umiusi_control::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

auto suchw::IndicatorLed::on_init(const hif::HardwareInfo & info) -> hif::CallbackReturn {
    this->hif::SystemInterface::on_init(info);

    auto led_pin = std::make_unique<sinsei_umiusi_control::util::Pigpio>();

    // FIXME: ピン番号はパラメーターなどで設定できるようにする
    this->model.emplace(std::move(led_pin), 24);

    auto res = this->model->on_init();
    if (!res) {
        RCLCPP_ERROR(
            this->get_logger(), "\n  Failed to initialize Indicator LED: %s", res.error().c_str());
        // GPIOの初期化に失敗した場合、モデルにnullを再代入する
        this->model.reset();
    }

    return hif::CallbackReturn::SUCCESS;
}

auto suchw::IndicatorLed::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hif::return_type {
    if (this->model) this->model->on_read();
    return hif::return_type::OK;
}

auto suchw::IndicatorLed::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    -> hif::return_type {
    double enabled_raw = get_command("indicator_led/enabled");
    auto enabled =
        util::from_interface_data<sinsei_umiusi_control::cmd::indicator_led::Enabled>(enabled_raw);
    if (!this->model) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION,
            "\n  Indicator LED model is not initialized");
        return hif::return_type::OK;
    }

    auto res = this->model->on_write(enabled);
    if (!res) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION,
            "\n  Failed to write Indicator LED: %s", res.error().c_str());
    }

    return hif::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::hardware::IndicatorLed, hardware_interface::SystemInterface)