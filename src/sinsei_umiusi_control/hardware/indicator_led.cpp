#include "sinsei_umiusi_control/hardware/indicator_led.hpp"

#include <string>

#include "sinsei_umiusi_control/hardware_model/impl/linux_gpio.hpp"
#include "sinsei_umiusi_control/state/indicator_led.hpp"
#include "sinsei_umiusi_control/util/params.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"
#include "sinsei_umiusi_control/util/string.hpp"

using namespace sinsei_umiusi_control::hardware;

auto IndicatorLed::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
    -> hardware_interface::CallbackReturn {
    this->hardware_interface::SystemInterface::on_init(params);

    const auto gpiochip_device =
        util::find_param(params.hardware_info.hardware_parameters, "gpiochip_device");
    if (!gpiochip_device) {
        RCLCPP_ERROR(
            this->get_logger(), "Parameter 'gpiochip_device' not found in hardware parameters.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    auto gpio = std::make_unique<sinsei_umiusi_control::hardware_model::impl::LinuxGpioChip>(
        gpiochip_device.value());

    // GPIO line offsetをパラメータから取得
    const auto led_line_offset_str_opt =
        util::find_param(params.hardware_info.hardware_parameters, "line_offset");
    if (!led_line_offset_str_opt) {
        RCLCPP_ERROR(
            this->get_logger(), "Parameter 'line_offset' not found in hardware parameters.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // std::stringからGpioOffsetに変換
    auto led_line_offset_res = util::from_chars_expected<hardware_model::interface::GpioOffset>(
        led_line_offset_str_opt.value());
    if (!led_line_offset_res) {
        RCLCPP_ERROR(
            this->get_logger(), "Invalid GPIO line offset for 'line_offset' ('%s'): %s",
            led_line_offset_str_opt.value().c_str(), led_line_offset_res.error().c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    this->model.emplace(std::move(gpio), *led_line_offset_res);

    const auto res = this->model->on_init();
    if (!res) {
        RCLCPP_ERROR(
            this->get_logger(), "\n  Failed to initialize Indicator LED: %s", res.error().c_str());
        // GPIOの初期化に失敗した場合、モデルにnullを再代入する
        this->model.reset();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

auto IndicatorLed::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hardware_interface::return_type {
    if (this->model) this->model->on_read();
    return hardware_interface::return_type::OK;
}

auto IndicatorLed::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    -> hardware_interface::return_type {
    auto enabled = util::from_interface_data<sinsei_umiusi_control::cmd::indicator_led::Enabled>(
        this->get_command("indicator_led/enabled"));
    if (!this->model) {
        this->set_state(
            "indicator_led/health", util::to_interface_data(state::indicator_led::Health{false}));

        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION,
            "\n  Indicator LED model is not initialized");
        return hardware_interface::return_type::OK;
    }

    const auto res = this->model->on_write(std::move(enabled));
    if (!res) {
        this->set_state(
            "indicator_led/health", util::to_interface_data(state::indicator_led::Health{false}));

        constexpr auto DURATION = 3000;  // ms
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION,
            "\n  Failed to write Indicator LED: %s", res.error().c_str());

        return hardware_interface::return_type::OK;
    }

    return hardware_interface::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::hardware::IndicatorLed, hardware_interface::SystemInterface)
