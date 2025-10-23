#include "sinsei_umiusi_control/hardware/indicator_led.hpp"

#include "sinsei_umiusi_control/hardware_model/impl/pigpio.hpp"
#include "sinsei_umiusi_control/state/indicator_led.hpp"
#include "sinsei_umiusi_control/util/params.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"

using namespace sinsei_umiusi_control::hardware;

auto IndicatorLed::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
    -> hardware_interface::CallbackReturn {
    this->hardware_interface::SystemInterface::on_init(params);

    auto gpio = std::make_unique<sinsei_umiusi_control::hardware_model::impl::Pigpio>();

    // ピン番号をパラメーターから取得
    const auto led_pin_num_str = util::find_param(params.hardware_info.hardware_parameters, "pin");
    if (!led_pin_num_str) {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'pin' not found in hardware parameters.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    int led_pin_num;
    try {
        led_pin_num = std::stoi(led_pin_num_str.value());
    } catch (const std::invalid_argument & e) {
        RCLCPP_ERROR(this->get_logger(), "Invalid pin number: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    this->model.emplace(std::move(gpio), led_pin_num);

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
