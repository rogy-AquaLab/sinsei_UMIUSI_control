#include "sinsei_umiusi_control/hardware/headlights.hpp"

#include <memory>

#include "sinsei_umiusi_control/hardware_model/impl/pigpio.hpp"
#include "sinsei_umiusi_control/state/headlights.hpp"
#include "sinsei_umiusi_control/util/params.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"

using namespace sinsei_umiusi_control::hardware;

auto Headlights::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
    -> hardware_interface::CallbackReturn {
    this->hardware_interface::SystemInterface::on_init(params);

    auto gpio = std::make_unique<hardware_model::impl::Pigpio>();
    auto high_beam_pin = std::make_unique<hardware_model::impl::Pigpio>();
    auto low_beam_pin = std::make_unique<hardware_model::impl::Pigpio>();
    auto ir_pin = std::make_unique<hardware_model::impl::Pigpio>();

    // ピン番号をパラメーターから取得
    const auto high_beam_pin_num_str =
        util::find_param(params.hardware_info.hardware_parameters, "high_beam_pin");
    if (!high_beam_pin_num_str) {
        RCLCPP_ERROR(
            this->get_logger(), "Parameter 'high_beam_pin' not found in hardware parameters.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto low_beam_pin_num_str =
        util::find_param(params.hardware_info.hardware_parameters, "low_beam_pin");
    if (!low_beam_pin_num_str) {
        RCLCPP_ERROR(
            this->get_logger(), "Parameter 'low_beam_pin' not found in hardware parameters.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto ir_pin_num_str =
        util::find_param(params.hardware_info.hardware_parameters, "ir_pin");
    if (!ir_pin_num_str) {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'ir_pin' not found in hardware parameters.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    int high_beam_pin_num, low_beam_pin_num, ir_pin_num;
    try {
        high_beam_pin_num = std::stoi(high_beam_pin_num_str.value());
        low_beam_pin_num = std::stoi(low_beam_pin_num_str.value());
        ir_pin_num = std::stoi(ir_pin_num_str.value());
    } catch (const std::invalid_argument & e) {
        RCLCPP_ERROR(this->get_logger(), "Invalid pin number: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    this->model.emplace(std::move(gpio), high_beam_pin_num, low_beam_pin_num, ir_pin_num);

    const auto res = this->model->on_init();
    if (!res) {
        RCLCPP_ERROR(
            this->get_logger(), "\n  Failed to initialize Headlights: %s", res.error().c_str());
        // GPIOの初期化に失敗した場合、モデルにnullを再代入する
        this->model.reset();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

auto Headlights::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hardware_interface::return_type {
    if (this->model) this->model->on_read();
    return hardware_interface::return_type::OK;
}

auto Headlights::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    -> hardware_interface::return_type {
    auto high_beam_enabled =
        util::from_interface_data<sinsei_umiusi_control::cmd::headlights::HighBeamEnabled>(
            this->get_command("headlights/high_beam_enabled"));
    auto low_beam_enabled =
        util::from_interface_data<sinsei_umiusi_control::cmd::headlights::LowBeamEnabled>(
            this->get_command("headlights/low_beam_enabled"));
    auto ir_enabled = util::from_interface_data<sinsei_umiusi_control::cmd::headlights::IrEnabled>(
        this->get_command("headlights/ir_enabled"));

    if (!this->model) {
        this->set_state(
            "headlights/health", util::to_interface_data(state::headlights::Health{false}));

        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION,
            "\n  Headlights model is not initialized");
        return hardware_interface::return_type::OK;
    }

    const auto res = this->model->on_write(
        std::move(high_beam_enabled), std::move(low_beam_enabled), std::move(ir_enabled));
    if (!res) {
        this->set_state(
            "headlights/health", util::to_interface_data(state::headlights::Health{false}));

        constexpr auto DURATION = 3000;  // ms
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Failed to write Headlights: %s",
            res.error().c_str());

        return hardware_interface::return_type::OK;
    }

    return hardware_interface::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::hardware::Headlights, hardware_interface::SystemInterface)
