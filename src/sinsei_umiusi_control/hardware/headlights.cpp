#include "sinsei_umiusi_control/hardware/headlights.hpp"

#include <memory>

#include "sinsei_umiusi_control/hardware_model/impl/pigpio.hpp"
#include "sinsei_umiusi_control/util/params.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"

namespace suchw = sinsei_umiusi_control::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

auto suchw::Headlights::on_init(const hif::HardwareInfo & info) -> hif::CallbackReturn {
    this->hif::SystemInterface::on_init(info);

    auto gpio = std::make_unique<sinsei_umiusi_control::util::Pigpio>();
    auto high_beam_pin = std::make_unique<sinsei_umiusi_control::util::Pigpio>();
    auto low_beam_pin = std::make_unique<sinsei_umiusi_control::util::Pigpio>();
    auto ir_pin = std::make_unique<sinsei_umiusi_control::util::Pigpio>();

    // ピン番号をパラメーターから取得
    const auto high_beam_pin_num_str = util::find_param(info.hardware_parameters, "high_beam_pin");
    if (!high_beam_pin_num_str) {
        RCLCPP_ERROR(
            this->get_logger(), "Parameter 'high_beam_pin' not found in hardware parameters.");
        return hif::CallbackReturn::ERROR;
    }
    const auto low_beam_pin_num_str = util::find_param(info.hardware_parameters, "low_beam_pin");
    if (!low_beam_pin_num_str) {
        RCLCPP_ERROR(
            this->get_logger(), "Parameter 'low_beam_pin' not found in hardware parameters.");
        return hif::CallbackReturn::ERROR;
    }
    const auto ir_pin_num_str = util::find_param(info.hardware_parameters, "ir_pin");
    if (!ir_pin_num_str) {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'ir_pin' not found in hardware parameters.");
        return hif::CallbackReturn::ERROR;
    }

    int high_beam_pin_num, low_beam_pin_num, ir_pin_num;
    try {
        high_beam_pin_num = std::stoi(high_beam_pin_num_str.value());
        low_beam_pin_num = std::stoi(low_beam_pin_num_str.value());
        ir_pin_num = std::stoi(ir_pin_num_str.value());
    } catch (const std::invalid_argument & e) {
        RCLCPP_ERROR(this->get_logger(), "Invalid pin number: %s", e.what());
        return hif::CallbackReturn::ERROR;
    }

    this->model.emplace(std::move(gpio), high_beam_pin_num, low_beam_pin_num, ir_pin_num);

    auto res = this->model->on_init();
    if (!res) {
        RCLCPP_ERROR(
            this->get_logger(), "\n  Failed to initialize Headlights: %s", res.error().c_str());
        // GPIOの初期化に失敗した場合、モデルにnullを再代入する
        this->model.reset();
    }

    return hif::CallbackReturn::SUCCESS;
}

auto suchw::Headlights::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hif::return_type {
    if (this->model) this->model->on_read();
    return hif::return_type::OK;
}

auto suchw::Headlights::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    -> hif::return_type {
    double high_beam_enabled_raw = get_command("headlights/high_beam_enabled");
    double low_beam_enabled_raw = get_command("headlights/low_beam_enabled");
    double ir_enabled_raw = get_command("headlights/ir_enabled");
    auto high_beam_enabled =
        util::from_interface_data<sinsei_umiusi_control::cmd::headlights::HighBeamEnabled>(
            high_beam_enabled_raw);
    auto low_beam_enabled =
        util::from_interface_data<sinsei_umiusi_control::cmd::headlights::LowBeamEnabled>(
            low_beam_enabled_raw);
    auto ir_enabled = util::from_interface_data<sinsei_umiusi_control::cmd::headlights::IrEnabled>(
        ir_enabled_raw);

    if (!this->model) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION,
            "\n  Headlights model is not initialized");
        return hif::return_type::OK;
    }

    auto res = this->model->on_write(high_beam_enabled, low_beam_enabled, ir_enabled);
    if (!res) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Failed to write Headlights: %s",
            res.error().c_str());
    }

    return hif::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::hardware::Headlights, hardware_interface::SystemInterface)