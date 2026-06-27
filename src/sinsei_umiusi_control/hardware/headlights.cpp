#include "sinsei_umiusi_control/hardware/headlights.hpp"

#include <memory>

#include "sinsei_umiusi_control/hardware_model/impl/linux_gpio.hpp"
#include "sinsei_umiusi_control/state/headlights.hpp"
#include "sinsei_umiusi_control/util/params.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"
#include "sinsei_umiusi_control/util/string.hpp"

using namespace sinsei_umiusi_control::hardware;

auto Headlights::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
    -> hardware_interface::CallbackReturn {
    this->hardware_interface::SystemInterface::on_init(params);

    const auto gpiochip_device =
        util::find_param(params.hardware_info.hardware_parameters, "gpiochip_device");
    if (!gpiochip_device) {
        RCLCPP_ERROR(
            this->get_logger(), "Parameter 'gpiochip_device' not found in hardware parameters.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    auto gpio = std::make_unique<hardware_model::impl::LinuxGpio>(gpiochip_device.value());

    auto get_offset_param =
        [&](const std::string & key) -> std::optional<hardware_model::interface::GpioOffset> {
        // GPIO line offsetをパラメータから取得
        const auto str_opt = util::find_param(params.hardware_info.hardware_parameters, key);
        if (!str_opt) {
            RCLCPP_ERROR(
                this->get_logger(), "Parameter '%s' not found in hardware parameters.",
                key.c_str());
            return std::nullopt;
        }

        // std::stringからGpioOffsetに変換
        auto offset_res =
            util::to_number<hardware_model::interface::GpioOffset>(str_opt.value());
        if (!offset_res) {
            RCLCPP_ERROR(
                this->get_logger(), "Invalid GPIO line offset for '%s' ('%s'): %s", key.c_str(),
                str_opt.value().c_str(), offset_res.error().c_str());
            return std::nullopt;
        }
        return *offset_res;
    };

    const auto high_beam_line_offset = get_offset_param("high_beam_line_offset");
    const auto low_beam_line_offset = get_offset_param("low_beam_line_offset");
    const auto ir_line_offset = get_offset_param("ir_line_offset");
    if (!high_beam_line_offset || !low_beam_line_offset || !ir_line_offset) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    this->model.emplace(
        std::move(gpio), *high_beam_line_offset, *low_beam_line_offset, *ir_line_offset);

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
