#include "sinsei_umiusi_control/hardware/thruster_direct/esc_direct.hpp"

#include "sinsei_umiusi_control/hardware_model/impl/pigpio.hpp"
#include "sinsei_umiusi_control/state/thruster/esc.hpp"
#include "sinsei_umiusi_control/util/params.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"

using namespace sinsei_umiusi_control::hardware;

auto thruster_direct::EscDirect::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
    -> hardware_interface::CallbackReturn {
    this->hardware_interface::SystemInterface::on_init(params);

    auto esc_pin = std::make_unique<sinsei_umiusi_control::hardware_model::impl::Pigpio>();

    // ID、ピン番号をパラメーターから取得
    const auto id_str = util::find_param(params.hardware_info.hardware_parameters, "id");
    if (!id_str) {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'id' not found in hardware parameters.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    try {
        this->id = static_cast<ID>(std::stoi(id_str.value()));
    } catch (const std::invalid_argument & e) {
        RCLCPP_ERROR(this->get_logger(), "Invalid ID: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (this->id < 1 || this->id > 4) {
        RCLCPP_ERROR(this->get_logger(), "ID must be between 1 and 4.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto esc_pin_num_str = util::find_param(params.hardware_info.hardware_parameters, "pin");
    if (!esc_pin_num_str) {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'pin' not found in hardware parameters.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    int esc_pin_num;
    try {
        esc_pin_num = std::stoi(esc_pin_num_str.value());
    } catch (const std::invalid_argument & e) {
        RCLCPP_ERROR(this->get_logger(), "Invalid pin number: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto center_pulse_width_str =
        util::find_param(params.hardware_info.hardware_parameters, "center_pulse_width");
    if (!center_pulse_width_str) {
        RCLCPP_ERROR(
            this->get_logger(), "Parameter 'center_pulse_width' not found in hardware parameters.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    int center_pulse_width;
    try {
        center_pulse_width = std::stoi(center_pulse_width_str.value());
    } catch (const std::invalid_argument & e) {
        RCLCPP_ERROR(this->get_logger(), "Invalid center pulse width: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto negative_pulse_width_radius_str =
        util::find_param(params.hardware_info.hardware_parameters, "negative_pulse_width_radius");
    if (!negative_pulse_width_radius_str) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Parameter 'negative_pulse_width_radius' not found in hardware parameters.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    int negative_pulse_width_radius;
    try {
        negative_pulse_width_radius = std::stoi(negative_pulse_width_radius_str.value());
    } catch (const std::invalid_argument & e) {
        RCLCPP_ERROR(this->get_logger(), "Invalid negative pulse width radius: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto positive_pulse_width_radius_str =
        util::find_param(params.hardware_info.hardware_parameters, "positive_pulse_width_radius");
    if (!positive_pulse_width_radius_str) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Parameter 'positive_pulse_width_radius' not found in hardware parameters.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    int positive_pulse_width_radius;
    try {
        positive_pulse_width_radius = std::stoi(positive_pulse_width_radius_str.value());
    } catch (const std::invalid_argument & e) {
        RCLCPP_ERROR(this->get_logger(), "Invalid positive pulse width radius: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    this->model.emplace(
        std::move(esc_pin), esc_pin_num, center_pulse_width, negative_pulse_width_radius,
        positive_pulse_width_radius);

    const auto res = this->model->on_init();
    if (!res) {
        RCLCPP_ERROR(
            this->get_logger(), "\n  Failed to initialize ESC Direct: %s", res.error().c_str());
        // GPIOの初期化に失敗した場合、モデルにnullを再代入する
        this->model.reset();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

auto thruster_direct::EscDirect::read(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*preiod*/) -> hardware_interface::return_type {
    if (this->model) this->model->on_read();
    return hardware_interface::return_type::OK;
}

auto thruster_direct::EscDirect::write(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/) -> hardware_interface::return_type {
    auto prefix = "thruster_direct" + std::to_string(this->id) + "/";

    auto enabled = util::from_interface_data<sinsei_umiusi_control::cmd::thruster::esc::Enabled>(
        this->get_command(prefix + "esc/enabled"));
    auto duty_cycle =
        util::from_interface_data<sinsei_umiusi_control::cmd::thruster::esc::DutyCycle>(
            this->get_command(prefix + "esc/duty_cycle"));

    if (!this->model) {
        this->set_state(
            prefix + "esc/health", util::to_interface_data(state::thruster::esc::Health{false}));

        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION,
            "\n  ESC Direct model is not initialized");
        return hardware_interface::return_type::OK;
    }

    const auto res = this->model->on_write(std::move(enabled), std::move(duty_cycle));
    if (!res) {
        this->set_state(
            prefix + "esc/health", util::to_interface_data(state::thruster::esc::Health{false}));

        constexpr auto DURATION = 3000;  // ms
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Failed to write ESC Direct: %s",
            res.error().c_str());

        return hardware_interface::return_type::OK;
    }

    return hardware_interface::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::hardware::thruster_direct::EscDirect,
    hardware_interface::SystemInterface)
