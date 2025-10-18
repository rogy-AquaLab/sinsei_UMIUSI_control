#include "sinsei_umiusi_control/hardware/thruster_direct/servo_direct.hpp"

#include "sinsei_umiusi_control/hardware_model/impl/pigpio.hpp"
#include "sinsei_umiusi_control/state/thruster/servo.hpp"
#include "sinsei_umiusi_control/util/params.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"

using namespace sinsei_umiusi_control::hardware;

auto thruster_direct::ServoDirect::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
    -> hardware_interface::CallbackReturn {
    this->hardware_interface::SystemInterface::on_init(params);

    auto servo_pin = std::make_unique<sinsei_umiusi_control::hardware_model::impl::Pigpio>();

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

    const auto servo_pin_num_str =
        util::find_param(params.hardware_info.hardware_parameters, "pin");
    if (!servo_pin_num_str) {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'pin' not found in hardware parameters.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    int servo_pin_num;
    try {
        servo_pin_num = std::stoi(servo_pin_num_str.value());
    } catch (const std::invalid_argument & e) {
        RCLCPP_ERROR(this->get_logger(), "Invalid pin number: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    this->model.emplace(std::move(servo_pin), servo_pin_num);

    const auto res = this->model->on_init();
    if (!res) {
        RCLCPP_ERROR(
            this->get_logger(), "\n  Failed to initialize Servo Direct: %s", res.error().c_str());
        // GPIOの初期化に失敗した場合、モデルにnullを再代入する
        this->model.reset();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

auto thruster_direct::ServoDirect::read(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*preiod*/) -> hardware_interface::return_type {
    if (this->model) this->model->on_read();
    return hardware_interface::return_type::OK;
}

auto thruster_direct::ServoDirect::write(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/) -> hardware_interface::return_type {
    auto prefix = "thruster_direct" + std::to_string(this->id) + "/";

    auto enabled = util::from_interface_data<sinsei_umiusi_control::cmd::thruster::servo::Enabled>(
        this->get_command(prefix + "servo/enabled"));
    auto angle = util::from_interface_data<sinsei_umiusi_control::cmd::thruster::servo::Angle>(
        this->get_command(prefix + "servo/angle"));

    if (!this->model) {
        this->set_state(
            prefix + "servo/health",
            util::to_interface_data(state::thruster::servo::Health{false}));

        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION,
            "\n  Servo Direct model is not initialized");
        return hardware_interface::return_type::OK;
    }

    const auto res = this->model->on_write(std::move(enabled), std::move(angle));
    if (!res) {
        this->set_state(
            prefix + "servo/health",
            util::to_interface_data(state::thruster::servo::Health{false}));

        constexpr auto DURATION = 3000;  // ms
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION,
            "\n  Failed to write Servo Direct: %s", res.error().c_str());

        return hardware_interface::return_type::OK;
    }

    return hardware_interface::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::hardware::thruster_direct::ServoDirect,
    hardware_interface::SystemInterface)
