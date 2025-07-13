#include "sinsei_umiusi_control/hardware/can.hpp"

#include "sinsei_umiusi_control/util/linux_can.hpp"

namespace suchw = sinsei_umiusi_control::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

suchw::Can::~Can() {
    if (!this->model.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "Can model is not initialized.");
    }

    auto res = this->model->on_destroy();
    if (!res) {
        RCLCPP_ERROR(
            this->get_logger(), "\n  Failed to destroy Can model: %s", res.error().c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Can model destroyed successfully.");
    }
}

auto suchw::Can::on_init(const hif::HardwareInfo & info) -> hif::CallbackReturn {
    this->hif::SystemInterface::on_init(info);

    auto thruster_mode = info.hardware_parameters.find("thruster_mode");
    if (thruster_mode == info.hardware_parameters.end()) {
        RCLCPP_ERROR(
            this->get_logger(), "Parameter 'thruster_mode' not found in hardware parameters. ");
        return hif::CallbackReturn::ERROR;
    }
    auto mode_res = util::get_mode_from_str(thruster_mode->second);
    if (!mode_res) {
        RCLCPP_ERROR(this->get_logger(), "Invalid thruster mode: %s", mode_res.error().c_str());
        return hif::CallbackReturn::ERROR;
    }
    this->thruster_mode = mode_res.value();

    this->model.emplace(
        std::make_shared<sinsei_umiusi_control::util::LinuxCan>(), this->thruster_mode);

    auto res = this->model->on_init();
    if (!res) {
        RCLCPP_ERROR(this->get_logger(), "\n  Failed to initialize Can: %s", res.error().c_str());
    }

    return hif::CallbackReturn::SUCCESS;
}

auto suchw::Can::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hif::return_type {
    if (!this->model.has_value()) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Can model is not initialized");
        return hif::return_type::OK;
    }

    auto res = this->model->on_read();
    if (!res) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Failed to read CAN data: %s",
            res.error().c_str());
        return hif::return_type::OK;
    }

    auto [servo_current, rpm, battery_current, battery_voltage, temperature, water_leaked] =
        res.value();
    if (this->thruster_mode == util::ThrusterMode::Can) {
        for (size_t i = 0; i < 4; ++i) {
            auto thruster_name = "thruster" + std::to_string(i + 1);
            // TODO: 実装したらコメントアウトを外す
            /*this->set_state(
                thruster_name + "/servo/servo_current_raw",
                *reinterpret_cast<const double *>(&servo_current[i]));*/
            this->set_state(
                thruster_name + "/esc/rpm_raw", *reinterpret_cast<const double *>(&rpm[i]));
        }
    }
    // TODO: 実装したらコメントアウトを外す
    // this->set_state(
    //     "main_power/battery_current", *reinterpret_cast<const double *>(&battery_current));
    // this->set_state(
    //     "main_power/battery_voltage", *reinterpret_cast<const double *>(&battery_voltage));
    // this->set_state("main_power/temperature", *reinterpret_cast<const double *>(&temperature));
    // this->set_state("main_power/water_leaked", *reinterpret_cast<const double *>(&water_leaked));

    return hif::return_type::OK;
}

auto suchw::Can::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    -> hif::return_type {
    auto is_can = this->thruster_mode == util::ThrusterMode::Can;

    std::array<suc::cmd::thruster::EscEnabled, 4> thruster_esc_enabled_cmd;
    std::array<suc::cmd::thruster::ServoEnabled, 4> thruster_servo_enabled_cmd;
    std::array<suc::cmd::thruster::Angle, 4> thruster_angle_cmd;
    std::array<suc::cmd::thruster::Thrust, 4> thruster_thrust_cmd;

    if (is_can) {
        for (size_t i = 0; i < 4; ++i) {
            auto thruster_name = "thruster" + std::to_string(i + 1);
            auto esc_enabled_raw = this->get_command(thruster_name + "/esc/enabled_raw");
            auto servo_enabled_raw = this->get_command(thruster_name + "/servo/enabled_raw");
            auto angle_raw = this->get_command(thruster_name + "/servo/angle_raw");
            auto thrust_raw = this->get_command(thruster_name + "/esc/thrust_raw");

            thruster_esc_enabled_cmd[i] =
                *reinterpret_cast<sinsei_umiusi_control::cmd::thruster::EscEnabled *>(
                    &esc_enabled_raw);
            thruster_servo_enabled_cmd[i] =
                *reinterpret_cast<sinsei_umiusi_control::cmd::thruster::ServoEnabled *>(
                    &servo_enabled_raw);
            thruster_angle_cmd[i] =
                *reinterpret_cast<sinsei_umiusi_control::cmd::thruster::Angle *>(&angle_raw);
            thruster_thrust_cmd[i] =
                *reinterpret_cast<sinsei_umiusi_control::cmd::thruster::Thrust *>(&thrust_raw);
        }
    }

    auto main_power_enabled = this->get_command("main_power/enabled");
    auto led_tape_color = this->get_command("led_tape/color");

    auto main_power_enabled_cmd =
        *reinterpret_cast<sinsei_umiusi_control::cmd::main_power::Enabled *>(&main_power_enabled);
    auto led_tape_color_cmd =
        *reinterpret_cast<sinsei_umiusi_control::cmd::led_tape::Color *>(&led_tape_color);

    if (!this->model.has_value()) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Can model is not initialized");
        return hif::return_type::OK;
    }

    auto res = is_can
                   ? this->model->on_write(
                         thruster_esc_enabled_cmd, thruster_servo_enabled_cmd, thruster_angle_cmd,
                         thruster_thrust_cmd, main_power_enabled_cmd, led_tape_color_cmd)
                   : this->model->on_write(main_power_enabled_cmd, led_tape_color_cmd);
    if (!res) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Failed to write Can: %s",
            res.error().c_str());
    }

    return hif::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sinsei_umiusi_control::hardware::Can, hardware_interface::SystemInterface)