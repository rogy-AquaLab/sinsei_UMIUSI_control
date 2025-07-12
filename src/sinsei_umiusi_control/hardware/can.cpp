#include "sinsei_umiusi_control/hardware/can.hpp"

#include "sinsei_umiusi_control/util/linux_can.hpp"

namespace suchw = sinsei_umiusi_control::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

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

    return hif::CallbackReturn::SUCCESS;
}

auto suchw::Can::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hif::return_type {
    auto res = this->model->on_read();
    if (res) {
        auto [servo_current, rpm, battery_current, battery_voltage, temperature, water_leaked] =
            res.value();
        if (this->thruster_mode == util::ThrusterMode::Can) {
            this->set_state(
                "thruster1/servo/servo_current_raw",
                *reinterpret_cast<const double *>(&servo_current[0]));
            this->set_state(
                "thruster2/servo/servo_current_raw",
                *reinterpret_cast<const double *>(&servo_current[1]));
            this->set_state(
                "thruster3/servo/servo_current_raw",
                *reinterpret_cast<const double *>(&servo_current[2]));
            this->set_state(
                "thruster4/servo/servo_current_raw",
                *reinterpret_cast<const double *>(&servo_current[3]));
            this->set_state("thruster1/esc/rpm_raw", *reinterpret_cast<const double *>(&rpm[0]));
            this->set_state("thruster2/esc/rpm_raw", *reinterpret_cast<const double *>(&rpm[1]));
            this->set_state("thruster3/esc/rpm_raw", *reinterpret_cast<const double *>(&rpm[2]));
            this->set_state("thruster4/esc/rpm_raw", *reinterpret_cast<const double *>(&rpm[3]));
        }
        this->set_state(
            "main_power/battery_current", *reinterpret_cast<const double *>(&battery_current));
        this->set_state(
            "main_power/battery_voltage", *reinterpret_cast<const double *>(&battery_voltage));
        this->set_state("main_power/temperature", *reinterpret_cast<const double *>(&temperature));
        this->set_state(
            "main_power/water_leaked", *reinterpret_cast<const double *>(&water_leaked));
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to read CAN data: %s", res.error().c_str());
    }
    return hif::return_type::OK;
}

auto suchw::Can::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    -> hif::return_type {
    auto main_power_enabled = this->get_command("main_power/enabled");
    auto led_tape_color = this->get_command("led_tape/color");

    auto main_power_enabled_cmd =
        *reinterpret_cast<sinsei_umiusi_control::cmd::main_power::Enabled *>(&main_power_enabled);
    auto led_tape_color_cmd =
        *reinterpret_cast<sinsei_umiusi_control::cmd::led_tape::Color *>(&led_tape_color);

    if (this->thruster_mode == util::ThrusterMode::Direct) {
        if (this->model.has_value()) {
            this->model->on_write(main_power_enabled_cmd, led_tape_color_cmd);
        };
    } else {
        auto thruster1_esc_enabled_raw = this->get_command("thruster1/esc/enabled_raw");
        auto thruster2_esc_enabled_raw = this->get_command("thruster2/esc/enabled_raw");
        auto thruster3_esc_enabled_raw = this->get_command("thruster3/esc/enabled_raw");
        auto thruster4_esc_enabled_raw = this->get_command("thruster4/esc/enabled_raw");
        auto thruster1_servo_enabled_raw = this->get_command("thruster1/servo/enabled_raw");
        auto thruster2_servo_enabled_raw = this->get_command("thruster2/servo/enabled_raw");
        auto thruster3_servo_enabled_raw = this->get_command("thruster3/servo/enabled_raw");
        auto thruster4_servo_enabled_raw = this->get_command("thruster4/servo/enabled_raw");
        auto thruster1_angle_raw = this->get_command("thruster1/servo/angle_raw");
        auto thruster2_angle_raw = this->get_command("thruster2/servo/angle_raw");
        auto thruster3_angle_raw = this->get_command("thruster3/servo/angle_raw");
        auto thruster4_angle_raw = this->get_command("thruster4/servo/angle_raw");
        auto thruster1_thrust_raw = this->get_command("thruster1/esc/thrust_raw");
        auto thruster2_thrust_raw = this->get_command("thruster2/esc/thrust_raw");
        auto thruster3_thrust_raw = this->get_command("thruster3/esc/thrust_raw");
        auto thruster4_thrust_raw = this->get_command("thruster4/esc/thrust_raw");

        std::array<suc::cmd::thruster::EscEnabled, 4> thruster_esc_enabled_cmd = {
            *reinterpret_cast<sinsei_umiusi_control::cmd::thruster::EscEnabled *>(
                &thruster1_esc_enabled_raw),
            *reinterpret_cast<sinsei_umiusi_control::cmd::thruster::EscEnabled *>(
                &thruster2_esc_enabled_raw),
            *reinterpret_cast<sinsei_umiusi_control::cmd::thruster::EscEnabled *>(
                &thruster3_esc_enabled_raw),
            *reinterpret_cast<sinsei_umiusi_control::cmd::thruster::EscEnabled *>(
                &thruster4_esc_enabled_raw)};
        std::array<suc::cmd::thruster::ServoEnabled, 4> thruster_servo_enabled_cmd = {
            *reinterpret_cast<sinsei_umiusi_control::cmd::thruster::ServoEnabled *>(
                &thruster1_servo_enabled_raw),
            *reinterpret_cast<sinsei_umiusi_control::cmd::thruster::ServoEnabled *>(
                &thruster2_servo_enabled_raw),
            *reinterpret_cast<sinsei_umiusi_control::cmd::thruster::ServoEnabled *>(
                &thruster3_servo_enabled_raw),
            *reinterpret_cast<sinsei_umiusi_control::cmd::thruster::ServoEnabled *>(
                &thruster4_servo_enabled_raw)};
        std::array<suc::cmd::thruster::Angle, 4> thruster_angle_cmd = {
            *reinterpret_cast<sinsei_umiusi_control::cmd::thruster::Angle *>(&thruster1_angle_raw),
            *reinterpret_cast<sinsei_umiusi_control::cmd::thruster::Angle *>(&thruster2_angle_raw),
            *reinterpret_cast<sinsei_umiusi_control::cmd::thruster::Angle *>(&thruster3_angle_raw),
            *reinterpret_cast<sinsei_umiusi_control::cmd::thruster::Angle *>(&thruster4_angle_raw)};
        std::array<suc::cmd::thruster::Thrust, 4> thruster_thrust_cmd = {
            *reinterpret_cast<sinsei_umiusi_control::cmd::thruster::Thrust *>(
                &thruster1_thrust_raw),
            *reinterpret_cast<sinsei_umiusi_control::cmd::thruster::Thrust *>(
                &thruster2_thrust_raw),
            *reinterpret_cast<sinsei_umiusi_control::cmd::thruster::Thrust *>(
                &thruster3_thrust_raw),
            *reinterpret_cast<sinsei_umiusi_control::cmd::thruster::Thrust *>(
                &thruster4_thrust_raw)};

        if (this->model.has_value()) {
            this->model->on_write(
                thruster_esc_enabled_cmd, thruster_servo_enabled_cmd, thruster_angle_cmd,
                thruster_thrust_cmd, main_power_enabled_cmd, led_tape_color_cmd);
        };
    }

    return hif::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sinsei_umiusi_control::hardware::Can, hardware_interface::SystemInterface)