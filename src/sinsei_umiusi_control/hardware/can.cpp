#include "sinsei_umiusi_control/hardware/can.hpp"

#include "sinsei_umiusi_control/cmd/thruster.hpp"
#include "sinsei_umiusi_control/hardware_model/impl/linux_can.hpp"
#include "sinsei_umiusi_control/util/params.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"

namespace suchw = sinsei_umiusi_control::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

suchw::Can::~Can() {
    if (!this->model) {
        RCLCPP_ERROR(this->get_logger(), "Can model is not initialized.");
        return;
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

    const auto thruster_mode = util::find_param(info.hardware_parameters, "thruster_mode");
    if (!thruster_mode) {
        RCLCPP_ERROR(
            this->get_logger(), "Parameter 'thruster_mode' not found in hardware parameters.");
        return hif::CallbackReturn::ERROR;
    }
    const auto mode_res = util::get_mode_from_str(thruster_mode.value());
    if (!mode_res) {
        RCLCPP_ERROR(this->get_logger(), "Invalid thruster mode: %s", mode_res.error().c_str());
        return hif::CallbackReturn::ERROR;
    }
    this->thruster_mode = mode_res.value();

    std::array<int, 4> vesc_ids;
    for (size_t i = 0; i < 4; ++i) {
        auto vesc_id_key = "vesc" + std::to_string(i + 1) + "_id";
        auto vesc_id_str = util::find_param(info.hardware_parameters, vesc_id_key);
        if (!vesc_id_str) {
            RCLCPP_ERROR(
                this->get_logger(), "Parameter '%s' not found in hardware parameters.",
                vesc_id_key.c_str());
            return hif::CallbackReturn::ERROR;
        }
        try {
            vesc_ids[i] = std::stoi(vesc_id_str.value());
        } catch (const std::invalid_argument & e) {
            RCLCPP_ERROR(
                this->get_logger(), "Invalid VESC ID '%s': %s", vesc_id_str.value().c_str(),
                e.what());
            return hif::CallbackReturn::ERROR;
        }
    }

    this->model.emplace(std::make_shared<sinsei_umiusi_control::util::LinuxCan>(), vesc_ids);

    auto res = this->model->on_init();
    if (!res) {
        RCLCPP_ERROR(this->get_logger(), "\n  Failed to initialize Can: %s", res.error().c_str());
        // CANの初期化に失敗した場合、モデルにnullを再代入する
        this->model.reset();
    }

    return hif::CallbackReturn::SUCCESS;
}

auto suchw::Can::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hif::return_type {
    if (!this->model) {
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

    auto [rpm, battery_current, battery_voltage, temperature, water_leaked] = res.value();
    if (this->thruster_mode == util::ThrusterMode::Can) {
        for (size_t i = 0; i < 4; ++i) {
            auto thruster_name = "thruster" + std::to_string(i + 1);
            this->set_state(thruster_name + "/esc/rpm", util::to_interface_data(rpm[i]));
        }
    }
    // TODO: 実装したらコメントアウトを外す
    // this->set_state(
    //     "main_power/battery_current", util::to_interface_data(battery_current));
    // this->set_state(
    //     "main_power/battery_voltage", util::to_interface_data(battery_voltage));
    // this->set_state("main_power/temperature", util::to_interface_data(temperature));
    // this->set_state("main_power/water_leaked", util::to_interface_data(water_leaked));

    return hif::return_type::OK;
}

auto suchw::Can::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    -> hif::return_type {
    auto is_can = this->thruster_mode == util::ThrusterMode::Can;

    std::array<suc::cmd::thruster::EscEnabled, 4> thruster_esc_enabled_cmd;
    std::array<suc::cmd::thruster::ServoEnabled, 4> thruster_servo_enabled_cmd;
    std::array<suc::cmd::thruster::Angle, 4> thruster_angle_cmd;
    std::array<suc::cmd::thruster::DutyCycle, 4> thruster_duty_cycle_cmd;

    if (is_can) {
        for (size_t i = 0; i < 4; ++i) {
            auto thruster_name = "thruster" + std::to_string(i + 1);
            auto esc_enabled_raw = this->get_command(thruster_name + "/esc/enabled");
            auto servo_enabled_raw = this->get_command(thruster_name + "/servo/enabled");
            auto angle_raw = this->get_command(thruster_name + "/servo/angle");
            auto duty_cycle_raw = this->get_command(thruster_name + "/esc/duty_cycle");

            thruster_esc_enabled_cmd[i] =
                util::from_interface_data<sinsei_umiusi_control::cmd::thruster::EscEnabled>(
                    esc_enabled_raw);
            thruster_servo_enabled_cmd[i] =
                util::from_interface_data<sinsei_umiusi_control::cmd::thruster::ServoEnabled>(
                    servo_enabled_raw);
            thruster_angle_cmd[i] =
                util::from_interface_data<sinsei_umiusi_control::cmd::thruster::Angle>(angle_raw);
            thruster_duty_cycle_cmd[i] =
                util::from_interface_data<sinsei_umiusi_control::cmd::thruster::DutyCycle>(
                    duty_cycle_raw);
        }
    }

    auto main_power_enabled = this->get_command("main_power/enabled");
    auto led_tape_color = this->get_command("led_tape/color");

    auto main_power_enabled_cmd =
        util::from_interface_data<sinsei_umiusi_control::cmd::main_power::Enabled>(
            main_power_enabled);
    auto led_tape_color_cmd =
        util::from_interface_data<sinsei_umiusi_control::cmd::led_tape::Color>(led_tape_color);

    if (!this->model) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Can model is not initialized");
        return hif::return_type::OK;
    }

    auto res = is_can
                   ? this->model->on_write(
                         thruster_esc_enabled_cmd, thruster_servo_enabled_cmd, thruster_angle_cmd,
                         thruster_duty_cycle_cmd, main_power_enabled_cmd, led_tape_color_cmd)
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