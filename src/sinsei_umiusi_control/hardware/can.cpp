#include "sinsei_umiusi_control/hardware/can.hpp"

#include "sinsei_umiusi_control/cmd/main_power.hpp"
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

    // Thrusterすべてに信号を`period_led_tape_per_thrusters`回送るごとにLEDテープの信号を1回送る
    const auto period_led_tape_per_thrusters_str =
        util::find_param(info.hardware_parameters, "period_led_tape_per_thrusters");
    if (!period_led_tape_per_thrusters_str) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Parameter 'period_led_tape_per_thrusters' not found in hardware parameters.");
        return hif::CallbackReturn::ERROR;
    }
    size_t period_led_tape_per_thrusters = 0;
    try {
        period_led_tape_per_thrusters =
            static_cast<size_t>(std::stoi(period_led_tape_per_thrusters_str.value()));
    } catch (const std::invalid_argument & e) {
        RCLCPP_ERROR(
            this->get_logger(), "Invalid value for `period_led_tape_per_thrusters` (%s): %s",
            period_led_tape_per_thrusters_str.value().c_str(), e.what());
        return hif::CallbackReturn::ERROR;
    }
    // `period_led_tape_per_thrusters`が1以下のとき、特定のコマンドがLEDテープのコマンドに邪魔されて送れなくなってしまう。
    if (period_led_tape_per_thrusters <= 1) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Invalid value for `period_led_tape_per_thrusters` (%zu): must be greater than 1",
            period_led_tape_per_thrusters);
        return hif::CallbackReturn::ERROR;
    }

    this->model.emplace(
        std::make_shared<hardware_model::impl::LinuxCan>(), vesc_ids,
        period_led_tape_per_thrusters);

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

    auto variant = res.value();

    switch (variant.index()) {
        case 0: {  // Rpm
            const auto [index, rpm] = std::get<0>(variant);
            const auto thruster_name = "thruster" + std::to_string(index + 1);
            this->set_state(thruster_name + "/esc/rpm", util::to_interface_data(rpm));
            break;
        }
        case 1: {  // ESC WaterLeaked
            const auto [index, water_leaked] = std::get<1>(variant);
            const auto thruster_name = "thruster" + std::to_string(index + 1);
            this->set_state(
                thruster_name + "/esc/water_leaked", util::to_interface_data(water_leaked));
            break;
        }
        case 2: {  // BatteryCurrent
            const auto battery_current =
                std::get<sinsei_umiusi_control::state::main_power::BatteryCurrent>(variant);
            this->set_state("main_power/battery_current", util::to_interface_data(battery_current));
            break;
        }
        case 3: {  // BatteryVoltage
            const auto battery_voltage =
                std::get<sinsei_umiusi_control::state::main_power::BatteryVoltage>(variant);
            this->set_state("main_power/battery_voltage", util::to_interface_data(battery_voltage));
            break;
        }
        case 4: {  // Temperature
            const auto temperature =
                std::get<sinsei_umiusi_control::state::main_power::Temperature>(variant);
            this->set_state("main_power/temperature", util::to_interface_data(temperature));
            break;
        }
        case 5: {  // WaterLeaked
            const auto water_leaked =
                std::get<sinsei_umiusi_control::state::main_power::WaterLeaked>(variant);
            this->set_state("main_power/water_leaked", util::to_interface_data(water_leaked));
            break;
        }
    }

    return hif::return_type::OK;
}

auto suchw::Can::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    -> hif::return_type {
    if (!this->model) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Can model is not initialized");
        return hif::return_type::OK;
    }

    auto && main_power_enabled = util::from_interface_data<cmd::main_power::Enabled>(
        this->get_command("main_power/enabled"));
    auto && led_tape_color =
        util::from_interface_data<cmd::led_tape::Color>(this->get_command("led_tape/color"));

    switch (this->thruster_mode) {
        case util::ThrusterMode::Can: {
            auto thruster_name = [](size_t i) { return "thruster" + std::to_string(i + 1); };

            auto && thruster_esc_enabled = std::array<cmd::thruster::EscEnabled, 4>{};
            auto && thruster_servo_enabled = std::array<cmd::thruster::ServoEnabled, 4>{};
            auto && thruster_duty_cycle = std::array<cmd::thruster::DutyCycle, 4>{};
            auto && thruster_angle = std::array<cmd::thruster::Angle, 4>{};

            for (size_t i = 0; i < 4; ++i) {
                thruster_esc_enabled[i] = util::from_interface_data<cmd::thruster::EscEnabled>(
                    this->get_command(thruster_name(i) + "/esc/enabled"));
                thruster_servo_enabled[i] = util::from_interface_data<cmd::thruster::ServoEnabled>(
                    this->get_command(thruster_name(i) + "/servo/enabled"));
                thruster_duty_cycle[i] = util::from_interface_data<cmd::thruster::DutyCycle>(
                    this->get_command(thruster_name(i) + "/esc/duty_cycle"));
                thruster_angle[i] = util::from_interface_data<cmd::thruster::Angle>(
                    this->get_command(thruster_name(i) + "/servo/angle"));
            }

            const auto res = this->model->on_write(
                std::move(main_power_enabled), std::move(thruster_esc_enabled),
                std::move(thruster_servo_enabled), std::move(thruster_duty_cycle),
                std::move(thruster_angle), std::move(led_tape_color));
            if (!res) {
                constexpr auto DURATION = 3000;  // ms
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), *this->get_clock(), DURATION, "\n  Failed to write Can: %s",
                    res.error().c_str());
                return hif::return_type::OK;
            }
            break;
        }

        case util::ThrusterMode::Direct: {  // this->thruster_mode == util::ThrusterMode::Direct
            const auto res =
                this->model->on_write(std::move(main_power_enabled), std::move(led_tape_color));
            if (!res) {
                constexpr auto DURATION = 3000;  // ms
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), *this->get_clock(), DURATION, "\n  Failed to write Can: %s",
                    res.error().c_str());
                return hif::return_type::OK;
            }
            break;
        }

        default: {
            return hif::return_type::ERROR;  // unreachable
        }
    }
    return hif::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sinsei_umiusi_control::hardware::Can, hardware_interface::SystemInterface)
