#include "sinsei_umiusi_control/hardware/can.hpp"

#include <cstdint>
#include <utility>
#include <vector>

#include "sinsei_umiusi_control/hardware_model/impl/linux_can.hpp"
#include "sinsei_umiusi_control/state/can.hpp"
#include "sinsei_umiusi_control/util/params.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"
#include "sinsei_umiusi_control/util/string.hpp"

using namespace sinsei_umiusi_control::hardware;

Can::~Can() {
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

auto Can::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
    -> hardware_interface::CallbackReturn {
    this->hardware_interface::SystemInterface::on_init(params);

    // FIXME: URDF側での名前付きGPIO設定を導入するまでは、既存のハードウェアパラメータを維持する。
    // CanModel自体は、この固定長の表現には依存していない
    constexpr size_t LEGACY_THRUSTER_COUNT = 4;
    auto thruster_configs = std::vector<hardware_model::CanModel::ThrusterConfig>{};
    auto thruster_names = std::vector<std::string>{};
    thruster_configs.reserve(LEGACY_THRUSTER_COUNT);
    thruster_names.reserve(LEGACY_THRUSTER_COUNT);
    for (size_t i = 0; i < LEGACY_THRUSTER_COUNT; ++i) {
        auto vesc_id_key = "vesc" + std::to_string(i + 1) + "_id";
        auto vesc_id_str = util::find_param(params.hardware_info.hardware_parameters, vesc_id_key);
        if (!vesc_id_str) {
            RCLCPP_ERROR(
                this->get_logger(), "Parameter '%s' not found in hardware parameters.",
                vesc_id_key.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        const auto vesc_id_res = util::from_chars_expected<unsigned int>(vesc_id_str.value());
        if (!vesc_id_res || vesc_id_res.value() > UINT8_MAX) {
            RCLCPP_ERROR(
                this->get_logger(), "Invalid VESC ID '%s' for parameter '%s'",
                vesc_id_str.value().c_str(), vesc_id_key.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        const auto thruster_name = "thruster" + std::to_string(i + 1);
        thruster_configs.push_back(hardware_model::CanModel::ThrusterConfig{
            thruster_name,
            static_cast<hardware_model::can::VescModel::Id>(vesc_id_res.value()),
        });
        thruster_names.push_back(thruster_name);
    }

    // Thrusterすべてに信号を`period_led_tape_per_thrusters`回送るごとにLEDテープの信号を1回送る
    const auto period_led_tape_per_thrusters_str =
        util::find_param(params.hardware_info.hardware_parameters, "period_led_tape_per_thrusters");
    if (!period_led_tape_per_thrusters_str) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Parameter 'period_led_tape_per_thrusters' not found in hardware parameters.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    size_t period_led_tape_per_thrusters = 0;
    try {
        period_led_tape_per_thrusters =
            static_cast<size_t>(std::stoi(period_led_tape_per_thrusters_str.value()));
    } catch (const std::invalid_argument & e) {
        RCLCPP_ERROR(
            this->get_logger(), "Invalid value for `period_led_tape_per_thrusters` (%s): %s",
            period_led_tape_per_thrusters_str.value().c_str(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    // `period_led_tape_per_thrusters`が1以下のとき、特定のコマンドがLEDテープのコマンドに邪魔されて送れなくなってしまう。
    if (period_led_tape_per_thrusters <= 1) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Invalid value for `period_led_tape_per_thrusters` (%zu): must be greater than 1",
            period_led_tape_per_thrusters);
        return hardware_interface::CallbackReturn::ERROR;
    }

    this->thruster_names = std::move(thruster_names);
    this->model.emplace(
        std::make_shared<hardware_model::impl::LinuxCan>(), std::move(thruster_configs),
        period_led_tape_per_thrusters);

    auto res = this->model->on_init();
    if (!res) {
        RCLCPP_ERROR(this->get_logger(), "\n  Failed to initialize Can: %s", res.error().c_str());
        // CANの初期化に失敗した場合、モデルにnullを再代入する
        this->model.reset();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

auto Can::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hardware_interface::return_type {
    if (!this->model) {
        this->set_state("can/health", util::to_interface_data(state::can::Health{false}));

        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Can model is not initialized");
        return hardware_interface::return_type::OK;
    }

    auto res = this->model->on_read();
    if (!res) {
        this->set_state("can/health", util::to_interface_data(state::can::Health{false}));

        constexpr auto DURATION = 3000;  // ms
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Failed to read CAN data: %s",
            res.error().c_str());
        return hardware_interface::return_type::OK;
    }
    this->set_state("can/health", util::to_interface_data(state::can::Health{true}));

    auto variant = res.value();

    switch (variant.index()) {
        case 0: {  // Rpm
            const auto & [thruster_name, rpm] = std::get<0>(variant);
            this->set_state(thruster_name + "/esc/rpm", util::to_interface_data(rpm));
            break;
        }
        case 1: {  // ESC Voltage
            const auto & [thruster_name, voltage] = std::get<1>(variant);
            this->set_state(thruster_name + "/esc/voltage", util::to_interface_data(voltage));
            break;
        }
        case 2: {  // ESC WaterLeaked
            const auto & [thruster_name, water_leaked] = std::get<2>(variant);
            this->set_state(
                thruster_name + "/esc/water_leaked", util::to_interface_data(water_leaked));
            break;
        }
        case 3: {  // BatteryCurrent
            const auto battery_current =
                std::get<sinsei_umiusi_control::state::main_power::BatteryCurrent>(variant);
            this->set_state("main_power/battery_current", util::to_interface_data(battery_current));
            break;
        }
        case 4: {  // BatteryVoltage
            const auto battery_voltage =
                std::get<sinsei_umiusi_control::state::main_power::BatteryVoltage>(variant);
            this->set_state("main_power/battery_voltage", util::to_interface_data(battery_voltage));
            break;
        }
        case 5: {  // Temperature
            const auto temperature =
                std::get<sinsei_umiusi_control::state::main_power::Temperature>(variant);
            this->set_state("main_power/temperature", util::to_interface_data(temperature));
            break;
        }
        case 6: {  // WaterLeaked
            const auto water_leaked =
                std::get<sinsei_umiusi_control::state::main_power::WaterLeaked>(variant);
            this->set_state("main_power/water_leaked", util::to_interface_data(water_leaked));
            break;
        }
    }

    return hardware_interface::return_type::OK;
}

auto Can::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    -> hardware_interface::return_type {
    if (!this->model) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Can model is not initialized");
        return hardware_interface::return_type::OK;
    }

    auto && main_power_enabled = util::from_interface_data<cmd::main_power::Enabled>(
        this->get_command("main_power/enabled"));
    auto && led_tape_color =
        util::from_interface_data<cmd::led_tape::Color>(this->get_command("led_tape/color"));

    auto thruster_commands = std::vector<hardware_model::CanModel::ThrusterCommand>{};
    thruster_commands.reserve(this->thruster_names.size());
    for (const auto & name : this->thruster_names) {
        thruster_commands.push_back(hardware_model::CanModel::ThrusterCommand{
            util::from_interface_data<cmd::thruster::esc::Allowed>(
                this->get_command(name + "/esc/allowed")),
            util::from_interface_data<cmd::thruster::esc::DutyCycle>(
                this->get_command(name + "/esc/duty_cycle")),
            util::from_interface_data<cmd::thruster::servo::Allowed>(
                this->get_command(name + "/servo/allowed")),
            util::from_interface_data<cmd::thruster::servo::Angle>(
                this->get_command(name + "/servo/angle")),
        });
    }

    const auto res = this->model->on_write(main_power_enabled, thruster_commands, led_tape_color);
    if (!res) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Failed to write Can: %s",
            res.error().c_str());
        return hardware_interface::return_type::OK;
    }

    return hardware_interface::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sinsei_umiusi_control::hardware::Can, hardware_interface::SystemInterface)
