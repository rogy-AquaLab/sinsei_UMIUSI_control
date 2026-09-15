#include "sinsei_umiusi_control/hardware/can.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "sinsei_umiusi_control/hardware_model/impl/linux_can.hpp"
#include "sinsei_umiusi_control/state/can.hpp"
#include "sinsei_umiusi_control/util/params.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"
#include "sinsei_umiusi_control/util/string.hpp"

using namespace sinsei_umiusi_control::hardware;

namespace {

using ActuatorConfig = sinsei_umiusi_control::hardware_model::CanModel::ActuatorConfig;
using MotorType = sinsei_umiusi_control::hardware_model::CanModel::MotorType;

auto parse_motor_type(std::string motor_type) -> std::optional<MotorType> {
    std::transform(
        motor_type.begin(), motor_type.end(), motor_type.begin(),
        [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    if (motor_type == "dc") {
        return MotorType::DC;
    }
    if (motor_type == "bldc") {
        return MotorType::BLDC;
    }
    if (motor_type == "none") {
        return MotorType::None;
    }
    return std::nullopt;
}

auto has_interface(
    const std::vector<hardware_interface::InterfaceInfo> & interfaces,
    const std::string & name) -> bool {
    return std::any_of(interfaces.begin(), interfaces.end(), [&name](const auto & interface) {
        return interface.name == name;
    });
}

}  // namespace

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

    auto actuator_configs = std::vector<ActuatorConfig>{};
    auto actuators = std::vector<Actuator>{};
    auto vesc_ids = std::unordered_set<hardware_model::can::VescModel::Id>{};
    for (const auto & gpio : params.hardware_info.gpios) {
        const auto vesc_id_str = util::find_param(gpio.parameters, "vesc_id");
        if (!vesc_id_str) {
            continue;
        }

        const auto motor_type_str = util::find_param(gpio.parameters, "motor_type");
        if (!motor_type_str) {
            RCLCPP_ERROR(
                this->get_logger(), "Parameter 'motor_type' not found for actuator '%s'.",
                gpio.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        const auto motor_type = parse_motor_type(motor_type_str.value());
        if (!motor_type) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Invalid motor type '%s' for actuator '%s' (expected: dc, bldc, or none).",
                motor_type_str.value().c_str(), gpio.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        const auto vesc_id_res = util::from_chars_expected<unsigned int>(vesc_id_str.value());
        if (!vesc_id_res || vesc_id_res.value() > UINT8_MAX) {
            RCLCPP_ERROR(
                this->get_logger(), "Invalid VESC ID '%s' for actuator '%s'.",
                vesc_id_str.value().c_str(), gpio.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        const auto has_motor_allowed = has_interface(gpio.command_interfaces, "esc/allowed");
        const auto has_motor_duty = has_interface(gpio.command_interfaces, "esc/duty_cycle");
        const auto has_motor_rpm = has_interface(gpio.state_interfaces, "esc/rpm");
        const auto has_motor_voltage = has_interface(gpio.state_interfaces, "esc/voltage");
        const auto has_motor_water_leaked =
            has_interface(gpio.state_interfaces, "esc/water_leaked");
        const auto has_any_motor_interface = has_motor_allowed || has_motor_duty || has_motor_rpm ||
                                             has_motor_voltage || has_motor_water_leaked;
        const auto has_all_motor_interfaces = has_motor_allowed && has_motor_duty &&
                                              has_motor_rpm && has_motor_voltage &&
                                              has_motor_water_leaked;
        if ((*motor_type == MotorType::None && has_any_motor_interface) ||
            (*motor_type != MotorType::None && !has_all_motor_interfaces)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Motor interfaces for actuator '%s' do not match motor_type '%s'.",
                gpio.name.c_str(), motor_type_str.value().c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        const auto has_servo_allowed = has_interface(gpio.command_interfaces, "servo/allowed");
        const auto has_servo_angle = has_interface(gpio.command_interfaces, "servo/angle");
        if (has_servo_allowed != has_servo_angle) {
            RCLCPP_ERROR(
                this->get_logger(), "Incomplete servo interfaces for actuator '%s'.",
                gpio.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        const auto has_servo = has_servo_allowed && has_servo_angle;

        const auto vesc_id = static_cast<hardware_model::can::VescModel::Id>(vesc_id_res.value());
        if (!vesc_ids.insert(vesc_id).second) {
            RCLCPP_ERROR(
                this->get_logger(), "Duplicate VESC ID: %u.", static_cast<unsigned int>(vesc_id));
            return hardware_interface::CallbackReturn::ERROR;
        }

        actuator_configs.push_back(ActuatorConfig{gpio.name, vesc_id, *motor_type, has_servo});
        actuators.push_back(Actuator{gpio.name, *motor_type, has_servo});
    }
    if (actuator_configs.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No VESC actuators are configured.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Actuatorすべてに信号を`period_led_tape_per_actuators`回送るごとにLEDテープの信号を1回送る
    const auto period_led_tape_per_actuators_str =
        util::find_param(params.hardware_info.hardware_parameters, "period_led_tape_per_actuators");
    if (!period_led_tape_per_actuators_str) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Parameter 'period_led_tape_per_actuators' not found in hardware parameters.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto period_led_tape_per_actuators_res =
        util::from_chars_expected<size_t>(period_led_tape_per_actuators_str.value());
    if (!period_led_tape_per_actuators_res) {
        RCLCPP_ERROR(
            this->get_logger(), "Invalid value for `period_led_tape_per_actuators` (%s): %s",
            period_led_tape_per_actuators_str.value().c_str(),
            period_led_tape_per_actuators_res.error().c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto period_led_tape_per_actuators = period_led_tape_per_actuators_res.value();
    // `period_led_tape_per_actuators`が1以下のとき、特定のコマンドがLEDテープのコマンドに邪魔されて送れなくなってしまう。
    if (period_led_tape_per_actuators <= 1) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Invalid value for `period_led_tape_per_actuators` (%zu): must be greater than 1",
            period_led_tape_per_actuators);
        return hardware_interface::CallbackReturn::ERROR;
    }

    this->actuators = std::move(actuators);
    this->model.emplace(
        std::make_shared<hardware_model::impl::LinuxCan>(), std::move(actuator_configs),
        period_led_tape_per_actuators);

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
            const auto & [actuator_name, rpm] = std::get<0>(variant);
            this->set_state(actuator_name + "/esc/rpm", util::to_interface_data(rpm));
            break;
        }
        case 1: {  // Motor controller voltage
            const auto & [actuator_name, voltage] = std::get<1>(variant);
            this->set_state(actuator_name + "/esc/voltage", util::to_interface_data(voltage));
            break;
        }
        case 2: {  // Motor controller water leakage
            const auto & [actuator_name, water_leaked] = std::get<2>(variant);
            this->set_state(
                actuator_name + "/esc/water_leaked", util::to_interface_data(water_leaked));
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
        case 7: {  // IgnoredUpdate
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

    auto actuator_commands = std::vector<hardware_model::CanModel::ActuatorCommand>{};
    actuator_commands.reserve(this->actuators.size());
    for (const auto & actuator : this->actuators) {
        auto command = hardware_model::CanModel::ActuatorCommand{
            cmd::actuator::motor::Allowed{false}, cmd::actuator::motor::DutyCycle{0.0},
            cmd::actuator::servo::Allowed{false}, cmd::actuator::servo::Angle{0.0}};
        if (actuator.motor_type != hardware_model::CanModel::MotorType::None) {
            command.motor_allowed = util::from_interface_data<cmd::actuator::motor::Allowed>(
                this->get_command(actuator.name + "/esc/allowed"));
            command.motor_duty_cycle = util::from_interface_data<cmd::actuator::motor::DutyCycle>(
                this->get_command(actuator.name + "/esc/duty_cycle"));
        }
        if (actuator.has_servo) {
            command.servo_allowed = util::from_interface_data<cmd::actuator::servo::Allowed>(
                this->get_command(actuator.name + "/servo/allowed"));
            command.servo_angle = util::from_interface_data<cmd::actuator::servo::Angle>(
                this->get_command(actuator.name + "/servo/angle"));
        }
        actuator_commands.push_back(command);
    }

    const auto res = this->model->on_write(main_power_enabled, actuator_commands, led_tape_color);
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
