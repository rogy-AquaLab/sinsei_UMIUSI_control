#include "sinsei_umiusi_control/hardware/can.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "sinsei_umiusi_control/hardware_model/impl/linux_can.hpp"
#include "sinsei_umiusi_control/state/bms.hpp"
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
        const auto has_motor_health = has_interface(gpio.state_interfaces, "esc/health");
        const auto has_any_motor_interface = has_motor_allowed || has_motor_duty || has_motor_rpm ||
                                             has_motor_voltage || has_motor_water_leaked ||
                                             has_motor_health;
        const auto has_all_motor_interfaces = has_motor_allowed && has_motor_duty &&
                                              has_motor_rpm && has_motor_voltage &&
                                              has_motor_water_leaked && has_motor_health;
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

    this->actuators = std::move(actuators);
    this->cycles_since_bms_update = MAX_CYCLES_SINCE_NODE_UPDATE;
    this->cycles_since_vesc_update.assign(this->actuators.size(), MAX_CYCLES_SINCE_NODE_UPDATE);

    auto harmony_bms_id_str =
        util::find_param(params.hardware_info.hardware_parameters, "harmony_bms_id");
    if (!harmony_bms_id_str) {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'harmony_bms_id' not found.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto harmony_bms_id_res =
        util::from_chars_expected<unsigned int>(harmony_bms_id_str.value());
    if (!harmony_bms_id_res || harmony_bms_id_res.value() > UINT8_MAX) {
        RCLCPP_ERROR(
            this->get_logger(), "Invalid Harmony BMS ID '%s'", harmony_bms_id_str.value().c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    this->model.emplace(
        std::make_shared<hardware_model::impl::LinuxCan>(), std::move(actuator_configs),
        static_cast<hardware_model::can::HarmonyBmsModel::Id>(harmony_bms_id_res.value()));

    auto res = this->model->on_init();
    if (!res) {
        RCLCPP_ERROR(this->get_logger(), "\n  Failed to initialize Can: %s", res.error().c_str());
        // CANの初期化に失敗した場合、モデルにnullを再代入する
        this->model.reset();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

auto Can::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    -> hardware_interface::CallbackReturn {
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    for (const auto & name : {
             "bms/voltages.pack",
             "bms/voltages.charger",
             "bms/currents.input",
             "bms/currents.measured",
             "bms/capacity_state.state_of_charge",
             "bms/capacity_state.state_of_health",
             "bms/cell_voltage_range.min",
             "bms/cell_voltage_range.max",
         }) {
        this->set_state(name, nan);
    }
    for (std::size_t i = 0; i < state::bms::CELL_COUNT; ++i) {
        this->set_state("bms/cell_" + std::to_string(i) + ".voltage", nan);
    }
    const auto nan_temperature = util::to_interface_data(state::bms::Temperature{nan});
    this->set_state("bms/balance_ic_temperature", nan_temperature);
    this->set_state("bms/mosfet_temperature", nan_temperature);
    this->set_state("bms/ambient_temperature", nan_temperature);
    for (std::size_t i = 0; i < state::bms::ADDITIONAL_TEMPERATURE_COUNT; ++i) {
        this->set_state("bms/additional_temperature_" + std::to_string(i), nan_temperature);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

auto Can::find_actuator_index(const std::string & actuator_name) const
    -> std::optional<std::size_t> {
    const auto it = std::find_if(
        this->actuators.begin(), this->actuators.end(),
        [&actuator_name](const auto & actuator) { return actuator.name == actuator_name; });
    if (it == this->actuators.end()) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(it - this->actuators.begin());
}

auto Can::set_vesc_health(std::size_t index, bool is_ok) -> void {
    const auto & actuator = this->actuators[index];
    // サーボ専用VESCにはモーター状態のインターフェースが無い
    if (actuator.motor_type == hardware_model::CanModel::MotorType::None) {
        return;
    }
    this->set_state(
        actuator.name + "/esc/health",
        util::to_interface_data(state::actuator::motor::Health{is_ok}));
}

auto Can::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hardware_interface::return_type {
    if (!this->model) {
        this->set_state("can/health", util::to_interface_data(state::can::Health{false}));
        this->set_state("bms/health", util::to_interface_data(state::bms::Health{false}));
        for (std::size_t i = 0; i < this->actuators.size(); ++i) {
            this->set_vesc_health(i, false);
        }

        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Can model is not initialized");
        return hardware_interface::return_type::OK;
    }

    auto res = this->model->on_read();
    if (!res) {
        this->set_state("can/health", util::to_interface_data(state::can::Health{false}));
        this->set_state("bms/health", util::to_interface_data(state::bms::Health{false}));
        for (std::size_t i = 0; i < this->actuators.size(); ++i) {
            this->set_vesc_health(i, false);
        }

        constexpr auto DURATION = 3000;  // ms
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Failed to read CAN data: %s",
            res.error().c_str());
        return hardware_interface::return_type::OK;
    }

    const auto & read_batch = res.value();
    auto vesc_updated = std::vector<bool>(this->actuators.size(), false);
    auto bms_updated = false;
    for (const auto & state : read_batch.states) {
        switch (state.index()) {
            case 0: {  // Rpm
                const auto & [actuator_name, rpm] = std::get<0>(state);
                this->set_state(actuator_name + "/esc/rpm", util::to_interface_data(rpm));
                if (const auto index = this->find_actuator_index(actuator_name)) {
                    vesc_updated[index.value()] = true;
                }
                break;
            }
            case 1: {  // ESC Voltage
                const auto & [actuator_name, voltage] = std::get<1>(state);
                this->set_state(actuator_name + "/esc/voltage", util::to_interface_data(voltage));
                if (const auto index = this->find_actuator_index(actuator_name)) {
                    vesc_updated[index.value()] = true;
                }
                break;
            }
            case 2: {  // ESC WaterLeaked
                const auto & [actuator_name, water_leaked] = std::get<2>(state);
                this->set_state(
                    actuator_name + "/esc/water_leaked", util::to_interface_data(water_leaked));
                if (const auto index = this->find_actuator_index(actuator_name)) {
                    vesc_updated[index.value()] = true;
                }
                break;
            }
            case 3: {  // VESC heartbeat
                const auto & [actuator_name, _health] = std::get<3>(state);
                if (const auto index = this->find_actuator_index(actuator_name)) {
                    vesc_updated[index.value()] = true;
                }
                break;
            }
            case 4: {  // Harmony BMS
                using HarmonyBmsModel = hardware_model::can::HarmonyBmsModel;
                const auto & packet = std::get<HarmonyBmsModel::AnyPacket>(state);
                switch (packet.index()) {
                    case 1: {  // PacketVoltage
                        const auto & voltage = std::get<HarmonyBmsModel::PacketVoltage>(packet);
                        this->set_state("bms/voltages.pack", voltage.pack);
                        this->set_state("bms/voltages.charger", voltage.charger);
                        break;
                    }
                    case 2: {  // PacketCurrent
                        const auto & current = std::get<HarmonyBmsModel::PacketCurrent>(packet);
                        this->set_state("bms/currents.input", current.input);
                        this->set_state("bms/currents.measured", current.measured);
                        break;
                    }
                    case 3:  // PacketCounters
                        // 累積値は使用しない
                        break;
                    case 4: {  // PacketCellVoltage
                        const auto & cell_voltage =
                            std::get<HarmonyBmsModel::PacketCellVoltage>(packet);
                        this->set_state(
                            "bms/cell_count", util::to_interface_data(
                                                  state::bms::CellCount{cell_voltage.cell_count}));
                        for (std::size_t i = 0; i < cell_voltage.value_count; ++i) {
                            const auto index = cell_voltage.offset + i;
                            if (index < state::bms::CELL_COUNT) {
                                this->set_state(
                                    "bms/cell_" + std::to_string(index) + ".voltage",
                                    cell_voltage.voltages[i]);
                            }
                        }
                        break;
                    }
                    case 5: {  // PacketBalancing
                        const auto & balancing = std::get<HarmonyBmsModel::PacketBalancing>(packet);
                        for (std::size_t i = 0; i < state::bms::CELL_COUNT; ++i) {
                            this->set_state(
                                "bms/cell_" + std::to_string(i) + ".is_balancing",
                                util::to_interface_data(balancing.balancing[i]));
                        }
                        break;
                    }
                    case 6: {  // PacketTemperatures
                        const auto & temperatures =
                            std::get<HarmonyBmsModel::PacketTemperatures>(packet);
                        for (std::size_t i = 0; i < temperatures.value_count; ++i) {
                            const auto index = temperatures.offset + i;
                            const auto temperature = util::to_interface_data(
                                state::bms::Temperature{temperatures.temperatures[i]});
                            if (index == HarmonyBmsModel::MOSFET_TEMPERATURE_INDEX) {
                                this->set_state("bms/mosfet_temperature", temperature);
                            } else if (index == HarmonyBmsModel::AMBIENT_TEMPERATURE_INDEX) {
                                this->set_state("bms/ambient_temperature", temperature);
                            } else if (
                                index >= HarmonyBmsModel::ADDITIONAL_TEMPERATURE_OFFSET &&
                                index < HarmonyBmsModel::ADDITIONAL_TEMPERATURE_OFFSET +
                                            state::bms::ADDITIONAL_TEMPERATURE_COUNT) {
                                this->set_state(
                                    "bms/additional_temperature_" +
                                        std::to_string(
                                            index - HarmonyBmsModel::ADDITIONAL_TEMPERATURE_OFFSET),
                                    temperature);
                            }
                        }
                        break;
                    }
                    case 7: {  // PacketHumidity
                        // 湿度センサーは互換基板に搭載されていないため、バランスICの温度のみ使う
                        const auto & humidity = std::get<HarmonyBmsModel::PacketHumidity>(packet);
                        this->set_state(
                            "bms/balance_ic_temperature",
                            util::to_interface_data(
                                state::bms::Temperature{humidity.balance_ic_temperature}));
                        break;
                    }
                    case 8: {  // PacketSummary
                        const auto & summary = std::get<HarmonyBmsModel::PacketSummary>(packet);
                        this->set_state("bms/cell_voltage_range.min", summary.cell_voltage_min);
                        this->set_state("bms/cell_voltage_range.max", summary.cell_voltage_max);
                        this->set_state(
                            "bms/capacity_state.state_of_charge", summary.state_of_charge);
                        this->set_state(
                            "bms/capacity_state.state_of_health", summary.state_of_health);
                        this->set_state(
                            "bms/charge_status.is_charging",
                            util::to_interface_data(summary.charging));
                        this->set_state(
                            "bms/charge_status.is_balancing",
                            util::to_interface_data(summary.balancing));
                        this->set_state(
                            "bms/charge_status.is_charge_allowed",
                            util::to_interface_data(summary.charge_allowed));
                        break;
                    }
                    case 9:   // PacketChargeTotals
                    case 10:  // PacketDischargeTotals
                        // 累積値は使用しない
                        break;
                    case 11: {  // PacketStatus
                        const auto & status = std::get<HarmonyBmsModel::PacketStatus>(packet);
                        this->set_state(
                            "bms/status", util::to_interface_data(state::bms::Status{
                                              status.faults, status.power_switch_state}));
                        constexpr auto DURATION = 3000;  // ms
                        if (util::has_bms_fault(status.faults)) {
                            RCLCPP_ERROR_THROTTLE(
                                this->get_logger(), *this->get_clock(), DURATION,
                                "\n  Harmony BMS fault: %s", status.text.c_str());
                        } else if (
                            status.power_switch_state == util::BmsPowerSwitchState::Unknown) {
                            RCLCPP_WARN_THROTTLE(
                                this->get_logger(), *this->get_clock(), DURATION,
                                "\n  Unknown Harmony BMS status: %s", status.text.c_str());
                        }
                        break;
                    }
                }
                bms_updated = true;
                break;
            }
        }
    }

    if (bms_updated) {
        this->cycles_since_bms_update = 0;
    } else if (this->cycles_since_bms_update < MAX_CYCLES_SINCE_NODE_UPDATE) {
        ++this->cycles_since_bms_update;
    }
    this->set_state(
        "bms/health", util::to_interface_data(state::bms::Health{
                          this->cycles_since_bms_update < MAX_CYCLES_SINCE_NODE_UPDATE}));

    for (std::size_t i = 0; i < this->actuators.size(); ++i) {
        if (vesc_updated[i]) {
            this->cycles_since_vesc_update[i] = 0;
        } else if (this->cycles_since_vesc_update[i] < MAX_CYCLES_SINCE_NODE_UPDATE) {
            ++this->cycles_since_vesc_update[i];
        }
        this->set_vesc_health(
            i, this->cycles_since_vesc_update[i] < MAX_CYCLES_SINCE_NODE_UPDATE);
    }

    if (!read_batch.states.empty()) {
        this->cycles_since_any_node_update = 0;
    }

    // この周期数だけ状態更新がなければCANを異常とみなす
    if (!read_batch.error_message.empty()) {
        this->set_state("can/health", util::to_interface_data(state::can::Health{false}));

        constexpr auto DURATION = 3000;  // ms
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Failed to read CAN data: %s",
            read_batch.error_message.c_str());
    } else if (read_batch.states.empty()) {
        if (this->cycles_since_any_node_update < MAX_CYCLES_SINCE_ANY_NODE_UPDATE) {
            ++this->cycles_since_any_node_update;
        }
        if (this->cycles_since_any_node_update >= MAX_CYCLES_SINCE_ANY_NODE_UPDATE) {
            this->set_state("can/health", util::to_interface_data(state::can::Health{false}));
        }
    } else {
        this->set_state("can/health", util::to_interface_data(state::can::Health{true}));
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

    auto && power_distribution_enabled =
        util::from_interface_data<cmd::power_distribution::Enabled>(
            this->get_command("power_distribution/enabled"));
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

    const auto res =
        this->model->on_write(power_distribution_enabled, actuator_commands, led_tape_color);
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
