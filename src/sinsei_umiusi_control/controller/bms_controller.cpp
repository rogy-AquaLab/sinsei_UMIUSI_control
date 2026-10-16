#include "sinsei_umiusi_control/controller/bms_controller.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>

#include "sinsei_umiusi_control/util/bms_status.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"
#include "sinsei_umiusi_msgs/msg/bms_faults.hpp"

using namespace sinsei_umiusi_control::controller;

namespace msg = sinsei_umiusi_msgs::msg;

auto BmsController::command_interface_configuration() const
    -> controller_interface::InterfaceConfiguration {
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::NONE, {}};
}

auto BmsController::state_interface_configuration() const
    -> controller_interface::InterfaceConfiguration {
    auto state_names = std::vector<std::string>{};
    for (const auto & [name, _data, _size] : this->state_interface_data) {
        state_names.push_back(name);
    }

    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::INDIVIDUAL, state_names};
}

auto BmsController::on_init() -> controller_interface::CallbackReturn {
    using rcl_interfaces::msg::ParameterDescriptor;

    this->get_node()->declare_parameter(
        "state_topic_prefix", this->state_topic_prefix,
        ParameterDescriptor{}
            .set__description("Prefix of BMS state topics")
            .set__type(rclcpp::PARAMETER_STRING)
            .set__read_only(true));

    return controller_interface::CallbackReturn::SUCCESS;
}

auto BmsController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    -> controller_interface::CallbackReturn {
    this->state_topic_prefix = this->get_node()->get_parameter("state_topic_prefix").as_string();

    this->state = State{};

    using util::to_interface_data_ptr;
    auto & data = this->state_interface_data;
    auto & state = this->state;
    data.clear();
    data.emplace_back("bms/health", to_interface_data_ptr(state.health), sizeof(state.health));
    data.emplace_back(
        "bms/voltages.pack", to_interface_data_ptr(state.voltages.pack),
        sizeof(state.voltages.pack));
    data.emplace_back(
        "bms/voltages.charger", to_interface_data_ptr(state.voltages.charger),
        sizeof(state.voltages.charger));
    data.emplace_back(
        "bms/currents.input", to_interface_data_ptr(state.currents.input),
        sizeof(state.currents.input));
    data.emplace_back(
        "bms/currents.measured", to_interface_data_ptr(state.currents.measured),
        sizeof(state.currents.measured));
    data.emplace_back(
        "bms/capacity_state.state_of_charge", to_interface_data_ptr(state.capacity.state_of_charge),
        sizeof(state.capacity.state_of_charge));
    data.emplace_back(
        "bms/capacity_state.state_of_health", to_interface_data_ptr(state.capacity.state_of_health),
        sizeof(state.capacity.state_of_health));
    data.emplace_back(
        "bms/cell_voltage_range.min", to_interface_data_ptr(state.cell_voltage_range.min),
        sizeof(state.cell_voltage_range.min));
    data.emplace_back(
        "bms/cell_voltage_range.max", to_interface_data_ptr(state.cell_voltage_range.max),
        sizeof(state.cell_voltage_range.max));
    data.emplace_back(
        "bms/charge_status.is_charging", to_interface_data_ptr(state.charge_status.is_charging),
        sizeof(state.charge_status.is_charging));
    data.emplace_back(
        "bms/charge_status.is_balancing", to_interface_data_ptr(state.charge_status.is_balancing),
        sizeof(state.charge_status.is_balancing));
    data.emplace_back(
        "bms/charge_status.is_charge_allowed",
        to_interface_data_ptr(state.charge_status.is_charge_allowed),
        sizeof(state.charge_status.is_charge_allowed));
    data.emplace_back("bms/status", to_interface_data_ptr(state.status), sizeof(state.status));
    data.emplace_back(
        "bms/cell_count", to_interface_data_ptr(state.cell_count), sizeof(state.cell_count));
    for (std::size_t i = 0; i < state.cells.size(); ++i) {
        const auto prefix = "bms/cell_" + std::to_string(i);
        data.emplace_back(
            prefix + ".voltage", to_interface_data_ptr(state.cells[i].voltage),
            sizeof(state.cells[i].voltage));
        data.emplace_back(
            prefix + ".is_balancing", to_interface_data_ptr(state.cells[i].is_balancing),
            sizeof(state.cells[i].is_balancing));
    }
    data.emplace_back(
        "bms/balance_ic_temperature", to_interface_data_ptr(state.balance_ic_temperature),
        sizeof(state.balance_ic_temperature));
    data.emplace_back(
        "bms/mosfet_temperature", to_interface_data_ptr(state.mosfet_temperature),
        sizeof(state.mosfet_temperature));
    data.emplace_back(
        "bms/ambient_temperature", to_interface_data_ptr(state.ambient_temperature),
        sizeof(state.ambient_temperature));
    for (std::size_t i = 0; i < state.additional_temperatures.size(); ++i) {
        data.emplace_back(
            "bms/additional_temperature_" + std::to_string(i),
            to_interface_data_ptr(state.additional_temperatures[i]),
            sizeof(state.additional_temperatures[i]));
    }

    // UMIUSIのGateControllerと同じtopicに出す
    const auto qos = rclcpp::SystemDefaultsQoS();
    this->battery_state_publisher =
        this->get_node()->create_publisher<sensor_msgs::msg::BatteryState>(
            this->state_topic_prefix + "power/battery", qos);
    this->bms_state_publisher = this->get_node()->create_publisher<msg::BmsState>(
        this->state_topic_prefix + "power/bms", qos);

    return controller_interface::CallbackReturn::SUCCESS;
}

// 変換はGateController::updateと揃えている
auto BmsController::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
    -> controller_interface::return_type {
    if (!util::interface_accessor::get_states_from_loaned_interfaces(
            this->state_interfaces_, this->state_interface_data)) {
        RCLCPP_WARN_THROTTLE(
            this->get_node()->get_logger(), *this->get_node()->get_clock(), 3000,
            "Failed to get BMS state interfaces");
    }
    const auto & state = this->state;

    const auto cell_count = std::min<std::size_t>(state.cell_count.value, state.cells.size());
    auto cell_voltages = std::vector<float>{};
    auto cell_balancing = std::vector<bool>{};
    cell_voltages.reserve(cell_count);
    cell_balancing.reserve(cell_count);
    for (std::size_t i = 0; i < cell_count; ++i) {
        cell_voltages.push_back(static_cast<float>(state.cells[i].voltage));
        cell_balancing.push_back(state.cells[i].is_balancing);
    }
    auto additional_temperatures = std::array<float, state::bms::ADDITIONAL_TEMPERATURE_COUNT>{};
    for (std::size_t i = 0; i < additional_temperatures.size(); ++i) {
        additional_temperatures[i] = static_cast<float>(state.additional_temperatures[i].value);
    }

    this->battery_state_publisher->publish(
        sensor_msgs::msg::BatteryState()
            .set__header(std_msgs::msg::Header().set__stamp(time).set__frame_id("harmony_bms"))
            .set__voltage(static_cast<float>(state.voltages.pack))
            // VESC BMSは放電時を正、BatteryStateは充電時を正とするため、符号を反転する
            .set__current(static_cast<float>(-state.currents.measured))
            .set__temperature(std::numeric_limits<float>::quiet_NaN())
            .set__charge(std::numeric_limits<float>::quiet_NaN())
            .set__capacity(std::numeric_limits<float>::quiet_NaN())
            .set__design_capacity(std::numeric_limits<float>::quiet_NaN())
            .set__percentage(static_cast<float>(state.capacity.state_of_charge))
            .set__power_supply_status(
                state.charge_status.is_charging
                    ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING
                    : (state.currents.measured > 0.0
                           ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING
                           : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING))
            .set__power_supply_health(
                state.health.is_ok && !util::has_bms_fault(state.status.faults)
                    ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD
                    : sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
            .set__power_supply_technology(
                sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO)
            .set__present(state.health.is_ok)
            .set__cell_voltage(cell_voltages));
    this->bms_state_publisher->publish(
        msg::BmsState()
            .set__header(std_msgs::msg::Header().set__stamp(time).set__frame_id("harmony_bms"))
            .set__state_of_health(static_cast<float>(state.capacity.state_of_health))
            .set__cell_voltage_min(static_cast<float>(state.cell_voltage_range.min))
            .set__cell_voltage_max(static_cast<float>(state.cell_voltage_range.max))
            .set__balancing(state.charge_status.is_balancing)
            .set__charge_allowed(state.charge_status.is_charge_allowed)
            .set__cell_balancing(cell_balancing)
            .set__charger_voltage(static_cast<float>(state.voltages.charger))
            .set__balance_ic_temperature(static_cast<float>(state.balance_ic_temperature.value))
            .set__mosfet_temperature(static_cast<float>(state.mosfet_temperature.value))
            .set__ambient_temperature(static_cast<float>(state.ambient_temperature.value))
            .set__additional_temperatures(additional_temperatures)
            // `BmsPowerSwitchState`の値は`BmsState`の`POWER_SWITCH_*`と揃えている
            .set__power_switch_state(static_cast<uint8_t>(state.status.power_switch_state))
            .set__faults(
                msg::BmsFaults()
                    .set__precharge(state.status.faults.precharge)
                    .set__short_circuit(state.status.faults.short_circuit)
                    .set__switch_over_temperature(state.status.faults.switch_over_temperature)
                    .set__charge_overcurrent(state.status.faults.charge_overcurrent)));

    return controller_interface::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::BmsController, controller_interface::ControllerInterface)
