#include "sinsei_umiusi_control/controller/gate_controller.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <rclcpp/logging.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>

#include "sinsei_umiusi_control/util/interface_accessor.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"
#include "sinsei_umiusi_msgs/msg/thruster_mode.hpp"
#include "sinsei_umiusi_msgs/msg/thruster_runnable_all.hpp"

using namespace sinsei_umiusi_control::controller;

namespace msg = sinsei_umiusi_msgs::msg;

auto GateController::command_interface_configuration() const
    -> controller_interface::InterfaceConfiguration {
    auto cmd_names = std::vector<std::string>{};
    for (const auto & [name, _data, _size] : this->command_interface_data) {
        cmd_names.push_back(name);
    }

    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::INDIVIDUAL,
        cmd_names,
    };
}

auto GateController::state_interface_configuration() const
    -> controller_interface::InterfaceConfiguration {
    auto state_names = std::vector<std::string>{};
    for (const auto & [name, _data, _size] : this->state_interface_data) {
        state_names.push_back(name);
    }

    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::INDIVIDUAL,
        state_names,
    };
}

auto GateController::on_init() -> controller_interface::CallbackReturn {
    this->output.cmd = GateController::Output::Command{};
    this->input.state = GateController::Input::State{};

    return controller_interface::CallbackReturn::SUCCESS;
}

auto GateController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    -> controller_interface::CallbackReturn {
    constexpr std::string_view THRUSTER_SUFFIX[4] = {"_lf", "_lb", "_rb", "_rf"};

    {  // Input
        // State interface (in)
        using util::to_interface_data_ptr;

        const auto add_bms_state = [this](const std::string & name, auto & value) {
            this->state_interface_data.emplace_back(
                "bms/" + name, util::to_interface_data_ptr(value), sizeof(value));
        };
        add_bms_state("health", this->input.state.bms_health);
        add_bms_state("pack_voltage", this->input.state.bms_pack_voltage);
        add_bms_state("charger_voltage", this->input.state.bms_charger_voltage);
        add_bms_state("input_current", this->input.state.bms_input_current);
        add_bms_state("measured_current", this->input.state.bms_measured_current);
        add_bms_state("state_of_charge", this->input.state.bms_state_of_charge);
        add_bms_state("state_of_health", this->input.state.bms_state_of_health);
        add_bms_state("cell_voltage_min", this->input.state.bms_cell_voltage_min);
        add_bms_state("cell_voltage_max", this->input.state.bms_cell_voltage_max);
        add_bms_state("cell_temperature_max", this->input.state.bms_cell_temperature_max);
        add_bms_state("charging", this->input.state.bms_charging);
        add_bms_state("balancing", this->input.state.bms_balancing);
        add_bms_state("charge_allowed", this->input.state.bms_charge_allowed);
        add_bms_state("cell_count", this->input.state.bms_cell_count);
        for (std::size_t i = 0; i < this->input.state.bms_cell_voltages.size(); ++i) {
            add_bms_state(
                "cell_voltage_" + std::to_string(i), this->input.state.bms_cell_voltages[i]);
            add_bms_state(
                "cell_balancing_" + std::to_string(i), this->input.state.bms_cell_balancing[i]);
        }
        add_bms_state("temperature_count", this->input.state.bms_temperature_count);
        for (std::size_t i = 0; i < this->input.state.bms_temperatures.size(); ++i) {
            add_bms_state(
                "temperature_" + std::to_string(i), this->input.state.bms_temperatures[i]);
        }
        add_bms_state(
            "humidity_sensor_temperature", this->input.state.bms_humidity_sensor_temperature);
        add_bms_state("relative_humidity", this->input.state.bms_relative_humidity);
        add_bms_state("balance_ic_temperature", this->input.state.bms_balance_ic_temperature);
        add_bms_state("net_consumed_charge", this->input.state.bms_net_consumed_charge);
        add_bms_state("net_consumed_energy", this->input.state.bms_net_consumed_energy);
        add_bms_state("total_charged_charge", this->input.state.bms_total_charged_charge);
        add_bms_state("total_charged_energy", this->input.state.bms_total_charged_energy);
        add_bms_state("total_discharged_charge", this->input.state.bms_total_discharged_charge);
        add_bms_state("total_discharged_energy", this->input.state.bms_total_discharged_energy);
        add_bms_state("power_switch_state", this->input.state.bms_power_switch_state);
        add_bms_state("fault_flags", this->input.state.bms_fault_flags);
        add_bms_state("data_version", this->input.state.bms_data_version);
        for (std::size_t i = 0; i < this->input.state.bms_status_chunks.size(); ++i) {
            add_bms_state(
                "status_chunk_" + std::to_string(i), this->input.state.bms_status_chunks[i]);
        }
        this->state_interface_data.emplace_back(
            "imu/temperature", to_interface_data_ptr(this->input.state.imu_temperature),
            sizeof(this->input.state.imu_temperature));
        this->state_interface_data.emplace_back(
            "attitude_controller/imu/quaternion.x",
            to_interface_data_ptr(this->input.state.imu_quaternion.x),
            sizeof(this->input.state.imu_quaternion.x));
        this->state_interface_data.emplace_back(
            "attitude_controller/imu/quaternion.y",
            to_interface_data_ptr(this->input.state.imu_quaternion.y),
            sizeof(this->input.state.imu_quaternion.y));
        this->state_interface_data.emplace_back(
            "attitude_controller/imu/quaternion.z",
            to_interface_data_ptr(this->input.state.imu_quaternion.z),
            sizeof(this->input.state.imu_quaternion.z));
        this->state_interface_data.emplace_back(
            "attitude_controller/imu/quaternion.w",
            to_interface_data_ptr(this->input.state.imu_quaternion.w),
            sizeof(this->input.state.imu_quaternion.w));
        this->state_interface_data.emplace_back(
            "attitude_controller/imu/acceleration.x",
            to_interface_data_ptr(this->input.state.imu_acceleration.x),
            sizeof(this->input.state.imu_acceleration.x));
        this->state_interface_data.emplace_back(
            "attitude_controller/imu/acceleration.y",
            to_interface_data_ptr(this->input.state.imu_acceleration.y),
            sizeof(this->input.state.imu_acceleration.y));
        this->state_interface_data.emplace_back(
            "attitude_controller/imu/acceleration.z",
            to_interface_data_ptr(this->input.state.imu_acceleration.z),
            sizeof(this->input.state.imu_acceleration.z));
        this->state_interface_data.emplace_back(
            "attitude_controller/imu/angular_velocity.x",
            to_interface_data_ptr(this->input.state.imu_angular_velocity.x),
            sizeof(this->input.state.imu_angular_velocity.x));
        this->state_interface_data.emplace_back(
            "attitude_controller/imu/angular_velocity.y",
            to_interface_data_ptr(this->input.state.imu_angular_velocity.y),
            sizeof(this->input.state.imu_angular_velocity.y));
        this->state_interface_data.emplace_back(
            "attitude_controller/imu/angular_velocity.z",
            to_interface_data_ptr(this->input.state.imu_angular_velocity.z),
            sizeof(this->input.state.imu_angular_velocity.z));
        this->state_interface_data.emplace_back(
            "can/health", to_interface_data_ptr(this->input.state.can_health),
            sizeof(this->input.state.can_health));
        this->state_interface_data.emplace_back(
            "headlights/health", to_interface_data_ptr(this->input.state.headlights_health),
            sizeof(this->input.state.headlights_health));
        this->state_interface_data.emplace_back(
            "imu/health", to_interface_data_ptr(this->input.state.imu_health),
            sizeof(this->input.state.imu_health));
        this->state_interface_data.emplace_back(
            "indicator_led/health", to_interface_data_ptr(this->input.state.indicator_led_health),
            sizeof(this->input.state.indicator_led_health));
        for (size_t i = 0; i < 4; ++i) {
            const auto tc_prefix = "thruster_controller" + std::string(THRUSTER_SUFFIX[i]) + "/";

            this->state_interface_data.emplace_back(
                tc_prefix + "esc/mode", to_interface_data_ptr(this->input.state.esc_modes[i]),
                sizeof(this->input.state.esc_modes[i]));
            this->state_interface_data.emplace_back(
                tc_prefix + "esc/duty_cycle",
                to_interface_data_ptr(this->input.state.esc_duty_cycles[i]),
                sizeof(this->input.state.esc_duty_cycles[i]));
            this->state_interface_data.emplace_back(
                tc_prefix + "servo/mode", to_interface_data_ptr(this->input.state.servo_modes[i]),
                sizeof(this->input.state.servo_modes[i]));
            this->state_interface_data.emplace_back(
                tc_prefix + "servo/angle", to_interface_data_ptr(this->input.state.servo_angles[i]),
                sizeof(this->input.state.servo_angles[i]));

            this->state_interface_data.emplace_back(
                tc_prefix + "thruster/esc/voltage",
                to_interface_data_ptr(this->input.state.esc_voltages[i]),
                sizeof(this->input.state.esc_voltages[i]));
            this->state_interface_data.emplace_back(
                tc_prefix + "thruster/esc/water_leaked",
                to_interface_data_ptr(this->input.state.esc_water_leaked_flags[i]),
                sizeof(this->input.state.esc_water_leaked_flags[i]));
            this->state_interface_data.emplace_back(
                tc_prefix + "thruster/esc/health",
                to_interface_data_ptr(this->input.state.esc_health[i]),
                sizeof(this->input.state.esc_health[i]));

            // RPMのみ`attitude_controller`経由で取得する
            const auto ac_prefix = "attitude_controller/thruster_controller" +
                                   std::string(THRUSTER_SUFFIX[i]) + "/thruster/";
            this->state_interface_data.emplace_back(
                ac_prefix + "esc/rpm", to_interface_data_ptr(this->input.state.esc_rpms[i]),
                sizeof(this->input.state.esc_rpms[i]));
        }

        // Subscribers
        const auto cmd_prefix = std::string("cmd/");
        const auto qos = rclcpp::SystemDefaultsQoS();
        this->input.sub.indicator_led_output_subscriber =
            this->get_node()->create_subscription<msg::IndicatorLedOutput>(
                cmd_prefix + "indicator_led_output", qos,
                [this](const msg::IndicatorLedOutput::SharedPtr input) {
                    this->output.cmd.indicator_led_enabled_ref.value = input->enabled;
                });
        this->input.sub.power_distribution_output_subscriber =
            this->get_node()->create_subscription<msg::PowerDistributionOutput>(
                cmd_prefix + "power_distribution_output", qos,
                [this](const msg::PowerDistributionOutput::SharedPtr input) {
                    this->output.cmd.power_distribution_enabled_ref.value = input->enabled;
                });
        this->input.sub.led_tape_output_subscriber =
            this->get_node()->create_subscription<msg::LedTapeOutput>(
                cmd_prefix + "led_tape_output", qos,
                [this](const msg::LedTapeOutput::SharedPtr input) {
                    // alphaは無視
                    this->output.cmd.led_tape_color_ref.red =
                        static_cast<uint8_t>(std::clamp(input->color.r, 0.0f, 1.0f) * 255.0);
                    this->output.cmd.led_tape_color_ref.green =
                        static_cast<uint8_t>(std::clamp(input->color.g, 0.0f, 1.0f) * 255.0);
                    this->output.cmd.led_tape_color_ref.blue =
                        static_cast<uint8_t>(std::clamp(input->color.b, 0.0f, 1.0f) * 255.0);
                });
        this->input.sub.headlights_output_subscriber =
            this->get_node()->create_subscription<msg::HeadlightsOutput>(
                cmd_prefix + "headlights_output", qos,
                [this](const msg::HeadlightsOutput::SharedPtr input) {
                    this->output.cmd.high_beam_enabled_ref.value = input->high_beam_enabled;
                    this->output.cmd.low_beam_enabled_ref.value = input->low_beam_enabled;
                    this->output.cmd.ir_enabled_ref.value = input->ir_enabled;
                });
        this->input.sub.thruster_runnable_all_subscriber =
            this->get_node()->create_subscription<msg::ThrusterRunnableAll>(
                cmd_prefix + "thruster_runnable_all", qos,
                [this](const msg::ThrusterRunnableAll::SharedPtr input) {
                    this->output.cmd.esc_runnable_refs[0].value = input->lf.esc;
                    this->output.cmd.esc_runnable_refs[1].value = input->lb.esc;
                    this->output.cmd.esc_runnable_refs[2].value = input->rb.esc;
                    this->output.cmd.esc_runnable_refs[3].value = input->rf.esc;
                    this->output.cmd.servo_runnable_refs[0].value = input->lf.servo;
                    this->output.cmd.servo_runnable_refs[1].value = input->lb.servo;
                    this->output.cmd.servo_runnable_refs[2].value = input->rb.servo;
                    this->output.cmd.servo_runnable_refs[3].value = input->rf.servo;
                });
        this->input.sub.target_subscriber = this->get_node()->create_subscription<msg::Target>(
            cmd_prefix + "target", qos, [this](const msg::Target::SharedPtr input) {
                this->output.cmd.target_orientation_ref.x = input->orientation.x;
                this->output.cmd.target_orientation_ref.y = input->orientation.y;
                this->output.cmd.target_orientation_ref.z = input->orientation.z;
                this->output.cmd.target_velocity_ref.x = input->velocity.x;
                this->output.cmd.target_velocity_ref.y = input->velocity.y;
                this->output.cmd.target_velocity_ref.z = input->velocity.z;
            });
    }
    {  // Output
        // Command interface (out)
        using util::to_interface_data_ptr;

        this->command_interface_data.push_back(std::make_tuple(
            "indicator_led/enabled",
            to_interface_data_ptr(this->output.cmd.indicator_led_enabled_ref),
            sizeof(this->output.cmd.indicator_led_enabled_ref)));
        this->command_interface_data.push_back(std::make_tuple(
            "power_distribution/enabled",
            to_interface_data_ptr(this->output.cmd.power_distribution_enabled_ref),
            sizeof(this->output.cmd.power_distribution_enabled_ref)));
        this->command_interface_data.push_back(std::make_tuple(
            "headlights/high_beam_enabled",
            to_interface_data_ptr(this->output.cmd.high_beam_enabled_ref),
            sizeof(this->output.cmd.high_beam_enabled_ref)));
        this->command_interface_data.push_back(std::make_tuple(
            "headlights/low_beam_enabled",
            to_interface_data_ptr(this->output.cmd.low_beam_enabled_ref),
            sizeof(this->output.cmd.low_beam_enabled_ref)));
        this->command_interface_data.push_back(std::make_tuple(
            "headlights/ir_enabled", to_interface_data_ptr(this->output.cmd.ir_enabled_ref),
            sizeof(this->output.cmd.ir_enabled_ref)));
        this->command_interface_data.push_back(std::make_tuple(
            "led_tape/color", to_interface_data_ptr(this->output.cmd.led_tape_color_ref),
            sizeof(this->output.cmd.led_tape_color_ref)));
        this->command_interface_data.push_back(std::make_tuple(
            "attitude_controller/target_orientation.x",
            to_interface_data_ptr(this->output.cmd.target_orientation_ref.x),
            sizeof(this->output.cmd.target_orientation_ref.x)));
        this->command_interface_data.push_back(std::make_tuple(
            "attitude_controller/target_orientation.y",
            to_interface_data_ptr(this->output.cmd.target_orientation_ref.y),
            sizeof(this->output.cmd.target_orientation_ref.y)));
        this->command_interface_data.push_back(std::make_tuple(
            "attitude_controller/target_orientation.z",
            to_interface_data_ptr(this->output.cmd.target_orientation_ref.z),
            sizeof(this->output.cmd.target_orientation_ref.z)));
        this->command_interface_data.push_back(std::make_tuple(
            "attitude_controller/target_velocity.x",
            to_interface_data_ptr(this->output.cmd.target_velocity_ref.x),
            sizeof(this->output.cmd.target_velocity_ref.x)));
        this->command_interface_data.push_back(std::make_tuple(
            "attitude_controller/target_velocity.y",
            to_interface_data_ptr(this->output.cmd.target_velocity_ref.y),
            sizeof(this->output.cmd.target_velocity_ref.y)));
        this->command_interface_data.push_back(std::make_tuple(
            "attitude_controller/target_velocity.z",
            to_interface_data_ptr(this->output.cmd.target_velocity_ref.z),
            sizeof(this->output.cmd.target_velocity_ref.z)));
        for (size_t i = 0; i < 4; ++i) {
            const auto prefix = "thruster_controller" + std::string(THRUSTER_SUFFIX[i]) + "/";

            this->command_interface_data.push_back(std::make_tuple(
                prefix + "esc/runnable",
                to_interface_data_ptr(this->output.cmd.esc_runnable_refs[i]),
                sizeof(this->output.cmd.esc_runnable_refs[i])));
            this->command_interface_data.push_back(std::make_tuple(
                prefix + "servo/runnable",
                to_interface_data_ptr(this->output.cmd.servo_runnable_refs[i]),
                sizeof(this->output.cmd.servo_runnable_refs[i])));
        }

        // Publishers
        const auto state_prefix = std::string("state/");
        const auto qos = rclcpp::SystemDefaultsQoS();
        this->output.pub.imu_publisher =
            this->get_node()->create_publisher<sensor_msgs::msg::Imu>(state_prefix + "imu", qos);
        this->output.pub.imu_temperature_publisher =
            this->get_node()->create_publisher<sensor_msgs::msg::Temperature>(
                state_prefix + "imu_temperature", qos);
        this->output.pub.power_distribution_enabled_publisher =
            this->get_node()->create_publisher<msg::PowerDistributionEnabled>(
                state_prefix + "power_distribution_enabled", qos);
        this->output.pub.battery_state_publisher =
            this->get_node()->create_publisher<sensor_msgs::msg::BatteryState>(
                state_prefix + "power/battery", qos);
        this->output.pub.bms_state_publisher =
            this->get_node()->create_publisher<msg::BmsState>(state_prefix + "power/bms", qos);
        this->output.pub.thruster_state_all_publisher =
            this->get_node()->create_publisher<msg::ThrusterStateAll>(
                state_prefix + "thruster_state_all", qos);
        this->output.pub.low_power_circuit_info_publisher =
            this->get_node()->create_publisher<msg::LowPowerCircuitInfo>(
                state_prefix + "low_power_circuit_info", qos);
        this->output.pub.high_power_circuit_info_publisher =
            this->get_node()->create_publisher<msg::HighPowerCircuitInfo>(
                state_prefix + "high_power_circuit_info", qos);
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

auto GateController::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
    -> controller_interface::return_type {
    // 状態を取得
    util::interface_accessor::get_states_from_loaned_interfaces(
        this->state_interfaces_, this->state_interface_data);

    this->output.pub.imu_publisher->publish(
        sensor_msgs::msg::Imu()
            .set__header(std_msgs::msg::Header().set__stamp(time).set__frame_id("imu"))
            .set__orientation(geometry_msgs::msg::Quaternion()
                                  .set__x(this->input.state.imu_quaternion.x)
                                  .set__y(this->input.state.imu_quaternion.y)
                                  .set__z(this->input.state.imu_quaternion.z)
                                  .set__w(this->input.state.imu_quaternion.w))
            .set__linear_acceleration(geometry_msgs::msg::Vector3()
                                          .set__x(this->input.state.imu_acceleration.x)
                                          .set__y(this->input.state.imu_acceleration.y)
                                          .set__z(this->input.state.imu_acceleration.z))
            .set__angular_velocity(geometry_msgs::msg::Vector3()
                                       .set__x(this->input.state.imu_angular_velocity.x)
                                       .set__y(this->input.state.imu_angular_velocity.y)
                                       .set__z(this->input.state.imu_angular_velocity.z)));
    this->output.pub.imu_temperature_publisher->publish(
        sensor_msgs::msg::Temperature()
            .set__header(std_msgs::msg::Header().set__stamp(time).set__frame_id("imu"))
            .set__temperature(this->input.state.imu_temperature.value));
    this->output.pub.power_distribution_enabled_publisher->publish(
        msg::PowerDistributionEnabled().set__enabled(
            this->output.cmd.power_distribution_enabled_ref.value));

    auto battery_state = sensor_msgs::msg::BatteryState{};
    battery_state.header = std_msgs::msg::Header().set__stamp(time).set__frame_id("harmony_bms");
    battery_state.voltage = static_cast<float>(this->input.state.bms_pack_voltage.value);
    // VESC BMS reports positive current while discharging; BatteryState uses the opposite sign.
    battery_state.current = static_cast<float>(-this->input.state.bms_measured_current.value);
    battery_state.temperature =
        static_cast<float>(this->input.state.bms_cell_temperature_max.value);
    battery_state.charge = std::numeric_limits<float>::quiet_NaN();
    battery_state.capacity = std::numeric_limits<float>::quiet_NaN();
    battery_state.design_capacity = std::numeric_limits<float>::quiet_NaN();
    battery_state.percentage = static_cast<float>(this->input.state.bms_state_of_charge.value);
    battery_state.power_supply_status =
        this->input.state.bms_charging.value
            ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING
            : (this->input.state.bms_measured_current.value > 0.0
                   ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING
                   : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING);
    battery_state.power_supply_health =
        this->input.state.bms_health.value && this->input.state.bms_fault_flags.value == 0
            ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD
            : sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
    battery_state.power_supply_technology =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
    battery_state.present = this->input.state.bms_health.value;
    const auto cell_count = std::min<std::size_t>(
        this->input.state.bms_cell_count.value, this->input.state.bms_cell_voltages.size());
    battery_state.cell_voltage.reserve(cell_count);
    for (std::size_t i = 0; i < cell_count; ++i) {
        battery_state.cell_voltage.push_back(
            static_cast<float>(this->input.state.bms_cell_voltages[i].value));
    }
    this->output.pub.battery_state_publisher->publish(battery_state);

    auto status_bytes = std::array<char, 40>{};
    for (std::size_t chunk = 0; chunk < this->input.state.bms_status_chunks.size(); ++chunk) {
        std::copy_n(
            this->input.state.bms_status_chunks[chunk].value.begin(), 8,
            status_bytes.begin() + chunk * 8);
    }
    const auto status_end = std::find(status_bytes.begin(), status_bytes.end(), '\0');
    const auto status_text = std::string(status_bytes.begin(), status_end);

    auto bms_state = msg::BmsState{};
    bms_state.header = std_msgs::msg::Header().set__stamp(time).set__frame_id("harmony_bms");
    bms_state.state_of_health = static_cast<float>(this->input.state.bms_state_of_health.value);
    bms_state.cell_voltage_min = static_cast<float>(this->input.state.bms_cell_voltage_min.value);
    bms_state.cell_voltage_max = static_cast<float>(this->input.state.bms_cell_voltage_max.value);
    bms_state.cell_temperature_max =
        static_cast<float>(this->input.state.bms_cell_temperature_max.value);
    bms_state.charging = this->input.state.bms_charging.value;
    bms_state.balancing = this->input.state.bms_balancing.value;
    bms_state.charge_allowed = this->input.state.bms_charge_allowed.value;
    bms_state.cell_balancing.reserve(cell_count);
    for (std::size_t i = 0; i < cell_count; ++i) {
        bms_state.cell_balancing.push_back(this->input.state.bms_cell_balancing[i].value);
    }
    bms_state.charger_voltage = static_cast<float>(this->input.state.bms_charger_voltage.value);

    const auto temp_count = std::min<std::size_t>(
        this->input.state.bms_temperature_count.value, this->input.state.bms_temperatures.size());
    const auto temperature_or_nan = [this, temp_count](std::size_t index) {
        return index < temp_count
                   ? static_cast<float>(this->input.state.bms_temperatures[index].value)
                   : std::numeric_limits<float>::quiet_NaN();
    };
    bms_state.balance_ic_temperature =
        static_cast<float>(this->input.state.bms_balance_ic_temperature.value);
    bms_state.cell_temperature_min = temperature_or_nan(1);
    bms_state.mosfet_temperature = temperature_or_nan(3);
    bms_state.ambient_temperature = temperature_or_nan(4);
    for (std::size_t i = 5; i < temp_count; ++i) {
        bms_state.additional_temperatures.push_back(
            static_cast<float>(this->input.state.bms_temperatures[i].value));
    }
    bms_state.humidity_sensor_temperature =
        static_cast<float>(this->input.state.bms_humidity_sensor_temperature.value);
    bms_state.relative_humidity = static_cast<float>(this->input.state.bms_relative_humidity.value);
    bms_state.net_consumed_charge =
        static_cast<float>(this->input.state.bms_net_consumed_charge.value);
    bms_state.net_consumed_energy =
        static_cast<float>(this->input.state.bms_net_consumed_energy.value);
    bms_state.total_charged_charge =
        static_cast<float>(this->input.state.bms_total_charged_charge.value);
    bms_state.total_charged_energy =
        static_cast<float>(this->input.state.bms_total_charged_energy.value);
    bms_state.total_discharged_charge =
        static_cast<float>(this->input.state.bms_total_discharged_charge.value);
    bms_state.total_discharged_energy =
        static_cast<float>(this->input.state.bms_total_discharged_energy.value);
    bms_state.power_switch_state = this->input.state.bms_power_switch_state.value;
    bms_state.fault_flags = this->input.state.bms_fault_flags.value;
    bms_state.data_version = this->input.state.bms_data_version.value;
    bms_state.status_text = status_text;
    this->output.pub.bms_state_publisher->publish(bms_state);

    this->output.pub.thruster_state_all_publisher->publish(
        msg::ThrusterStateAll()
            .set__lf(msg::ThrusterState()
                         .set__mode(msg::ThrusterMode()
                                        .set__esc(static_cast<int8_t>(
                                            this->input.state.esc_modes[0].value))
                                        .set__servo(static_cast<int8_t>(
                                            this->input.state.servo_modes[0].value)))
                         .set__duty_cycle(this->input.state.esc_duty_cycles[0].value)
                         .set__angle(this->input.state.servo_angles[0].value)
                         .set__rpm(this->input.state.esc_rpms[0].value)
                         .set__input_voltage(this->input.state.esc_voltages[0].value)
                         .set__water_leaked(this->input.state.esc_water_leaked_flags[0].value))
            .set__lb(msg::ThrusterState()
                         .set__mode(msg::ThrusterMode()
                                        .set__esc(static_cast<int8_t>(
                                            this->input.state.esc_modes[1].value))
                                        .set__servo(static_cast<int8_t>(
                                            this->input.state.servo_modes[1].value)))
                         .set__duty_cycle(this->input.state.esc_duty_cycles[1].value)
                         .set__angle(this->input.state.servo_angles[1].value)
                         .set__rpm(this->input.state.esc_rpms[1].value)
                         .set__input_voltage(this->input.state.esc_voltages[1].value)
                         .set__water_leaked(this->input.state.esc_water_leaked_flags[1].value))
            .set__rb(msg::ThrusterState()
                         .set__mode(msg::ThrusterMode()
                                        .set__esc(static_cast<int8_t>(
                                            this->input.state.esc_modes[2].value))
                                        .set__servo(static_cast<int8_t>(
                                            this->input.state.servo_modes[2].value)))
                         .set__duty_cycle(this->input.state.esc_duty_cycles[2].value)
                         .set__angle(this->input.state.servo_angles[2].value)
                         .set__rpm(this->input.state.esc_rpms[2].value)
                         .set__input_voltage(this->input.state.esc_voltages[2].value)
                         .set__water_leaked(this->input.state.esc_water_leaked_flags[2].value))
            .set__rf(msg::ThrusterState()
                         .set__mode(msg::ThrusterMode()
                                        .set__esc(static_cast<int8_t>(
                                            this->input.state.esc_modes[3].value))
                                        .set__servo(static_cast<int8_t>(
                                            this->input.state.servo_modes[3].value)))
                         .set__duty_cycle(this->input.state.esc_duty_cycles[3].value)
                         .set__angle(this->input.state.servo_angles[3].value)
                         .set__rpm(this->input.state.esc_rpms[3].value)
                         .set__input_voltage(this->input.state.esc_voltages[3].value)
                         .set__water_leaked(this->input.state.esc_water_leaked_flags[3].value)));
    this->output.pub.low_power_circuit_info_publisher->publish(
        msg::LowPowerCircuitInfo()
            .set__can(
                this->input.state.can_health.is_ok ? msg::LowPowerCircuitInfo::OK
                                                   : msg::LowPowerCircuitInfo::ERROR)
            .set__headlights(
                this->input.state.headlights_health.is_ok ? msg::LowPowerCircuitInfo::OK
                                                          : msg::LowPowerCircuitInfo::ERROR)
            .set__imu(
                this->input.state.imu_health.is_ok ? msg::LowPowerCircuitInfo::OK
                                                   : msg::LowPowerCircuitInfo::ERROR)
            .set__indicator_led(
                this->input.state.indicator_led_health.is_ok ? msg::LowPowerCircuitInfo::OK
                                                             : msg::LowPowerCircuitInfo::ERROR));
    this->output.pub.high_power_circuit_info_publisher->publish(
        msg::HighPowerCircuitInfo()
            .set__bms(
                this->input.state.bms_health.value ? msg::HighPowerCircuitInfo::OK
                                                   : msg::HighPowerCircuitInfo::ERROR)
            .set__battery(
                this->input.state.bms_health.value &&
                        this->input.state.bms_fault_flags.value == 0 &&
                        this->input.state.bms_pack_voltage.value > 0.0
                    ? msg::HighPowerCircuitInfo::OK
                    : msg::HighPowerCircuitInfo::ERROR)
            .set__esc_lf(
                this->input.state.esc_health[0].is_ok &&
                        this->input.state.esc_voltages[0].value > 0.0 &&
                        !this->input.state.esc_water_leaked_flags[0].value
                    ? msg::HighPowerCircuitInfo::OK
                    : msg::HighPowerCircuitInfo::ERROR)
            .set__esc_lb(
                this->input.state.esc_health[1].is_ok &&
                        this->input.state.esc_voltages[1].value > 0.0 &&
                        !this->input.state.esc_water_leaked_flags[1].value
                    ? msg::HighPowerCircuitInfo::OK
                    : msg::HighPowerCircuitInfo::ERROR)
            .set__esc_rb(
                this->input.state.esc_health[2].is_ok &&
                        this->input.state.esc_voltages[2].value > 0.0 &&
                        !this->input.state.esc_water_leaked_flags[2].value
                    ? msg::HighPowerCircuitInfo::OK
                    : msg::HighPowerCircuitInfo::ERROR)
            .set__esc_rf(
                this->input.state.esc_health[3].is_ok &&
                        this->input.state.esc_voltages[3].value > 0.0 &&
                        !this->input.state.esc_water_leaked_flags[3].value
                    ? msg::HighPowerCircuitInfo::OK
                    : msg::HighPowerCircuitInfo::ERROR));

    // コマンドを送信
    util::interface_accessor::set_commands_to_loaned_interfaces(
        this->command_interfaces_, this->command_interface_data);

    return controller_interface::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::GateController, controller_interface::ControllerInterface)
