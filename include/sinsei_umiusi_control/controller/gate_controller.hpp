#ifndef SINSEI_UMIUSI_CONTROL_GATE_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_GATE_CONTROLLER_HPP

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/header.hpp>

#include "sinsei_umiusi_control/cmd/attitude.hpp"
#include "sinsei_umiusi_control/cmd/headlights.hpp"
#include "sinsei_umiusi_control/cmd/indicator_led.hpp"
#include "sinsei_umiusi_control/cmd/led_tape.hpp"
#include "sinsei_umiusi_control/cmd/power_distribution.hpp"
#include "sinsei_umiusi_control/cmd/thruster/esc.hpp"
#include "sinsei_umiusi_control/cmd/thruster/servo.hpp"
#include "sinsei_umiusi_control/state/bms.hpp"
#include "sinsei_umiusi_control/state/can.hpp"
#include "sinsei_umiusi_control/state/headlights.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"
#include "sinsei_umiusi_control/state/indicator_led.hpp"
#include "sinsei_umiusi_control/state/thruster/esc.hpp"
#include "sinsei_umiusi_control/state/thruster/servo.hpp"
#include "sinsei_umiusi_control/util/interface_accessor.hpp"
#include "sinsei_umiusi_msgs/msg/bms_state.hpp"
#include "sinsei_umiusi_msgs/msg/headlights_output.hpp"
#include "sinsei_umiusi_msgs/msg/high_power_circuit_info.hpp"
#include "sinsei_umiusi_msgs/msg/indicator_led_output.hpp"
#include "sinsei_umiusi_msgs/msg/led_tape_output.hpp"
#include "sinsei_umiusi_msgs/msg/low_power_circuit_info.hpp"
#include "sinsei_umiusi_msgs/msg/power_distribution_enabled.hpp"
#include "sinsei_umiusi_msgs/msg/power_distribution_output.hpp"
#include "sinsei_umiusi_msgs/msg/target.hpp"
#include "sinsei_umiusi_msgs/msg/thruster_runnable_all.hpp"
#include "sinsei_umiusi_msgs/msg/thruster_state_all.hpp"

namespace sinsei_umiusi_control::controller {

class GateController : public controller_interface::ControllerInterface {
  public:
    struct Input {
        // State interfaces (in)
        struct State {
            sinsei_umiusi_control::state::bms::Boolean bms_health;
            sinsei_umiusi_control::state::bms::Scalar bms_pack_voltage;
            sinsei_umiusi_control::state::bms::Scalar bms_charger_voltage;
            sinsei_umiusi_control::state::bms::Scalar bms_input_current;
            sinsei_umiusi_control::state::bms::Scalar bms_measured_current;
            sinsei_umiusi_control::state::bms::Scalar bms_state_of_charge;
            sinsei_umiusi_control::state::bms::Scalar bms_state_of_health;
            sinsei_umiusi_control::state::bms::Scalar bms_cell_voltage_min;
            sinsei_umiusi_control::state::bms::Scalar bms_cell_voltage_max;
            sinsei_umiusi_control::state::bms::Scalar bms_cell_temperature_max;
            sinsei_umiusi_control::state::bms::Boolean bms_charging;
            sinsei_umiusi_control::state::bms::Boolean bms_balancing;
            sinsei_umiusi_control::state::bms::Boolean bms_charge_allowed;
            sinsei_umiusi_control::state::bms::Count bms_cell_count;
            std::array<sinsei_umiusi_control::state::bms::Scalar, 12> bms_cell_voltages;
            std::array<sinsei_umiusi_control::state::bms::Boolean, 12> bms_cell_balancing;
            sinsei_umiusi_control::state::bms::Count bms_temperature_count;
            std::array<sinsei_umiusi_control::state::bms::Scalar, 9> bms_temperatures;
            sinsei_umiusi_control::state::bms::Scalar bms_humidity_sensor_temperature;
            sinsei_umiusi_control::state::bms::Scalar bms_relative_humidity;
            sinsei_umiusi_control::state::bms::Scalar bms_balance_ic_temperature;
            sinsei_umiusi_control::state::bms::Scalar bms_net_consumed_charge;
            sinsei_umiusi_control::state::bms::Scalar bms_net_consumed_energy;
            sinsei_umiusi_control::state::bms::Scalar bms_total_charged_charge;
            sinsei_umiusi_control::state::bms::Scalar bms_total_charged_energy;
            sinsei_umiusi_control::state::bms::Scalar bms_total_discharged_charge;
            sinsei_umiusi_control::state::bms::Scalar bms_total_discharged_energy;
            sinsei_umiusi_control::state::bms::PowerSwitchState bms_power_switch_state;
            sinsei_umiusi_control::state::bms::FaultFlags bms_fault_flags;
            sinsei_umiusi_control::state::bms::Count bms_data_version;
            std::array<sinsei_umiusi_control::state::bms::StatusChunk, 5> bms_status_chunks;
            sinsei_umiusi_control::state::imu::Temperature imu_temperature;
            sinsei_umiusi_control::state::imu::Quaternion imu_quaternion;
            sinsei_umiusi_control::state::imu::Acceleration imu_acceleration;
            sinsei_umiusi_control::state::imu::AngularVelocity imu_angular_velocity;
            std::array<sinsei_umiusi_control::state::thruster::esc::Mode, 4> esc_modes;
            std::array<sinsei_umiusi_control::state::thruster::esc::DutyCycle, 4> esc_duty_cycles;
            std::array<sinsei_umiusi_control::state::thruster::esc::Rpm, 4> esc_rpms;
            std::array<sinsei_umiusi_control::state::thruster::esc::Voltage, 4> esc_voltages;
            std::array<sinsei_umiusi_control::state::thruster::esc::WaterLeaked, 4>
                esc_water_leaked_flags;
            std::array<sinsei_umiusi_control::state::thruster::esc::Health, 4> esc_health;
            std::array<sinsei_umiusi_control::state::thruster::servo::Mode, 4> servo_modes;
            std::array<sinsei_umiusi_control::state::thruster::servo::Angle, 4> servo_angles;
            sinsei_umiusi_control::state::can::Health can_health;
            sinsei_umiusi_control::state::headlights::Health headlights_health;
            sinsei_umiusi_control::state::imu::Health imu_health;
            sinsei_umiusi_control::state::indicator_led::Health indicator_led_health;
        };
        // Subscribers for commands
        struct Subscribers {
            rclcpp::Subscription<sinsei_umiusi_msgs::msg::IndicatorLedOutput>::SharedPtr
                indicator_led_output_subscriber;
            rclcpp::Subscription<sinsei_umiusi_msgs::msg::PowerDistributionOutput>::SharedPtr
                power_distribution_output_subscriber;
            rclcpp::Subscription<sinsei_umiusi_msgs::msg::HeadlightsOutput>::SharedPtr
                headlights_output_subscriber;
            rclcpp::Subscription<sinsei_umiusi_msgs::msg::ThrusterRunnableAll>::SharedPtr
                thruster_runnable_all_subscriber;
            rclcpp::Subscription<sinsei_umiusi_msgs::msg::LedTapeOutput>::SharedPtr
                led_tape_output_subscriber;
            rclcpp::Subscription<sinsei_umiusi_msgs::msg::Target>::SharedPtr target_subscriber;
        };
        State state;
        Subscribers sub;
    };
    struct Output {
        // Command interfaces (out)
        struct Command {
            sinsei_umiusi_control::cmd::indicator_led::Enabled indicator_led_enabled_ref;
            sinsei_umiusi_control::cmd::power_distribution::Enabled power_distribution_enabled_ref;
            sinsei_umiusi_control::cmd::headlights::HighBeamEnabled high_beam_enabled_ref;
            sinsei_umiusi_control::cmd::headlights::LowBeamEnabled low_beam_enabled_ref;
            sinsei_umiusi_control::cmd::headlights::IrEnabled ir_enabled_ref;

            std::array<sinsei_umiusi_control::cmd::thruster::esc::Runnable, 4> esc_runnable_refs;
            std::array<sinsei_umiusi_control::cmd::thruster::servo::Runnable, 4>
                servo_runnable_refs;

            sinsei_umiusi_control::cmd::led_tape::Color led_tape_color_ref;
            sinsei_umiusi_control::cmd::attitude::Orientation target_orientation_ref;
            sinsei_umiusi_control::cmd::attitude::Velocity target_velocity_ref;
        };
        // Publishers for states
        struct Publishers {
            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
            rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr imu_temperature_publisher;
            rclcpp::Publisher<sinsei_umiusi_msgs::msg::PowerDistributionEnabled>::SharedPtr
                power_distribution_enabled_publisher;
            rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher;
            rclcpp::Publisher<sinsei_umiusi_msgs::msg::BmsState>::SharedPtr bms_state_publisher;
            rclcpp::Publisher<sinsei_umiusi_msgs::msg::ThrusterStateAll>::SharedPtr
                thruster_state_all_publisher;
            rclcpp::Publisher<sinsei_umiusi_msgs::msg::LowPowerCircuitInfo>::SharedPtr
                low_power_circuit_info_publisher;
            rclcpp::Publisher<sinsei_umiusi_msgs::msg::HighPowerCircuitInfo>::SharedPtr
                high_power_circuit_info_publisher;
        };
        Command cmd;
        Publishers pub;
    };

  private:
    Input input;
    Output output;

    sinsei_umiusi_control::util::interface_accessor::InterfaceDataContainer command_interface_data;
    sinsei_umiusi_control::util::interface_accessor::InterfaceDataContainer state_interface_data;

  public:
    GateController() = default;

    auto command_interface_configuration() const
        -> controller_interface::InterfaceConfiguration override;
    auto state_interface_configuration() const
        -> controller_interface::InterfaceConfiguration override;
    auto on_init() -> CallbackReturn override;
    auto on_configure(const rclcpp_lifecycle::State & previous_state) -> CallbackReturn override;
    auto update(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> controller_interface::return_type override;
};
}  // namespace sinsei_umiusi_control::controller

#endif  // SINSEI_UMIUSI_CONTROL_GATE_CONTROLLER_HPP
