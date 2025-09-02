#include "sinsei_umiusi_control/controller/gate_controller.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>

#include "sinsei_umiusi_control/msg/current.hpp"
#include "sinsei_umiusi_control/msg/duty_cycle.hpp"
#include "sinsei_umiusi_control/msg/enabled.hpp"
#include "sinsei_umiusi_control/msg/orientation.hpp"
#include "sinsei_umiusi_control/msg/temprature.hpp"
#include "sinsei_umiusi_control/msg/water_leaked.hpp"
#include "sinsei_umiusi_control/util/interface_accessor.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"

namespace succ = sinsei_umiusi_control::controller;
namespace suc_util = sinsei_umiusi_control::util;
namespace rlc = rclcpp_lifecycle;
namespace hif = hardware_interface;
namespace cif = controller_interface;

auto succ::GateController::command_interface_configuration() const -> cif::InterfaceConfiguration {
    auto cmd_names = std::vector<std::string>{};
    for (const auto & [name, _] : this->command_interface_data) {
        cmd_names.push_back(name);
    }

    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        cmd_names,
    };
}

auto succ::GateController::state_interface_configuration() const -> cif::InterfaceConfiguration {
    auto state_names = std::vector<std::string>{};
    for (const auto & [name, _] : this->state_interface_data) {
        state_names.push_back(name);
    }

    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        state_names,
    };
}

auto succ::GateController::on_init() -> cif::CallbackReturn {
    this->get_node()->declare_parameter("thruster_mode", "unknown");

    this->cmd = succ::GateController::Command{};
    this->state = succ::GateController::State{};

    return cif::CallbackReturn::SUCCESS;
}

auto succ::GateController::on_configure(const rlc::State & /*previous_state*/)
    -> cif::CallbackReturn {
    const auto mode_str = this->get_node()->get_parameter("thruster_mode").as_string();
    const auto mode_res = util::get_mode_from_str(mode_str);
    if (!mode_res) {
        RCLCPP_ERROR(this->get_node()->get_logger(), "Invalid thruster mode: %s", mode_str.c_str());
        return cif::CallbackReturn::ERROR;
    }
    this->thruster_mode = mode_res.value();

    constexpr std::string_view THRUSTER_SUFFIX[4] = {"_lf", "_lb", "_rb", "_rf"};

    {  // Command interface
        using util::to_interface_data_ptr;

        this->command_interface_data.emplace_back(
            "indicator_led/enabled", to_interface_data_ptr(this->cmd.indicator_led_enabled_ref));
        this->command_interface_data.emplace_back(
            "main_power/enabled", to_interface_data_ptr(this->cmd.main_power_enabled_ref));
        this->command_interface_data.emplace_back(
            "headlights/high_beam_enabled", to_interface_data_ptr(this->cmd.high_beam_enabled_ref));
        this->command_interface_data.emplace_back(
            "headlights/low_beam_enabled", to_interface_data_ptr(this->cmd.low_beam_enabled_ref));
        this->command_interface_data.emplace_back(
            "headlights/ir_enabled", to_interface_data_ptr(this->cmd.ir_enabled_ref));
        this->command_interface_data.emplace_back(
            "led_tape/color", to_interface_data_ptr(this->cmd.led_tape_color_ref));
        this->command_interface_data.emplace_back(
            "attitude_controller/target_orientation.x",
            to_interface_data_ptr(this->cmd.target_orientation_ref.x));
        this->command_interface_data.emplace_back(
            "attitude_controller/target_orientation.y",
            to_interface_data_ptr(this->cmd.target_orientation_ref.y));
        this->command_interface_data.emplace_back(
            "attitude_controller/target_orientation.z",
            to_interface_data_ptr(this->cmd.target_orientation_ref.z));
        this->command_interface_data.emplace_back(
            "attitude_controller/target_velocity.x",
            to_interface_data_ptr(this->cmd.target_velocity_ref.x));
        this->command_interface_data.emplace_back(
            "attitude_controller/target_velocity.y",
            to_interface_data_ptr(this->cmd.target_velocity_ref.y));
        this->command_interface_data.emplace_back(
            "attitude_controller/target_velocity.z",
            to_interface_data_ptr(this->cmd.target_velocity_ref.z));
        for (size_t i = 0; i < 4; ++i) {
            const auto prefix = "thruster_controller" + std::string(THRUSTER_SUFFIX[i]) + "/";

            this->command_interface_data.emplace_back(
                prefix + "servo_enabled", to_interface_data_ptr(this->cmd.servo_enabled_ref));
            this->command_interface_data.emplace_back(
                prefix + "esc_enabled", to_interface_data_ptr(this->cmd.esc_enabled_ref));
        }

        // Subscribers
        const auto cmd_prefix = std::string("cmd/");
        const auto qos = rclcpp::SystemDefaultsQoS();
        this->sub.indicator_led_enabled_subscriber =
            this->get_node()->create_subscription<msg::Enabled>(
                cmd_prefix + "indicator_led_enabled", qos,
                [this](const msg::Enabled::SharedPtr input) {
                    this->cmd.indicator_led_enabled_ref.value = input->value;
                });
        this->sub.main_power_enabled_subscriber =
            this->get_node()->create_subscription<msg::Enabled>(
                cmd_prefix + "main_power_enabled", qos,
                [this](const msg::Enabled::SharedPtr input) {
                    this->cmd.main_power_enabled_ref.value = input->value;
                });
        this->sub.led_tape_color_subscriber = this->get_node()->create_subscription<msg::Color>(
            cmd_prefix + "led_tape_color", qos, [this](const msg::Color::SharedPtr input) {
                // alphaは無視
                this->cmd.led_tape_color_ref.red = input->r;
                this->cmd.led_tape_color_ref.green = input->g;
                this->cmd.led_tape_color_ref.blue = input->b;
            });
        this->sub.high_beam_enabled_subscriber =
            this->get_node()->create_subscription<msg::Enabled>(
                cmd_prefix + "high_beam_enabled", qos, [this](const msg::Enabled::SharedPtr input) {
                    this->cmd.high_beam_enabled_ref.value = input->value;
                });
        this->sub.low_beam_enabled_subscriber = this->get_node()->create_subscription<msg::Enabled>(
            cmd_prefix + "low_beam_enabled", qos, [this](const msg::Enabled::SharedPtr input) {
                this->cmd.low_beam_enabled_ref.value = input->value;
            });
        this->sub.ir_enabled_subscriber = this->get_node()->create_subscription<msg::Enabled>(
            cmd_prefix + "ir_enabled", qos, [this](const msg::Enabled::SharedPtr input) {
                this->cmd.ir_enabled_ref.value = input->value;
            });
        this->sub.servo_enabled_subscriber = this->get_node()->create_subscription<msg::Enabled>(
            cmd_prefix + "servo_enabled", qos, [this](const msg::Enabled::SharedPtr input) {
                this->cmd.servo_enabled_ref.value = input->value;
            });
        this->sub.esc_enabled_subscriber = this->get_node()->create_subscription<msg::Enabled>(
            cmd_prefix + "esc_enabled", qos, [this](const msg::Enabled::SharedPtr input) {
                this->cmd.esc_enabled_ref.value = input->value;
            });
        this->sub.target_orientation_subscriber =
            this->get_node()->create_subscription<msg::Orientation>(
                cmd_prefix + "target_orientation", qos,
                [this](const msg::Orientation::SharedPtr input) {
                    this->cmd.target_orientation_ref.x = input->x;
                    this->cmd.target_orientation_ref.y = input->y;
                    this->cmd.target_orientation_ref.z = input->z;
                });
        this->sub.target_velocity_subscriber = this->get_node()->create_subscription<msg::Velocity>(
            cmd_prefix + "target_velocity", qos, [this](const msg::Velocity::SharedPtr input) {
                this->cmd.target_velocity_ref.x = input->x;
                this->cmd.target_velocity_ref.y = input->y;
                this->cmd.target_velocity_ref.z = input->z;
            });
    }
    {  // State interface
        using util::to_interface_data_ptr;

        this->state_interface_data.emplace_back(
            "main_power/battery_current", to_interface_data_ptr(this->state.battery_current.value));
        this->state_interface_data.emplace_back(
            "main_power/battery_voltage", to_interface_data_ptr(this->state.battery_voltage.value));
        this->state_interface_data.emplace_back(
            "main_power/temperature", to_interface_data_ptr(this->state.main_temperature.value));
        this->state_interface_data.emplace_back(
            "main_power/water_leaked", to_interface_data_ptr(this->state.water_leaked.value));
        this->state_interface_data.emplace_back(
            "imu/temperature", to_interface_data_ptr(this->state.imu_temperature.value));
        this->state_interface_data.emplace_back(
            "attitude_controller/imu/quaternion.x",
            to_interface_data_ptr(this->state.imu_quaternion.x));
        this->state_interface_data.emplace_back(
            "attitude_controller/imu/quaternion.y",
            to_interface_data_ptr(this->state.imu_quaternion.y));
        this->state_interface_data.emplace_back(
            "attitude_controller/imu/quaternion.z",
            to_interface_data_ptr(this->state.imu_quaternion.z));
        this->state_interface_data.emplace_back(
            "attitude_controller/imu/quaternion.w",
            to_interface_data_ptr(this->state.imu_quaternion.w));
        this->state_interface_data.emplace_back(
            "attitude_controller/imu/velocity.x",
            to_interface_data_ptr(this->state.imu_velocity.x));
        this->state_interface_data.emplace_back(
            "attitude_controller/imu/velocity.y",
            to_interface_data_ptr(this->state.imu_velocity.y));
        this->state_interface_data.emplace_back(
            "attitude_controller/imu/velocity.z",
            to_interface_data_ptr(this->state.imu_velocity.z));
        for (size_t i = 0; i < 4; ++i) {
            const auto prefix = "thruster_controller" + std::string(THRUSTER_SUFFIX[i]) + "/";

            this->state_interface_data.emplace_back(
                prefix + "esc_enabled", to_interface_data_ptr(this->state.esc_enabled[i].value));
            this->state_interface_data.emplace_back(
                prefix + "servo_enabled",
                to_interface_data_ptr(this->state.servo_enabled[i].value));
            this->state_interface_data.emplace_back(
                prefix + "duty_cycle",
                to_interface_data_ptr(this->state.thruster_duty_cycles[i].value));
            this->state_interface_data.emplace_back(
                prefix + "angle", to_interface_data_ptr(this->state.thruster_angles[i].value));
        }
        if (this->thruster_mode == util::ThrusterMode::Can) {
            for (size_t i = 0; i < 4; ++i) {
                const auto prefix = "attitude_controller/thruster_controller" +
                                    std::string(THRUSTER_SUFFIX[i]) + "/thruster/";

                this->state_interface_data.emplace_back(
                    prefix + "esc/rpm", to_interface_data_ptr(this->state.rpm[i].value));
                this->state_interface_data.emplace_back(
                    "thruster" + std::to_string(i + 1) + "/esc/water_leaked",
                    to_interface_data_ptr(this->state.esc_water_leaked[i].value));
            }
        }
        // Publishers
        const auto state_prefix = std::string("state/");
        const auto qos = rclcpp::SystemDefaultsQoS();
        this->pub.battery_current_publisher =
            this->get_node()->create_publisher<msg::Current>(state_prefix + "battery_current", qos);
        this->pub.battery_voltage_publisher =
            this->get_node()->create_publisher<msg::Voltage>(state_prefix + "battery_voltage", qos);
        this->pub.main_temperature_publisher = this->get_node()->create_publisher<msg::Temprature>(
            state_prefix + "main_temperature", qos);
        this->pub.water_leaked_publisher = this->get_node()->create_publisher<msg::WaterLeaked>(
            state_prefix + "water_leaked", qos);
        this->pub.imu_temperature_publisher = this->get_node()->create_publisher<msg::Temprature>(
            state_prefix + "imu_temperature", qos);
        this->pub.imu_quaternion_publisher = this->get_node()->create_publisher<msg::Quaternion>(
            state_prefix + "imu_quaternion", qos);
        this->pub.imu_velocity_publisher =
            this->get_node()->create_publisher<msg::Velocity>(state_prefix + "imu_velocity", qos);
        for (size_t i = 0; i < 4; ++i) {
            using util::to_interface_data_ptr;

            this->pub.rpm_publisher[i] = this->get_node()->create_publisher<msg::Rpm>(
                state_prefix + "thruster_rpm" + std::string(THRUSTER_SUFFIX[i]), qos);

            this->pub.esc_enabled_publisher[i] = this->get_node()->create_publisher<msg::Enabled>(
                state_prefix + "thruster_esc_enabled" + std::string(THRUSTER_SUFFIX[i]), qos);
            this->pub.servo_enabled_publisher[i] = this->get_node()->create_publisher<msg::Enabled>(
                state_prefix + "thruster_servo_enabled" + std::string(THRUSTER_SUFFIX[i]), qos);
            this->pub.duty_cycles_publisher[i] = this->get_node()->create_publisher<msg::DutyCycle>(
                state_prefix + "thruster_duty_cycle" + std::string(THRUSTER_SUFFIX[i]), qos);
            this->pub.angle_publisher[i] = this->get_node()->create_publisher<msg::Angle>(
                state_prefix + "thruster_angle" + std::string(THRUSTER_SUFFIX[i]), qos);

            this->pub.esc_water_leaked_publisher[i] =
                this->get_node()->create_publisher<msg::WaterLeaked>(
                    state_prefix + "esc" + std::to_string(i + 1) + "_water_leaked", qos);
        }
    }

    return cif::CallbackReturn::SUCCESS;
}

auto succ::GateController::update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/
    ) -> cif::return_type {
    // 状態を取得
    util::interface_accessor::get_states_from_loaned_interfaces(
        this->state_interfaces_, this->state_interface_data);

    // 状態をトピックに出力
    this->pub.battery_current_publisher->publish(
        msg::Current().set__value(this->state.battery_current.value));
    this->pub.battery_voltage_publisher->publish(
        msg::Voltage().set__value(this->state.battery_voltage.value));
    this->pub.main_temperature_publisher->publish(
        msg::Temprature().set__value(this->state.main_temperature.value));
    this->pub.water_leaked_publisher->publish(
        msg::WaterLeaked().set__value(this->state.water_leaked.value));
    this->pub.imu_temperature_publisher->publish(
        msg::Temprature().set__value(this->state.imu_temperature.value));
    this->pub.imu_quaternion_publisher->publish(msg::Quaternion()
                                                    .set__x(this->state.imu_quaternion.x)
                                                    .set__y(this->state.imu_quaternion.y)
                                                    .set__z(this->state.imu_quaternion.z)
                                                    .set__w(this->state.imu_quaternion.w));
    this->pub.imu_velocity_publisher->publish(msg::Velocity()
                                                  .set__x(this->state.imu_velocity.x)
                                                  .set__y(this->state.imu_velocity.y)
                                                  .set__z(this->state.imu_velocity.z));
    for (size_t i = 0; i < 4; ++i) {
        this->pub.rpm_publisher[i]->publish(msg::Rpm().set__value(this->state.rpm[i].value));
        this->pub.esc_enabled_publisher[i]->publish(
            msg::Enabled().set__value(this->state.esc_enabled[i].value));
        this->pub.servo_enabled_publisher[i]->publish(
            msg::Enabled().set__value(this->state.servo_enabled[i].value));
        this->pub.duty_cycles_publisher[i]->publish(
            msg::DutyCycle().set__value(this->state.thruster_duty_cycles[i].value));
        this->pub.angle_publisher[i]->publish(
            msg::Angle().set__value(this->state.thruster_angles[i].value));
        this->pub.esc_water_leaked_publisher[i]->publish(
            msg::WaterLeaked().set__value(this->state.esc_water_leaked[i].value));
    }

    // コマンドを送信
    util::interface_accessor::set_commands_to_loaned_interfaces(
        this->command_interfaces_, this->command_interface_data);

    return cif::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::GateController, controller_interface::ControllerInterface)
