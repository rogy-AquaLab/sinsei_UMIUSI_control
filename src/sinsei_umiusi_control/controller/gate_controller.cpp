#include "sinsei_umiusi_control/controller/gate_controller.hpp"

#include <algorithm>
#include <rclcpp/logging.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>

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
                prefix + "esc_enabled",
                to_interface_data_ptr(this->cmd.thruster_esc_enabled_ref[i]));
            this->command_interface_data.emplace_back(
                prefix + "servo_enabled",
                to_interface_data_ptr(this->cmd.thruster_servo_enabled_ref[i]));
        }

        // Subscribers
        const auto cmd_prefix = std::string("cmd/");
        const auto qos = rclcpp::SystemDefaultsQoS();
        this->sub.indicator_led_output_subscriber =
            this->get_node()->create_subscription<msg::IndicatorLedOutput>(
                cmd_prefix + "indicator_led_output", qos,
                [this](const msg::IndicatorLedOutput::SharedPtr input) {
                    this->cmd.indicator_led_enabled_ref.value = input->enabled;
                });
        this->sub.main_power_output_subscriber =
            this->get_node()->create_subscription<msg::MainPowerOutput>(
                cmd_prefix + "main_power_output", qos,
                [this](const msg::MainPowerOutput::SharedPtr input) {
                    this->cmd.main_power_enabled_ref.value = input->enabled;
                });
        this->sub.led_tape_output_subscriber =
            this->get_node()->create_subscription<msg::LedTapeOutput>(
                cmd_prefix + "led_tape_output", qos,
                [this](const msg::LedTapeOutput::SharedPtr input) {
                    // alphaは無視
                    this->cmd.led_tape_color_ref.red =
                        static_cast<uint8_t>(std::clamp(input->color.r, 0.0f, 1.0f) * 255.0);
                    this->cmd.led_tape_color_ref.green =
                        static_cast<uint8_t>(std::clamp(input->color.g, 0.0f, 1.0f) * 255.0);
                    this->cmd.led_tape_color_ref.blue =
                        static_cast<uint8_t>(std::clamp(input->color.b, 0.0f, 1.0f) * 255.0);
                });
        this->sub.headlights_output_subscriber =
            this->get_node()->create_subscription<msg::HeadlightsOutput>(
                "head_lights_output", qos, [this](const msg::HeadlightsOutput::SharedPtr input) {
                    this->cmd.high_beam_enabled_ref.value = input->high_beam_enabled;
                    this->cmd.low_beam_enabled_ref.value = input->low_beam_enabled;
                    this->cmd.ir_enabled_ref.value = input->ir_enabled;
                });
        this->sub.thruster_enabled_all_subscriber =
            this->get_node()->create_subscription<msg::ThrusterEnabledAll>(
                cmd_prefix + "thruster_enabled_all", qos,
                [this](const msg::ThrusterEnabledAll::SharedPtr input) {
                    this->cmd.thruster_esc_enabled_ref[0].value = input->lf.esc;
                    this->cmd.thruster_esc_enabled_ref[1].value = input->lb.esc;
                    this->cmd.thruster_esc_enabled_ref[2].value = input->rb.esc;
                    this->cmd.thruster_esc_enabled_ref[3].value = input->rf.esc;
                    this->cmd.thruster_servo_enabled_ref[0].value = input->lf.servo;
                    this->cmd.thruster_servo_enabled_ref[1].value = input->lb.servo;
                    this->cmd.thruster_servo_enabled_ref[2].value = input->rb.servo;
                    this->cmd.thruster_servo_enabled_ref[3].value = input->rf.servo;
                });
        this->sub.target_subscriber = this->get_node()->create_subscription<msg::Target>(
            cmd_prefix + "target", qos, [this](const msg::Target::SharedPtr input) {
                this->cmd.target_orientation_ref.x = input->orientation.x;
                this->cmd.target_orientation_ref.y = input->orientation.y;
                this->cmd.target_orientation_ref.z = input->orientation.z;
                this->cmd.target_velocity_ref.x = input->velocity.x;
                this->cmd.target_velocity_ref.y = input->velocity.y;
                this->cmd.target_velocity_ref.z = input->velocity.z;
            });
    }
    {  // State interface
        using util::to_interface_data_ptr;

        this->state_interface_data.emplace_back(
            "main_power/battery_voltage",
            to_interface_data_ptr(this->state.main_power_battery_voltage));
        this->state_interface_data.emplace_back(
            "main_power/battery_current",
            to_interface_data_ptr(this->state.main_power_battery_current));
        this->state_interface_data.emplace_back(
            "main_power/temperature", to_interface_data_ptr(this->state.main_temperature));
        this->state_interface_data.emplace_back(
            "main_power/water_leaked", to_interface_data_ptr(this->state.water_leaked));
        this->state_interface_data.emplace_back(
            "imu/temperature", to_interface_data_ptr(this->state.imu_temperature));
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
                prefix + "esc_enabled", to_interface_data_ptr(this->state.thruster_esc_enabled[i]));
            this->state_interface_data.emplace_back(
                prefix + "servo_enabled",
                to_interface_data_ptr(this->state.thruster_servo_enabled[i]));
            this->state_interface_data.emplace_back(
                prefix + "duty_cycle", to_interface_data_ptr(this->state.thruster_duty_cycles[i]));
            this->state_interface_data.emplace_back(
                prefix + "angle", to_interface_data_ptr(this->state.thruster_angles[i]));
        }
        if (this->thruster_mode == util::ThrusterMode::Can) {
            for (size_t i = 0; i < 4; ++i) {
                const auto prefix = "attitude_controller/thruster_controller" +
                                    std::string(THRUSTER_SUFFIX[i]) + "/thruster/";

                this->state_interface_data.emplace_back(
                    prefix + "esc/rpm", to_interface_data_ptr(this->state.thruster_rpm[i]));

                this->state_interface_data.emplace_back(
                    "thruster" + std::to_string(i + 1) + "/esc/water_leaked",
                    to_interface_data_ptr(this->state.esc_water_leaked[i]));
            }
        }
        // Publishers
        const auto state_prefix = std::string("state/");
        const auto qos = rclcpp::SystemDefaultsQoS();
        this->pub.main_power_state_publisher =
            this->get_node()->create_publisher<msg::MainPowerState>(
                state_prefix + "main_power_state", qos);
        this->pub.imu_state_publisher =
            this->get_node()->create_publisher<msg::ImuState>(state_prefix + "imu_state", qos);
        this->pub.thruster_state_all_publisher =
            this->get_node()->create_publisher<msg::ThrusterStateAll>(
                state_prefix + "thruster_state_all", qos);
        if (this->thruster_mode == util::ThrusterMode::Can) {
            for (size_t i = 0; i < 4; ++i) {
                this->pub.esc_state_publishers[i] =
                    this->get_node()->create_publisher<msg::EscState>(
                        state_prefix + "esc" + std::to_string(i + 1) + "_state", qos);
            }
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
    this->pub.main_power_state_publisher->publish(
        msg::MainPowerState()
            .set__enabled(this->cmd.main_power_enabled_ref.value)
            .set__voltage(this->state.main_power_battery_voltage.value)
            .set__current(this->state.main_power_battery_current.value)
            .set__temperature(this->state.main_temperature.value)
            .set__water_leaked(this->state.water_leaked.value));
    this->pub.imu_state_publisher->publish(
        msg::ImuState()
            .set__velocity(geometry_msgs::msg::Vector3()
                               .set__x(this->state.imu_velocity.x)
                               .set__y(this->state.imu_velocity.y)
                               .set__z(this->state.imu_velocity.z))
            .set__quaternion(geometry_msgs::msg::Quaternion()
                                 .set__x(this->state.imu_quaternion.x)
                                 .set__y(this->state.imu_quaternion.y)
                                 .set__z(this->state.imu_quaternion.z)
                                 .set__w(this->state.imu_quaternion.w))
            .set__temperature(this->state.imu_temperature.value));
    this->pub.thruster_state_all_publisher->publish(
        msg::ThrusterStateAll()
            .set__lf(msg::ThrusterState()
                         .set__output(
                             msg::ThrusterOutput()
                                 .set__enabled(
                                     msg::ThrusterEnabled()
                                         .set__esc(this->state.thruster_esc_enabled[0].value)
                                         .set__servo(this->state.thruster_servo_enabled[0].value))
                                 .set__duty_cycle(this->state.thruster_duty_cycles[0].value)
                                 .set__angle(this->state.thruster_angles[0].value))
                         .set__rpm(this->state.thruster_rpm[0].value))
            .set__lb(msg::ThrusterState()
                         .set__output(
                             msg::ThrusterOutput()
                                 .set__enabled(
                                     msg::ThrusterEnabled()
                                         .set__esc(this->state.thruster_esc_enabled[1].value)
                                         .set__servo(this->state.thruster_servo_enabled[1].value))
                                 .set__duty_cycle(this->state.thruster_duty_cycles[1].value)
                                 .set__angle(this->state.thruster_angles[1].value))
                         .set__rpm(this->state.thruster_rpm[1].value))
            .set__rb(msg::ThrusterState()
                         .set__output(
                             msg::ThrusterOutput()
                                 .set__enabled(
                                     msg::ThrusterEnabled()
                                         .set__esc(this->state.thruster_esc_enabled[2].value)
                                         .set__servo(this->state.thruster_servo_enabled[2].value))
                                 .set__duty_cycle(this->state.thruster_duty_cycles[2].value)
                                 .set__angle(this->state.thruster_angles[2].value))
                         .set__rpm(this->state.thruster_rpm[2].value))
            .set__rf(msg::ThrusterState()
                         .set__output(
                             msg::ThrusterOutput()
                                 .set__enabled(
                                     msg::ThrusterEnabled()
                                         .set__esc(this->state.thruster_esc_enabled[3].value)
                                         .set__servo(this->state.thruster_servo_enabled[3].value))
                                 .set__duty_cycle(this->state.thruster_duty_cycles[3].value)
                                 .set__angle(this->state.thruster_angles[3].value))
                         .set__rpm(this->state.thruster_rpm[3].value)));
    if (this->thruster_mode == util::ThrusterMode::Can) {
        for (size_t i = 0; i < 4; ++i) {
            this->pub.esc_state_publishers[i]->publish(
                msg::EscState()
                    .set__voltage(this->state.main_power_battery_voltage
                                      .value)  // TODO: ESCの電圧センサを使う
                    .set__water_leaked(this->state.esc_water_leaked[i].value));
        }
    }

    // コマンドを送信
    util::interface_accessor::set_commands_to_loaned_interfaces(
        this->command_interfaces_, this->command_interface_data);

    return cif::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::GateController, controller_interface::ControllerInterface)
