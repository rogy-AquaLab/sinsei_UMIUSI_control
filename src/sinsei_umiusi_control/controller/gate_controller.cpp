#include "sinsei_umiusi_control/controller/gate_controller.hpp"

#include <algorithm>
#include <rclcpp/logging.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>

#include "sinsei_umiusi_control/util/interface_accessor.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"

using namespace sinsei_umiusi_control::controller;

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
    this->get_node()->declare_parameter("thruster_mode", "unknown");

    this->output.cmd = GateController::Output::Command{};
    this->input.state = GateController::Input::State{};

    return controller_interface::CallbackReturn::SUCCESS;
}

auto GateController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    -> controller_interface::CallbackReturn {
    const auto mode_str = this->get_node()->get_parameter("thruster_mode").as_string();
    const auto mode_res = util::get_mode_from_str(mode_str);
    if (!mode_res) {
        RCLCPP_ERROR(this->get_node()->get_logger(), "Invalid thruster mode: %s", mode_str.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }
    this->thruster_mode = mode_res.value();

    constexpr std::string_view THRUSTER_SUFFIX[4] = {"_lf", "_lb", "_rb", "_rf"};

    {  // Input
        // State interface (in)
        using util::to_interface_data_ptr;

        this->state_interface_data.emplace_back(
            "main_power/battery_voltage",
            to_interface_data_ptr(this->input.state.main_power_battery_voltage),
            sizeof(this->input.state.main_power_battery_voltage));
        this->state_interface_data.emplace_back(
            "main_power/battery_current",
            to_interface_data_ptr(this->input.state.main_power_battery_current),
            sizeof(this->input.state.main_power_battery_current));
        this->state_interface_data.emplace_back(
            "main_power/temperature", to_interface_data_ptr(this->input.state.main_temperature),
            sizeof(this->input.state.main_temperature));
        this->state_interface_data.emplace_back(
            "main_power/water_leaked", to_interface_data_ptr(this->input.state.water_leaked),
            sizeof(this->input.state.water_leaked));
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
            "attitude_controller/imu/velocity.x",
            to_interface_data_ptr(this->input.state.imu_velocity.x),
            sizeof(this->input.state.imu_velocity.x));
        this->state_interface_data.emplace_back(
            "attitude_controller/imu/velocity.y",
            to_interface_data_ptr(this->input.state.imu_velocity.y),
            sizeof(this->input.state.imu_velocity.y));
        this->state_interface_data.emplace_back(
            "attitude_controller/imu/velocity.z",
            to_interface_data_ptr(this->input.state.imu_velocity.z),
            sizeof(this->input.state.imu_velocity.z));
        for (size_t i = 0; i < 4; ++i) {
            const auto prefix = "thruster_controller" + std::string(THRUSTER_SUFFIX[i]) + "/";

            this->state_interface_data.emplace_back(
                prefix + "esc/enabled",
                to_interface_data_ptr(this->input.state.esc_enabled_flags[i]),
                sizeof(this->input.state.esc_enabled_flags[i]));
            this->state_interface_data.emplace_back(
                prefix + "esc/duty_cycle",
                to_interface_data_ptr(this->input.state.esc_duty_cycles[i]),
                sizeof(this->input.state.esc_duty_cycles[i]));
            this->state_interface_data.emplace_back(
                prefix + "servo/enabled",
                to_interface_data_ptr(this->input.state.servo_enabled_flags[i]),
                sizeof(this->input.state.servo_enabled_flags[i]));
            this->state_interface_data.emplace_back(
                prefix + "servo/angle", to_interface_data_ptr(this->input.state.servo_angles[i]),
                sizeof(this->input.state.servo_angles[i]));
        }
        if (this->thruster_mode == util::ThrusterMode::Can) {
            for (size_t i = 0; i < 4; ++i) {
                const auto prefix = "attitude_controller/thruster_controller" +
                                    std::string(THRUSTER_SUFFIX[i]) + "/thruster/";

                this->state_interface_data.emplace_back(
                    prefix + "esc/rpm", to_interface_data_ptr(this->input.state.esc_rpms[i]),
                    sizeof(this->input.state.esc_rpms[i]));
                this->state_interface_data.emplace_back(
                    prefix + "/esc/voltage",
                    to_interface_data_ptr(this->input.state.esc_voltages[i]),
                    sizeof(this->input.state.esc_voltages[i]));
                this->state_interface_data.emplace_back(
                    prefix + "/esc/water_leaked",
                    to_interface_data_ptr(this->input.state.esc_water_leaked_flags[i]),
                    sizeof(this->input.state.esc_water_leaked_flags[i]));
            }
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
        this->input.sub.main_power_output_subscriber =
            this->get_node()->create_subscription<msg::MainPowerOutput>(
                cmd_prefix + "main_power_output", qos,
                [this](const msg::MainPowerOutput::SharedPtr input) {
                    this->output.cmd.main_power_enabled_ref.value = input->enabled;
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
                "head_lights_output", qos, [this](const msg::HeadlightsOutput::SharedPtr input) {
                    this->output.cmd.high_beam_enabled_ref.value = input->high_beam_enabled;
                    this->output.cmd.low_beam_enabled_ref.value = input->low_beam_enabled;
                    this->output.cmd.ir_enabled_ref.value = input->ir_enabled;
                });
        this->input.sub.thruster_enabled_all_subscriber =
            this->get_node()->create_subscription<msg::ThrusterEnabledAll>(
                cmd_prefix + "thruster_enabled_all", qos,
                [this](const msg::ThrusterEnabledAll::SharedPtr input) {
                    this->output.cmd.esc_enabled_ref[0].value = input->lf.esc;
                    this->output.cmd.esc_enabled_ref[1].value = input->lb.esc;
                    this->output.cmd.esc_enabled_ref[2].value = input->rb.esc;
                    this->output.cmd.esc_enabled_ref[3].value = input->rf.esc;
                    this->output.cmd.servo_enabled_ref[0].value = input->lf.servo;
                    this->output.cmd.servo_enabled_ref[1].value = input->lb.servo;
                    this->output.cmd.servo_enabled_ref[2].value = input->rb.servo;
                    this->output.cmd.servo_enabled_ref[3].value = input->rf.servo;
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
            "main_power/enabled", to_interface_data_ptr(this->output.cmd.main_power_enabled_ref),
            sizeof(this->output.cmd.main_power_enabled_ref)));
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
                prefix + "esc/enabled", to_interface_data_ptr(this->output.cmd.esc_enabled_ref[i]),
                sizeof(this->output.cmd.esc_enabled_ref[i])));
            this->command_interface_data.push_back(std::make_tuple(
                prefix + "servo/enabled",
                to_interface_data_ptr(this->output.cmd.servo_enabled_ref[i]),
                sizeof(this->output.cmd.servo_enabled_ref[i])));
        }

        // Publishers
        const auto state_prefix = std::string("state/");
        const auto qos = rclcpp::SystemDefaultsQoS();
        this->output.pub.main_power_state_publisher =
            this->get_node()->create_publisher<msg::MainPowerState>(
                state_prefix + "main_power_state", qos);
        this->output.pub.imu_state_publisher =
            this->get_node()->create_publisher<msg::ImuState>(state_prefix + "imu_state", qos);
        this->output.pub.thruster_state_all_publisher =
            this->get_node()->create_publisher<msg::ThrusterStateAll>(
                state_prefix + "thruster_state_all", qos);
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

auto GateController::update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/
    ) -> controller_interface::return_type {
    // 状態を取得
    util::interface_accessor::get_states_from_loaned_interfaces(
        this->state_interfaces_, this->state_interface_data);

    // 状態をトピックに出力
    this->output.pub.main_power_state_publisher->publish(
        msg::MainPowerState()
            .set__enabled(this->output.cmd.main_power_enabled_ref.value)
            .set__voltage(this->input.state.main_power_battery_voltage.value)
            .set__current(this->input.state.main_power_battery_current.value)
            .set__temperature(this->input.state.main_temperature.value)
            .set__water_leaked(this->input.state.water_leaked.value));
    this->output.pub.imu_state_publisher->publish(
        msg::ImuState()
            .set__velocity(geometry_msgs::msg::Vector3()
                               .set__x(this->input.state.imu_velocity.x)
                               .set__y(this->input.state.imu_velocity.y)
                               .set__z(this->input.state.imu_velocity.z))
            .set__quaternion(geometry_msgs::msg::Quaternion()
                                 .set__x(this->input.state.imu_quaternion.x)
                                 .set__y(this->input.state.imu_quaternion.y)
                                 .set__z(this->input.state.imu_quaternion.z)
                                 .set__w(this->input.state.imu_quaternion.w))
            .set__temperature(this->input.state.imu_temperature.value));
    this->output.pub.thruster_state_all_publisher->publish(
        msg::ThrusterStateAll()
            .set__lf(
                msg::ThrusterState()
                    .set__output(
                        msg::ThrusterOutput()
                            .set__enabled(
                                msg::ThrusterEnabled()
                                    .set__esc(this->input.state.esc_enabled_flags[0].value)
                                    .set__servo(this->input.state.servo_enabled_flags[0].value))
                            .set__duty_cycle(this->input.state.esc_duty_cycles[0].value)
                            .set__angle(this->input.state.servo_angles[0].value))
                    .set__rpm(this->input.state.esc_rpms[0].value)
                    .set__voltage(this->input.state.esc_voltages[0].value)
                    .set__water_leaked(this->input.state.esc_water_leaked_flags[0].value))
            .set__lb(
                msg::ThrusterState()
                    .set__output(
                        msg::ThrusterOutput()
                            .set__enabled(
                                msg::ThrusterEnabled()
                                    .set__esc(this->input.state.esc_enabled_flags[1].value)
                                    .set__servo(this->input.state.servo_enabled_flags[1].value))
                            .set__duty_cycle(this->input.state.esc_duty_cycles[1].value)
                            .set__angle(this->input.state.servo_angles[1].value))
                    .set__rpm(this->input.state.esc_rpms[1].value)
                    .set__voltage(this->input.state.esc_voltages[1].value)
                    .set__water_leaked(this->input.state.esc_water_leaked_flags[1].value))
            .set__rb(
                msg::ThrusterState()
                    .set__output(
                        msg::ThrusterOutput()
                            .set__enabled(
                                msg::ThrusterEnabled()
                                    .set__esc(this->input.state.esc_enabled_flags[2].value)
                                    .set__servo(this->input.state.servo_enabled_flags[2].value))
                            .set__duty_cycle(this->input.state.esc_duty_cycles[2].value)
                            .set__angle(this->input.state.servo_angles[2].value))
                    .set__rpm(this->input.state.esc_rpms[2].value)
                    .set__voltage(this->input.state.esc_voltages[2].value)
                    .set__water_leaked(this->input.state.esc_water_leaked_flags[2].value))
            .set__rf(
                msg::ThrusterState()
                    .set__output(
                        msg::ThrusterOutput()
                            .set__enabled(
                                msg::ThrusterEnabled()
                                    .set__esc(this->input.state.esc_enabled_flags[3].value)
                                    .set__servo(this->input.state.servo_enabled_flags[3].value))
                            .set__duty_cycle(this->input.state.esc_duty_cycles[3].value)
                            .set__angle(this->input.state.servo_angles[3].value))
                    .set__rpm(this->input.state.esc_rpms[3].value)
                    .set__voltage(this->input.state.esc_voltages[3].value)
                    .set__water_leaked(this->input.state.esc_water_leaked_flags[3].value)));

    // コマンドを送信
    util::interface_accessor::set_commands_to_loaned_interfaces(
        this->command_interfaces_, this->command_interface_data);

    return controller_interface::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::GateController, controller_interface::ControllerInterface)
