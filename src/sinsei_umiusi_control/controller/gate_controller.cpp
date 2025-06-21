#include "sinsei_umiusi_control/controller/gate_controller.hpp"

#include <rclcpp/logging.hpp>

#include "sinsei_umiusi_control/util.hpp"

namespace succ = sinsei_umiusi_control::controller;
namespace suc_util = sinsei_umiusi_control::util;
namespace rlc = rclcpp_lifecycle;
namespace hif = hardware_interface;
namespace cif = controller_interface;

auto succ::GateController::command_interface_configuration() const -> cif::InterfaceConfiguration {
    std::vector<std::string> cmd_names(
        std::begin(this->CMD_INTERFACE_NAMES), std::end(this->CMD_INTERFACE_NAMES));

    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        cmd_names,
    };
}

auto succ::GateController::state_interface_configuration() const -> cif::InterfaceConfiguration {
    std::vector<std::string> state_names(
        std::begin(this->STATE_INTERFACE_NAMES), std::end(this->STATE_INTERFACE_NAMES));

    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        state_names,
    };
}

auto succ::GateController::on_init() -> cif::CallbackReturn {
    this->interface_helper = std::make_unique<
        InterfaceAccessHelper<rclcpp_lifecycle::LifecycleNode, CMD_SIZE, STATE_SIZE>>(
        this->get_node().get(), this->command_interfaces_, this->CMD_INTERFACE_NAMES,
        this->state_interfaces_, this->STATE_INTERFACE_NAMES);

    return cif::CallbackReturn::SUCCESS;
}

auto succ::GateController::on_configure(const rlc::State & /*pervious_state*/)
    -> cif::CallbackReturn {
    this->indicator_led_enabled_subscriber =
        this->get_node()->create_subscription<std_msgs::msg::Bool>(
            "indicator_led_enabled", rclcpp::SystemDefaultsQoS(),
            [this](const std_msgs::msg::Bool::SharedPtr input) {
                this->indicator_led_enabled_ref.value = input->data;
            });
    this->main_power_enabled_subscriber =
        this->get_node()->create_subscription<std_msgs::msg::Bool>(
            "main_power_enabled", rclcpp::SystemDefaultsQoS(),
            [this](const std_msgs::msg::Bool::SharedPtr input) {
                this->main_power_enabled_ref.value = input->data;
            });
    this->led_tape_color_subscriber =
        this->get_node()->create_subscription<std_msgs::msg::ColorRGBA>(
            "led_tape_color", rclcpp::SystemDefaultsQoS(),
            [this](const std_msgs::msg::ColorRGBA::SharedPtr input) {
                // alphaは無視
                this->led_tape_color_ref.red = static_cast<uint8_t>(input->r);
                this->led_tape_color_ref.green = static_cast<uint8_t>(input->g);
                this->led_tape_color_ref.blue = static_cast<uint8_t>(input->b);
            });
    this->high_beam_enabled_subscriber = this->get_node()->create_subscription<std_msgs::msg::Bool>(
        "high_beam_enabled", rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Bool::SharedPtr input) {
            this->high_beam_enabled_ref.value = input->data;
        });
    this->low_beam_enabled_subscriber = this->get_node()->create_subscription<std_msgs::msg::Bool>(
        "low_beam_enabled", rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Bool::SharedPtr input) {
            this->low_beam_enabled_ref.value = input->data;
        });
    this->ir_enabled_subscriber = this->get_node()->create_subscription<std_msgs::msg::Bool>(
        "ir_enabled", rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Bool::SharedPtr input) {
            this->ir_enabled_ref.value = input->data;
        });
    // TODO: usb_camera, raspi_camera
    this->servo_enabled_subscriber = this->get_node()->create_subscription<std_msgs::msg::Bool>(
        "servo_enabled", rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Bool::SharedPtr input) {
            this->servo_enabled_ref.value = input->data;
        });
    this->esc_enabled_subscriber = this->get_node()->create_subscription<std_msgs::msg::Bool>(
        "esc_enabled", rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Bool::SharedPtr input) {
            this->esc_enabled_ref.value = input->data;
        });
    this->target_orientation_subscriber =
        this->get_node()->create_subscription<geometry_msgs::msg::Vector3>(
            "target_orientation", rclcpp::SystemDefaultsQoS(),
            [this](const geometry_msgs::msg::Vector3::SharedPtr input) {
                this->target_orientation_ref.x = input->x;
                this->target_orientation_ref.y = input->y;
                this->target_orientation_ref.z = input->z;
            });
    this->target_velocity_subscriber =
        this->get_node()->create_subscription<geometry_msgs::msg::Vector3>(
            "target_velocity", rclcpp::SystemDefaultsQoS(),
            [this](const geometry_msgs::msg::Vector3::SharedPtr input) {
                this->target_velocity_ref.x = input->x;
                this->target_velocity_ref.y = input->y;
                this->target_velocity_ref.z = input->z;
            });

    this->battery_current_publisher = this->get_node()->create_publisher<std_msgs::msg::Float64>(
        "battery_current", rclcpp::SystemDefaultsQoS());
    this->battery_voltage_publisher = this->get_node()->create_publisher<std_msgs::msg::Float64>(
        "battery_voltage", rclcpp::SystemDefaultsQoS());
    this->main_temperature_publisher = this->get_node()->create_publisher<std_msgs::msg::Int8>(
        "main_temperature", rclcpp::SystemDefaultsQoS());
    this->water_leaked_publisher = this->get_node()->create_publisher<std_msgs::msg::Bool>(
        "water_leaked", rclcpp::SystemDefaultsQoS());
    for (size_t i = 0; i < 4; ++i) {
        this->servo_current_publisher[i] =
            this->get_node()->create_publisher<std_msgs::msg::Float64>(
                "servo_current_" + std::to_string(i + 1), rclcpp::SystemDefaultsQoS());
        this->rpm_publisher[i] = this->get_node()->create_publisher<std_msgs::msg::Float64>(
            "rpm_" + std::to_string(i + 1), rclcpp::SystemDefaultsQoS());
    }
    this->imu_orientation_publisher =
        this->get_node()->create_publisher<geometry_msgs::msg::Vector3>(
            "imu_orientation", rclcpp::SystemDefaultsQoS());
    this->imu_velocity_publisher = this->get_node()->create_publisher<geometry_msgs::msg::Vector3>(
        "imu_velocity", rclcpp::SystemDefaultsQoS());
    this->imu_temperature_publisher = this->get_node()->create_publisher<std_msgs::msg::Float64>(
        "imu_temperature", rclcpp::SystemDefaultsQoS());

    return cif::CallbackReturn::SUCCESS;
}

auto succ::GateController::on_activate(const rlc::State & /*previous_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto succ::GateController::on_deactivate(const rlc::State & /*previous_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto succ::GateController::update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/
    ) -> cif::return_type {
    constexpr auto INDICATOR_LED_ENABLED_INDEX =
        suc_util::get_index("indicator_led/indicator_led/enabled", CMD_INTERFACE_NAMES);
    constexpr auto MAIN_POWER_ENABLED_INDEX =
        suc_util::get_index("main_power/main_power/enabled", CMD_INTERFACE_NAMES);
    constexpr auto LED_TAPE_COLOR_INDEX =
        suc_util::get_index("led_tape/led_tape/color", CMD_INTERFACE_NAMES);
    constexpr auto HIGH_BEAM_ENABLED_INDEX =
        suc_util::get_index("headlights/headlights/high_beam_enabled", CMD_INTERFACE_NAMES);
    constexpr auto LOW_BEAM_ENABLED_INDEX =
        suc_util::get_index("headlights/headlights/low_beam_enabled", CMD_INTERFACE_NAMES);
    constexpr auto IR_ENABLED_INDEX =
        suc_util::get_index("headlights/headlights/ir_enabled", CMD_INTERFACE_NAMES);
    // TODO: usb_camera, raspi_camera
    constexpr auto SERVO1_ENABLED_INDEX = suc_util::get_index(
        "thruster_controller1/servo_enabled/servo_enabled", CMD_INTERFACE_NAMES);
    constexpr auto SERVO2_ENABLED_INDEX = suc_util::get_index(
        "thruster_controller2/servo_enabled/servo_enabled", CMD_INTERFACE_NAMES);
    constexpr auto SERVO3_ENABLED_INDEX = suc_util::get_index(
        "thruster_controller3/servo_enabled/servo_enabled", CMD_INTERFACE_NAMES);
    constexpr auto SERVO4_ENABLED_INDEX = suc_util::get_index(
        "thruster_controller4/servo_enabled/servo_enabled", CMD_INTERFACE_NAMES);
    constexpr auto ESC1_ENABLED_INDEX =
        suc_util::get_index("thruster_controller1/esc_enabled/esc_enabled", CMD_INTERFACE_NAMES);
    constexpr auto ESC2_ENABLED_INDEX =
        suc_util::get_index("thruster_controller2/esc_enabled/esc_enabled", CMD_INTERFACE_NAMES);
    constexpr auto ESC3_ENABLED_INDEX =
        suc_util::get_index("thruster_controller3/esc_enabled/esc_enabled", CMD_INTERFACE_NAMES);
    constexpr auto ESC4_ENABLED_INDEX =
        suc_util::get_index("thruster_controller4/esc_enabled/esc_enabled", CMD_INTERFACE_NAMES);
    constexpr auto TARGET_ORIENTATION_X_INDEX = suc_util::get_index(
        "app_controller/target_orientation.x/target_orientation.x", CMD_INTERFACE_NAMES);
    constexpr auto TARGET_ORIENTATION_Y_INDEX = suc_util::get_index(
        "app_controller/target_orientation.y/target_orientation.y", CMD_INTERFACE_NAMES);
    constexpr auto TARGET_ORIENTATION_Z_INDEX = suc_util::get_index(
        "app_controller/target_orientation.z/target_orientation.z", CMD_INTERFACE_NAMES);
    constexpr auto TARGET_VELOCITY_X_INDEX = suc_util::get_index(
        "app_controller/target_velocity.x/target_velocity.x", CMD_INTERFACE_NAMES);
    constexpr auto TARGET_VELOCITY_Y_INDEX = suc_util::get_index(
        "app_controller/target_velocity.y/target_velocity.y", CMD_INTERFACE_NAMES);
    constexpr auto TARGET_VELOCITY_Z_INDEX = suc_util::get_index(
        "app_controller/target_velocity.z/target_velocity.z", CMD_INTERFACE_NAMES);

    this->interface_helper->set_cmd_value(
        INDICATOR_LED_ENABLED_INDEX, this->indicator_led_enabled_ref);
    this->interface_helper->set_cmd_value(MAIN_POWER_ENABLED_INDEX, this->main_power_enabled_ref);
    this->interface_helper->set_cmd_value(LED_TAPE_COLOR_INDEX, this->led_tape_color_ref);
    this->interface_helper->set_cmd_value(HIGH_BEAM_ENABLED_INDEX, this->high_beam_enabled_ref);
    this->interface_helper->set_cmd_value(LOW_BEAM_ENABLED_INDEX, this->low_beam_enabled_ref);
    this->interface_helper->set_cmd_value(IR_ENABLED_INDEX, this->ir_enabled_ref);
    // TODO: usb_camera, raspi_camera
    this->interface_helper->set_cmd_value(SERVO1_ENABLED_INDEX, this->servo_enabled_ref);
    this->interface_helper->set_cmd_value(SERVO2_ENABLED_INDEX, this->servo_enabled_ref);
    this->interface_helper->set_cmd_value(SERVO3_ENABLED_INDEX, this->servo_enabled_ref);
    this->interface_helper->set_cmd_value(SERVO4_ENABLED_INDEX, this->servo_enabled_ref);
    this->interface_helper->set_cmd_value(ESC1_ENABLED_INDEX, this->esc_enabled_ref);
    this->interface_helper->set_cmd_value(ESC2_ENABLED_INDEX, this->esc_enabled_ref);
    this->interface_helper->set_cmd_value(ESC3_ENABLED_INDEX, this->esc_enabled_ref);
    this->interface_helper->set_cmd_value(ESC4_ENABLED_INDEX, this->esc_enabled_ref);
    this->interface_helper->set_cmd_value(
        TARGET_ORIENTATION_X_INDEX, this->target_orientation_ref.x);
    this->interface_helper->set_cmd_value(
        TARGET_ORIENTATION_Y_INDEX, this->target_orientation_ref.y);
    this->interface_helper->set_cmd_value(
        TARGET_ORIENTATION_Z_INDEX, this->target_orientation_ref.z);
    this->interface_helper->set_cmd_value(TARGET_VELOCITY_X_INDEX, this->target_velocity_ref.x);
    this->interface_helper->set_cmd_value(TARGET_VELOCITY_Y_INDEX, this->target_velocity_ref.y);
    this->interface_helper->set_cmd_value(TARGET_VELOCITY_Z_INDEX, this->target_velocity_ref.z);

    this->battery_current_publisher->publish(
        std_msgs::msg::Float64().set__data(this->battery_current_ref.value));
    this->battery_voltage_publisher->publish(
        std_msgs::msg::Float64().set__data(this->battery_voltage_ref.value));
    this->main_temperature_publisher->publish(
        std_msgs::msg::Int8().set__data(this->main_temperature_ref.value));
    this->water_leaked_publisher->publish(
        std_msgs::msg::Bool().set__data(this->water_leaked_ref.value));
    for (size_t i = 0; i < 4; ++i) {
        this->servo_current_publisher[i]->publish(
            std_msgs::msg::Float64().set__data(this->servo_current_ref[i].value));
        this->rpm_publisher[i]->publish(std_msgs::msg::Float64().set__data(this->rpm_ref[i].value));
    }
    this->imu_orientation_publisher->publish(geometry_msgs::msg::Vector3()
                                                 .set__x(this->imu_orientation_ref.x)
                                                 .set__y(this->imu_orientation_ref.y)
                                                 .set__z(this->imu_orientation_ref.z));
    this->imu_velocity_publisher->publish(geometry_msgs::msg::Vector3()
                                              .set__x(this->imu_velocity_ref.x)
                                              .set__y(this->imu_velocity_ref.y)
                                              .set__z(this->imu_velocity_ref.z));
    this->imu_temperature_publisher->publish(
        std_msgs::msg::Float64().set__data(this->imu_temperature_ref.value));

    return cif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::GateController, controller_interface::ControllerInterface)