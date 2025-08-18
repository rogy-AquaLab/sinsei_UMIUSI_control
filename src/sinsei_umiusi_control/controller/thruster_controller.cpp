#include "sinsei_umiusi_control/controller/thruster_controller.hpp"

#include <hardware_interface/handle.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>

#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "sinsei_umiusi_control/util/interface_accessor.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"
#include "sinsei_umiusi_control/util/thruster_mode.hpp"

using namespace sinsei_umiusi_control::controller;

auto ThrusterController::command_interface_configuration() const
    -> controller_interface::InterfaceConfiguration {
    auto cmd_names = std::vector<std::string>{};
    for (const auto & [name, _] : this->command_interface_data) {
        cmd_names.push_back(name);
    }

    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::INDIVIDUAL,
        cmd_names,
    };
}

auto ThrusterController::state_interface_configuration() const
    -> controller_interface::InterfaceConfiguration {
    auto state_names = std::vector<std::string>{};
    for (const auto & [name, _] : this->state_interface_data) {
        state_names.push_back(name);
    }

    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::INDIVIDUAL,
        state_names,
    };
}

auto ThrusterController::on_init() -> controller_interface::CallbackReturn {
    using rcl_interfaces::msg::FloatingPointRange;
    using rcl_interfaces::msg::IntegerRange;
    using rcl_interfaces::msg::ParameterDescriptor;

    this->get_node()->declare_parameter(
        "thruster_mode", "unknown",
        ParameterDescriptor{}
            .set__description("Mode of the thruster (can: CAN mode / direct: Direct mode)")
            .set__type(rclcpp::PARAMETER_STRING)
            .set__read_only(true)
            .set__additional_constraints("Must be one of `can` or `direct`"));
    this->get_node()->declare_parameter(
        "id", 1,
        ParameterDescriptor{}
            .set__description(
                "ID of the thruster hardware component (`N` for `thrusterN` in the URDF)")
            .set__type(rclcpp::PARAMETER_INTEGER)
            .set__integer_range({IntegerRange{}.set__from_value(1).set__to_value(4)})
            .set__read_only(true));
    this->get_node()->declare_parameter(
        "is_forward", true,
        ParameterDescriptor{}
            .set__description("Thruster direction (true for forward, false for reverse)")
            .set__type(rclcpp::PARAMETER_BOOL));
    this->get_node()->declare_parameter(
        "max_duty", 0.0,
        ParameterDescriptor{}
            .set__description("Maximum duty cycle (0.0 to 1.0)")
            .set__type(rclcpp::PARAMETER_DOUBLE)
            .set__floating_point_range(
                {FloatingPointRange{}.set__from_value(0.0).set__to_value(1.0)}));

    this->input = Input{};
    this->output = Output{};

    return controller_interface::CallbackReturn::SUCCESS;
}

auto ThrusterController::on_configure(const rclcpp_lifecycle::State & /*pervious_state*/)
    -> controller_interface::CallbackReturn {
    const auto mode_str = this->get_node()->get_parameter("thruster_mode").as_string();
    const auto mode_res = util::get_mode_from_str(mode_str);
    if (!mode_res) {
        RCLCPP_ERROR(this->get_node()->get_logger(), "%s", mode_str.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }
    this->mode = mode_res.value();

    this->id = static_cast<uint8_t>(this->get_node()
                                        ->get_parameter("id")
                                        .as_int());  // パラメータで範囲に制約を設けているので安全

    this->is_forward = this->get_node()->get_parameter("is_forward").as_bool();

    this->max_duty = this->get_node()
                         ->get_parameter("max_duty")
                         .as_double();  // パラメータで範囲に制約を設けているので安全

    const auto prefix = this->mode == util::ThrusterMode::Can
                            ? "thruster" + std::to_string(this->id) + "/"
                            : "thruster_direct" + std::to_string(this->id) + "/";

    this->command_interface_data.emplace_back(
        prefix + "esc/enabled", util::to_interface_data_ptr(this->output.cmd.esc_enabled));
    this->command_interface_data.emplace_back(
        prefix + "servo/enabled", util::to_interface_data_ptr(this->output.cmd.servo_enabled));
    this->command_interface_data.emplace_back(
        prefix + "esc/duty_cycle", util::to_interface_data_ptr(this->output.cmd.duty_cycle));
    this->command_interface_data.emplace_back(
        prefix + "servo/angle", util::to_interface_data_ptr(this->output.cmd.angle));

    if (this->mode == util::ThrusterMode::Can) {
        this->state_interface_data.emplace_back(
            prefix + "esc/rpm", util::to_interface_data_ptr(this->input.state.rpm));
    }

    this->ref_interface_data.emplace_back(
        "esc_enabled", util::to_interface_data_ptr(this->input.cmd.esc_enabled));
    this->ref_interface_data.emplace_back(
        "servo_enabled", util::to_interface_data_ptr(this->input.cmd.servo_enabled));
    this->ref_interface_data.emplace_back(
        "duty_cycle", util::to_interface_data_ptr(this->input.cmd.duty_cycle));
    this->ref_interface_data.emplace_back(
        "angle", util::to_interface_data_ptr(this->input.cmd.angle));

    {
        const auto prefix = "cmd/direct/" + this->get_name() + "/";
        const auto qos = rclcpp::SystemDefaultsQoS();
        this->input.sub.esc_enabled = this->get_node()->create_subscription<std_msgs::msg::Bool>(
            prefix + "esc_enabled", qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->output.state.esc_enabled.value = msg->data;
            });
        this->input.sub.servo_enabled = this->get_node()->create_subscription<std_msgs::msg::Bool>(
            prefix + "servo_enabled", qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->output.state.servo_enabled.value = msg->data;
            });
        this->input.sub.duty_cycle = this->get_node()->create_subscription<std_msgs::msg::Float64>(
            prefix + "duty_cycle", qos, [this](const std_msgs::msg::Float64::SharedPtr msg) {
                const auto sgn = this->is_forward ? 1.0 : -1.0;
                const auto resized = this->max_duty * msg->data;
                this->output.state.duty_cycle.value = sgn * resized;
            });
        this->input.sub.angle = this->get_node()->create_subscription<std_msgs::msg::Float64>(
            prefix + "angle", qos, [this](const std_msgs::msg::Float64::SharedPtr msg) {
                this->output.state.angle.value = msg->data;
            });
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

auto ThrusterController::on_export_reference_interfaces()
    -> std::vector<hardware_interface::CommandInterface> {
    // To avoid bug in ros2 control. `reference_interfaces_` is actually not used.
    this->reference_interfaces_.resize(this->ref_interface_data.size());

    auto interfaces = std::vector<hardware_interface::CommandInterface>{};
    for (auto & [name, data] : this->ref_interface_data) {
        interfaces.emplace_back(
            hardware_interface::CommandInterface(this->get_node()->get_name(), name, data));
    }
    return interfaces;
}

auto ThrusterController::on_export_state_interfaces()
    -> std::vector<hardware_interface::StateInterface> {
    auto interfaces = std::vector<hardware_interface::StateInterface>{};
    for (auto & [name, data] : this->state_interface_data) {
        // Thruster ID を隠蔽する (e.g. thruster1/rpm -> thruster/rpm)
        constexpr auto OFFSET = std::size("thrusterN") - 1;  // 末尾のnull文字を除くため、-1
        interfaces.emplace_back(hardware_interface::StateInterface(
            this->get_node()->get_name(), "thruster" + name.substr(OFFSET), data));
    }
    interfaces.emplace_back(hardware_interface::StateInterface(
        this->get_node()->get_name(), "thruster/esc_enabled",
        util::to_interface_data_ptr(this->output.state.esc_enabled)));
    interfaces.emplace_back(hardware_interface::StateInterface(
        this->get_node()->get_name(), "thruster/servo_enabled",
        util::to_interface_data_ptr(this->output.state.servo_enabled)));
    interfaces.emplace_back(hardware_interface::StateInterface(
        this->get_node()->get_name(), "thruster/duty_cycle",
        util::to_interface_data_ptr(this->output.state.duty_cycle)));
    interfaces.emplace_back(hardware_interface::StateInterface(
        this->get_node()->get_name(), "thruster/angle",
        util::to_interface_data_ptr(this->output.state.angle)));

    return interfaces;
}

auto ThrusterController::on_set_chained_mode(bool /*chained_mode*/) -> bool { return true; }

auto ThrusterController::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/) -> controller_interface::return_type {
    return controller_interface::return_type::OK;
}

auto ThrusterController::update_and_write_commands(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/) -> controller_interface::return_type {
    // パラメータを取得
    this->is_forward = this->get_node()->get_parameter("is_forward").as_bool();

    this->max_duty = this->get_node()
                         ->get_parameter("max_duty")
                         .as_double();  // パラメータで範囲に制約を設けているので安全

    // 状態を取得
    auto res = util::interface_accessor::get_states_from_loaned_interfaces(
        this->state_interfaces_, this->state_interface_data);
    if (!res) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_node()->get_logger(), *this->get_node()->get_clock(), DURATION,
            "Failed to get value of state interfaces");
    }

    // TODO: logicクラスを実装する
    if (this->input.sub.esc_enabled->get_publisher_count() == 0) {
        this->output.state.esc_enabled.value = this->input.cmd.esc_enabled.value;
    }
    if (this->input.sub.servo_enabled->get_publisher_count() == 0) {
        this->output.state.servo_enabled.value = this->input.cmd.servo_enabled.value;
    }
    if (this->input.sub.duty_cycle->get_publisher_count() == 0) {
        const auto sgn = this->is_forward ? 1.0 : -1.0;
        const auto resized = this->max_duty * this->input.cmd.duty_cycle.value;
        this->output.state.duty_cycle.value = sgn * resized;
    }
    if (this->input.sub.angle->get_publisher_count() == 0) {
        this->output.state.angle.value = this->input.cmd.angle.value;
    }

    this->output.cmd.esc_enabled.value = this->output.state.esc_enabled.value;
    this->output.cmd.servo_enabled.value = this->output.state.servo_enabled.value;
    this->output.cmd.duty_cycle.value = this->output.state.duty_cycle.value;
    this->output.cmd.angle.value = this->output.state.angle.value;

    // コマンドを送信
    res = util::interface_accessor::set_commands_to_loaned_interfaces(
        this->command_interfaces_, this->command_interface_data);
    if (!res) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_node()->get_logger(), *this->get_node()->get_clock(), DURATION,
            "Failed to set value of command interfaces");
    }
    return controller_interface::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::ThrusterController,
    controller_interface::ChainableControllerInterface)