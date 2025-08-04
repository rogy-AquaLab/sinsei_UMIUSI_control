#include "sinsei_umiusi_control/controller/thruster_controller.hpp"

#include <algorithm>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "sinsei_umiusi_control/controller/logic/thruster/direction.hpp"
#include "sinsei_umiusi_control/util/interface_accessor.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"
#include "sinsei_umiusi_control/util/thruster_mode.hpp"

namespace succ = sinsei_umiusi_control::controller;
namespace suc_util = sinsei_umiusi_control::util;
namespace rlc = rclcpp_lifecycle;
namespace hif = hardware_interface;
namespace cif = controller_interface;

auto succ::ThrusterController::command_interface_configuration() const
    -> cif::InterfaceConfiguration {
    auto cmd_names = std::vector<std::string>{};
    for (const auto & [name, _] : this->command_interface_data) {
        cmd_names.push_back(name);
    }

    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        cmd_names,
    };
}

auto succ::ThrusterController::state_interface_configuration() const
    -> cif::InterfaceConfiguration {
    auto state_names = std::vector<std::string>{};
    for (const auto & [name, _] : this->state_interface_data) {
        state_names.push_back(name);
    }

    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        state_names,
    };
}

auto succ::ThrusterController::on_init() -> cif::CallbackReturn {
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
            .set__read_only(true));
    this->get_node()->declare_parameter(
        "direction", "rh",
        ParameterDescriptor{}
            .set__description(
                "Direction of the thruster (rh: right-handed / lh: left-handed) while z-axis is up")
            .set__type(rclcpp::PARAMETER_STRING)
            .set__additional_constraints("Must be one of `rh` or `lh`"));
    this->get_node()->declare_parameter(
        "max_duty", 0.0,
        ParameterDescriptor{}
            .set__description("Maximum duty cycle (0.0 to 1.0)")
            .set__type(rclcpp::PARAMETER_DOUBLE));

    this->input = Input{};
    this->output = Output{};

    return cif::CallbackReturn::SUCCESS;
}

auto succ::ThrusterController::on_configure(const rlc::State & /*pervious_state*/)
    -> cif::CallbackReturn {
    const auto mode_str = this->get_node()->get_parameter("thruster_mode").as_string();
    const auto mode_res = util::get_mode_from_str(mode_str);
    if (!mode_res) {
        RCLCPP_ERROR(this->get_node()->get_logger(), "%s", mode_str.c_str());
        return cif::CallbackReturn::ERROR;
    }
    this->mode = mode_res.value();

    const auto id = this->get_node()->get_parameter("id").as_int();
    if (id < 1 || id > 4) {
        RCLCPP_ERROR(
            this->get_node()->get_logger(), "Invalid thruster ID: %ld (must be between 1 and 4)",
            id);
        return cif::CallbackReturn::ERROR;
    }
    this->id = static_cast<uint8_t>(id);

    const auto direction_str = this->get_node()->get_parameter("direction").as_string();
    const auto direction_res = logic::thruster::get_direction_from_str(direction_str);
    if (!direction_res) {
        RCLCPP_ERROR(this->get_node()->get_logger(), "%s", direction_str.c_str());
        return cif::CallbackReturn::ERROR;
    }
    this->direction = direction_res.value();

    const auto max_duty = this->get_node()->get_parameter("max_duty").as_double();
    if (max_duty < 0.0 || max_duty > 1.0) {
        RCLCPP_ERROR(
            this->get_node()->get_logger(),
            "Invalid max duty cycle: %f (must be between 0.0 and 1.0)", max_duty);
        return cif::CallbackReturn::ERROR;
    }
    this->max_duty = max_duty;

    const auto prefix = this->mode == util::ThrusterMode::Can
                            ? "thruster" + std::to_string(this->id) + "/"
                            : "thruster_direct" + std::to_string(this->id) + "/";

    this->command_interface_data.emplace_back(
        prefix + "esc/enabled", util::to_interface_data_ptr(this->output.cmd.esc_enabled));
    this->command_interface_data.emplace_back(
        prefix + "esc/duty_cycle", util::to_interface_data_ptr(this->output.cmd.duty_cycle));
    this->command_interface_data.emplace_back(
        prefix + "servo/enabled", util::to_interface_data_ptr(this->output.cmd.servo_enabled));
    this->command_interface_data.emplace_back(
        prefix + "servo/angle", util::to_interface_data_ptr(this->output.cmd.angle));

    if (this->mode == util::ThrusterMode::Can) {
        this->state_interface_data.emplace_back(
            prefix + "esc/rpm", util::to_interface_data_ptr(this->input.state.rpm));
    }

    this->ref_interface_data.emplace_back(
        "servo_enabled", util::to_interface_data_ptr(this->input.cmd.servo_enabled));
    this->ref_interface_data.emplace_back(
        "esc_enabled", util::to_interface_data_ptr(this->input.cmd.esc_enabled));
    this->ref_interface_data.emplace_back(
        "angle", util::to_interface_data_ptr(this->input.cmd.angle));
    this->ref_interface_data.emplace_back(
        "duty_cycle", util::to_interface_data_ptr(this->input.cmd.duty_cycle));

    return cif::CallbackReturn::SUCCESS;
}

auto succ::ThrusterController::on_export_reference_interfaces()
    -> std::vector<hif::CommandInterface> {
    // To avoid bug in ros2 control. `reference_interfaces_` is actually not used.
    this->reference_interfaces_.resize(this->ref_interface_data.size());

    auto interfaces = std::vector<hif::CommandInterface>{};
    for (auto & [name, data] : this->ref_interface_data) {
        interfaces.emplace_back(hif::CommandInterface(this->get_node()->get_name(), name, data));
    }
    return interfaces;
}

auto succ::ThrusterController::on_export_state_interfaces() -> std::vector<hif::StateInterface> {
    auto interfaces = std::vector<hif::StateInterface>{};
    for (auto & [name, data] : this->state_interface_data) {
        // Thruster ID を隠蔽する (e.g. thruster1/rpm -> thruster/rpm)
        constexpr auto OFFSET = std::size("thrusterN") - 1;  // 末尾のnull文字を除くため、-1
        interfaces.emplace_back(hif::StateInterface(
            this->get_node()->get_name(), "thruster" + name.substr(OFFSET), data));
    }
    return interfaces;
}

auto succ::ThrusterController::on_set_chained_mode(bool /*chained_mode*/) -> bool { return true; }

auto succ::ThrusterController::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) -> cif::return_type {
    return cif::return_type::OK;
}

auto succ::ThrusterController::update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) -> cif::return_type {
    // パラメータを取得
    const auto direction_str = this->get_node()->get_parameter("direction").as_string();
    const auto direction_res = logic::thruster::get_direction_from_str(direction_str);
    if (direction_res) {
        this->direction = direction_res.value();
    } else {
        RCLCPP_ERROR(this->get_node()->get_logger(), "%s", direction_str.c_str());
    }

    const auto max_duty = this->get_node()->get_parameter("max_duty").as_double();
    if (max_duty >= 0.0 && max_duty <= 1.0) {
        this->max_duty = max_duty;
    } else {
        RCLCPP_ERROR(
            this->get_node()->get_logger(),
            "Invalid max duty cycle: %f (must be between 0.0 and 1.0)", max_duty);
    }

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
    this->output.cmd.servo_enabled = this->input.cmd.servo_enabled;
    this->output.cmd.esc_enabled = this->input.cmd.esc_enabled;
    this->output.cmd.angle = this->input.cmd.angle;
    const auto sgn = this->direction == logic::thruster::Direction::RightHanded ? 1.0 : -1.0;
    const auto clamped =
        std::clamp(this->input.cmd.duty_cycle.value, -this->max_duty, this->max_duty);
    this->output.cmd.duty_cycle.value = sgn * clamped;

    // コマンドを送信
    res = util::interface_accessor::set_commands_to_loaned_interfaces(
        this->command_interfaces_, this->command_interface_data);
    if (!res) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_node()->get_logger(), *this->get_node()->get_clock(), DURATION,
            "Failed to set value of command interfaces");
    }
    return cif::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::ThrusterController,
    controller_interface::ChainableControllerInterface)