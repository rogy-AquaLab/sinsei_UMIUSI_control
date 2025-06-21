#include "sinsei_umiusi_control/controller/thruster_controller.hpp"

#include "sinsei_umiusi_control/util.hpp"

namespace succ = sinsei_umiusi_control::controller;
namespace suc_util = sinsei_umiusi_control::util;
namespace rlc = rclcpp_lifecycle;
namespace hif = hardware_interface;
namespace cif = controller_interface;

auto succ::ThrusterController::command_interface_configuration() const
    -> cif::InterfaceConfiguration {
    std::vector<std::string> cmd_names;
    if (this->mode == ThrusterMode::Can) {
        cmd_names.assign(
            std::begin(this->CAN_CMD_INTERFACE_NAMES), std::end(this->CAN_CMD_INTERFACE_NAMES));
    } else {
        cmd_names.assign(
            std::begin(this->DIRECT_CMD_INTERFACE_NAMES),
            std::end(this->DIRECT_CMD_INTERFACE_NAMES));
    }

    // リストではスラスタ名が省略されているため、ここで付与する
    std::string prefix = (this->mode == ThrusterMode::Can ? "thruster" : "thruster_direct") +
                         std::to_string(this->id);
    for (auto & name : cmd_names) {
        name = prefix + "/" + name;
    }

    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        cmd_names,
    };
}

auto succ::ThrusterController::state_interface_configuration() const
    -> cif::InterfaceConfiguration {
    std::vector<std::string> state_names;
    if (this->mode == ThrusterMode::Can) {
        state_names.assign(
            std::begin(this->CAN_STATE_INTERFACE_NAMES), std::end(this->CAN_STATE_INTERFACE_NAMES));
    } else {
        state_names = {};
    }

    // リストではスラスタ名が省略されているため、ここで付与する
    std::string prefix = (this->mode == ThrusterMode::Can ? "thruster" : "thruster_direct") +
                         std::to_string(this->id);

    for (auto & name : state_names) {
        name = prefix + "/" + name;
    }

    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        state_names,
    };
}

auto succ::ThrusterController::on_init() -> cif::CallbackReturn {
    this->get_node()->declare_parameter("id", 1);
    this->id = this->get_node()->get_parameter("id").as_int();
    RCLCPP_INFO(this->get_node()->get_logger(), "Thruster ID: %d", this->id);

    this->get_node()->declare_parameter("thruster_mode", "can");
    std::string mode_str = this->get_node()->get_parameter("thruster_mode").as_string();
    if (mode_str == "can") {
        this->mode = ThrusterMode::Can;
    } else if (mode_str == "direct") {
        this->mode = ThrusterMode::Direct;
    } else {
        RCLCPP_ERROR(this->get_node()->get_logger(), "Invalid thruster mode: %s", mode_str.c_str());
        return cif::CallbackReturn::ERROR;
    }

    this->can_interface_helper_ = std::make_unique<
        InterfaceAccessHelper<rclcpp_lifecycle::LifecycleNode, CAN_CMD_SIZE, CAN_STATE_SIZE>>(
        this->get_node().get(), this->command_interfaces_, this->CAN_CMD_INTERFACE_NAMES,
        this->state_interfaces_, this->CAN_STATE_INTERFACE_NAMES);
    this->direct_interface_helper_ = std::make_unique<
        InterfaceAccessHelper<rclcpp_lifecycle::LifecycleNode, DIRECT_CMD_SIZE, DIRECT_STATE_SIZE>>(
        this->get_node().get(), this->command_interfaces_, this->DIRECT_CMD_INTERFACE_NAMES,
        this->state_interfaces_, this->DIRECT_STATE_INTERFACE_NAMES);

    return cif::CallbackReturn::SUCCESS;
}

auto succ::ThrusterController::on_configure(const rlc::State & /*pervious_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto succ::ThrusterController::on_activate(const rlc::State & /*previous_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto succ::ThrusterController::on_deactivate(const rlc::State & /*previous_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto succ::ThrusterController::on_export_reference_interfaces()
    -> std::vector<hif::CommandInterface> {
    this->reference_interfaces_.resize(4);

    auto interfaces = std::vector<hif::CommandInterface>{};
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name() + std::string("/servo_enabled"), "servo_enabled",
        reinterpret_cast<double *>(&this->servo_enabled)));
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name() + std::string("/esc_enabled"), "esc_enabled",
        reinterpret_cast<double *>(&this->esc_enabled)));
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name() + std::string("/angle"), "angle",
        reinterpret_cast<double *>(&this->angle)));
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name() + std::string("/thrust"), "thrust",
        reinterpret_cast<double *>(&this->thrust)));
    return interfaces;
}

auto succ::ThrusterController::on_export_state_interfaces() -> std::vector<hif::StateInterface> {
    auto interfaces = std::vector<hif::StateInterface>{};

    interfaces.emplace_back(hif::StateInterface(
        this->get_node()->get_name() + std::string("/servo_current"), "servo_current",
        reinterpret_cast<double *>(&this->servo_current)));
    interfaces.emplace_back(hif::StateInterface(
        this->get_node()->get_name() + std::string("/rpm"), "rpm",
        reinterpret_cast<double *>(&this->rpm)));
    return interfaces;
}

auto succ::ThrusterController::on_set_chained_mode(bool /*chained_mode*/) -> bool { return true; }

auto succ::ThrusterController::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) -> cif::return_type {
    return cif::return_type::OK;
}

auto succ::ThrusterController::update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) -> cif::return_type {
    if (this->mode == ThrusterMode::Can) {
        constexpr auto servo_enabled_index =
            suc_util::get_index("servo/servo/enabled_raw", CAN_CMD_INTERFACE_NAMES);
        constexpr auto angle_index =
            suc_util::get_index("servo/servo/angle_raw", CAN_CMD_INTERFACE_NAMES);
        constexpr auto esc_enabled_index =
            suc_util::get_index("esc/esc/enabled_raw", CAN_CMD_INTERFACE_NAMES);
        constexpr auto thrust_index =
            suc_util::get_index("esc/esc/thrust_raw", CAN_CMD_INTERFACE_NAMES);

        constexpr auto servo_current_index =
            suc_util::get_index("servo/servo/servo_current_raw", CAN_STATE_INTERFACE_NAMES);
        constexpr auto rpm_index =
            suc_util::get_index("esc/esc/rpm_raw", CAN_STATE_INTERFACE_NAMES);

        this->can_interface_helper_->set_cmd_value(servo_enabled_index, this->servo_enabled);
        this->can_interface_helper_->set_cmd_value(angle_index, this->angle);
        this->can_interface_helper_->set_cmd_value(esc_enabled_index, this->esc_enabled);
        this->can_interface_helper_->set_cmd_value(thrust_index, this->thrust);

        this->can_interface_helper_->get_state_value(servo_current_index, this->servo_current);
        this->can_interface_helper_->get_state_value(rpm_index, this->rpm);
    } else {
        constexpr auto servo_enabled_index = suc_util::get_index(
            "servo_direct/servo_direct/enabled_raw", DIRECT_CMD_INTERFACE_NAMES);
        constexpr auto angle_index =
            suc_util::get_index("servo_direct/servo_direct/angle_raw", DIRECT_CMD_INTERFACE_NAMES);
        constexpr auto esc_enabled_index =
            suc_util::get_index("esc_direct/esc_direct/enabled_raw", DIRECT_CMD_INTERFACE_NAMES);
        constexpr auto thrust_index =
            suc_util::get_index("esc_direct/esc_direct/thrust_raw", DIRECT_CMD_INTERFACE_NAMES);

        this->direct_interface_helper_->set_cmd_value(servo_enabled_index, this->servo_enabled);
        this->direct_interface_helper_->set_cmd_value(angle_index, this->angle);
        this->direct_interface_helper_->set_cmd_value(esc_enabled_index, this->esc_enabled);
        this->direct_interface_helper_->set_cmd_value(thrust_index, this->thrust);
    }

    return cif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::ThrusterController,
    controller_interface::ChainableControllerInterface)