#include "sinsei_umiusi_control/controller/thruster_controller.hpp"

#include "sinsei_umiusi_control/util/constexpr.hpp"
#include "sinsei_umiusi_control/util/thruster_mode.hpp"

namespace succ = sinsei_umiusi_control::controller;
namespace suc_util = sinsei_umiusi_control::util;
namespace rlc = rclcpp_lifecycle;
namespace hif = hardware_interface;
namespace cif = controller_interface;

auto succ::ThrusterController::command_interface_configuration() const
    -> cif::InterfaceConfiguration {
    std::vector<std::string> cmd_names;
    if (this->mode == util::ThrusterMode::Can) {
        cmd_names.assign(
            std::begin(this->CAN_CMD_INTERFACE_NAMES), std::end(this->CAN_CMD_INTERFACE_NAMES));
    } else {
        cmd_names.assign(
            std::begin(this->DIRECT_CMD_INTERFACE_NAMES),
            std::end(this->DIRECT_CMD_INTERFACE_NAMES));
    }

    // リストではスラスタ名が省略されているため、ここで付与する
    std::string prefix = (this->mode == util::ThrusterMode::Can ? "thruster" : "thruster_direct") +
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
    if (this->mode == util::ThrusterMode::Can) {
        state_names.assign(
            std::begin(this->CAN_STATE_INTERFACE_NAMES), std::end(this->CAN_STATE_INTERFACE_NAMES));
    } else {
        state_names = {};
    }

    // リストではスラスタ名が省略されているため、ここで付与する
    std::string prefix = (this->mode == util::ThrusterMode::Can ? "thruster" : "thruster_direct") +
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
    auto mode_res = util::get_mode_from_str(mode_str);
    if (mode_res) {
        this->mode = mode_res.value();
    } else {
        RCLCPP_ERROR(this->get_node()->get_logger(), "Invalid thruster mode: %s", mode_str.c_str());
        return cif::CallbackReturn::ERROR;
    }

    this->can_interface_helper =
        std::make_unique<InterfaceAccessHelper<CAN_CMD_SIZE, CAN_STATE_SIZE>>(
            this->get_node().get(), this->command_interfaces_, this->CAN_CMD_INTERFACE_NAMES,
            this->state_interfaces_, this->CAN_STATE_INTERFACE_NAMES);
    this->direct_interface_helper =
        std::make_unique<InterfaceAccessHelper<DIRECT_CMD_SIZE, DIRECT_STATE_SIZE>>(
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
        this->get_node()->get_name(), "servo_enabled",
        reinterpret_cast<double *>(&this->servo_enabled)));
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name(), "esc_enabled",
        reinterpret_cast<double *>(&this->esc_enabled)));
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name(), "angle", reinterpret_cast<double *>(&this->angle)));
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name(), "thrust", reinterpret_cast<double *>(&this->thrust)));
    return interfaces;
}

auto succ::ThrusterController::on_export_state_interfaces() -> std::vector<hif::StateInterface> {
    auto interfaces = std::vector<hif::StateInterface>{};

    interfaces.emplace_back(hif::StateInterface(
        this->get_node()->get_name(), "servo_current",
        reinterpret_cast<double *>(&this->servo_current)));
    interfaces.emplace_back(hif::StateInterface(
        this->get_node()->get_name(), "rpm", reinterpret_cast<double *>(&this->rpm)));
    return interfaces;
}

auto succ::ThrusterController::on_set_chained_mode(bool /*chained_mode*/) -> bool { return true; }

auto succ::ThrusterController::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) -> cif::return_type {
    return cif::return_type::OK;
}

auto succ::ThrusterController::update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) -> cif::return_type {
    if (this->mode == util::ThrusterMode::Can) {
        constexpr auto SERVO_ENABLED_INDEX =
            suc_util::get_index("servo/enabled_raw", CAN_CMD_INTERFACE_NAMES);
        constexpr auto ANGLE_INDEX =
            suc_util::get_index("servo/angle_raw", CAN_CMD_INTERFACE_NAMES);
        constexpr auto ESC_ENABLED_INDEX =
            suc_util::get_index("esc/enabled_raw", CAN_CMD_INTERFACE_NAMES);
        constexpr auto THRUST_INDEX =
            suc_util::get_index("esc/thrust_raw", CAN_CMD_INTERFACE_NAMES);

        constexpr auto SERVO_CURRENT_INDEX =
            suc_util::get_index("servo/servo_current_raw", CAN_STATE_INTERFACE_NAMES);
        constexpr auto RPM_INDEX = suc_util::get_index("esc/rpm_raw", CAN_STATE_INTERFACE_NAMES);

        this->can_interface_helper->set_cmd_value(SERVO_ENABLED_INDEX, this->servo_enabled);
        this->can_interface_helper->set_cmd_value(ANGLE_INDEX, this->angle);
        this->can_interface_helper->set_cmd_value(ESC_ENABLED_INDEX, this->esc_enabled);
        this->can_interface_helper->set_cmd_value(THRUST_INDEX, this->thrust);

        this->can_interface_helper->get_state_value(SERVO_CURRENT_INDEX, this->servo_current);
        this->can_interface_helper->get_state_value(RPM_INDEX, this->rpm);

    } else {
        constexpr auto SERVO_ENABLED_INDEX =
            suc_util::get_index("servo_direct/enabled_raw", DIRECT_CMD_INTERFACE_NAMES);
        constexpr auto ANGLE_INDEX =
            suc_util::get_index("servo_direct/angle_raw", DIRECT_CMD_INTERFACE_NAMES);
        constexpr auto ESC_ENABLED_INDEX =
            suc_util::get_index("esc_direct/enabled_raw", DIRECT_CMD_INTERFACE_NAMES);
        constexpr auto THRUST_INDEX =
            suc_util::get_index("esc_direct/thrust_raw", DIRECT_CMD_INTERFACE_NAMES);

        this->direct_interface_helper->set_cmd_value(SERVO_ENABLED_INDEX, this->servo_enabled);
        this->direct_interface_helper->set_cmd_value(ANGLE_INDEX, this->angle);
        this->direct_interface_helper->set_cmd_value(ESC_ENABLED_INDEX, this->esc_enabled);
        this->direct_interface_helper->set_cmd_value(THRUST_INDEX, this->thrust);
    }

    return cif::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::ThrusterController,
    controller_interface::ChainableControllerInterface)