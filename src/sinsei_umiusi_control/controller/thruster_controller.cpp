#include "sinsei_umiusi_control/controller/thruster_controller.hpp"

namespace succ = sinsei_umiusi_control::controller;
namespace rlc = rclcpp_lifecycle;
namespace hif = hardware_interface;
namespace cif = controller_interface;

auto succ::ThrusterController::command_interface_configuration() const
    -> cif::InterfaceConfiguration {
    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        this->cmd_interface_names,
    };
}

auto succ::ThrusterController::state_interface_configuration() const
    -> cif::InterfaceConfiguration {
    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        this->state_interface_names,
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

    return cif::CallbackReturn::SUCCESS;
}

auto succ::ThrusterController::on_configure(const rlc::State & /*pervious_state*/)
    -> cif::CallbackReturn {
    std::string id_str = std::to_string(this->id);

    if (this->mode == ThrusterMode::Can) {
        this->cmd_interface_names = {
            "thruster" + id_str + "/servo/servo/enabled_raw",
            "thruster" + id_str + "/servo/servo/angle_raw",
            "thruster" + id_str + "/esc/esc/enabled_raw",
            "thruster" + id_str + "/esc/esc/thrust_raw",
        };
        this->state_interface_names = {
            "thruster" + id_str + "/servo/servo/servo_current_raw",
            "thruster" + id_str + "/esc/esc/rpm_raw",
        };
    } else if (this->mode == ThrusterMode::Direct) {
        this->cmd_interface_names = {
            "thruster_direct" + id_str + "/servo_direct/servo_direct/enabled_raw",
            "thruster_direct" + id_str + "/servo_direct/servo_direct/angle_raw",
            "thruster_direct" + id_str + "/esc_direct/esc_direct/enabled_raw",
            "thruster_direct" + id_str + "/esc_direct/esc_direct/thrust_raw",
        };
    }
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
    return cif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::ThrusterController,
    controller_interface::ChainableControllerInterface)