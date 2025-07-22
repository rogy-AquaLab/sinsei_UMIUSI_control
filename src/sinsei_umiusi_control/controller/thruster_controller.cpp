#include "sinsei_umiusi_control/controller/thruster_controller.hpp"

#include <rclcpp/logging.hpp>

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
    this->get_node()->declare_parameter("id", 1);
    this->get_node()->declare_parameter("thruster_mode", "unknown");

    this->servo_enabled = sinsei_umiusi_control::cmd::thruster::ServoEnabled{};
    this->esc_enabled = sinsei_umiusi_control::cmd::thruster::EscEnabled{};
    this->angle = sinsei_umiusi_control::cmd::thruster::Angle{};
    this->duty_cycle = sinsei_umiusi_control::cmd::thruster::DutyCycle{};

    this->rpm = sinsei_umiusi_control::state::thruster::Rpm{};

    return cif::CallbackReturn::SUCCESS;
}

auto succ::ThrusterController::on_configure(const rlc::State & /*pervious_state*/)
    -> cif::CallbackReturn {
    this->id = this->get_node()->get_parameter("id").as_int();

    const auto mode_str = this->get_node()->get_parameter("thruster_mode").as_string();
    const auto mode_res = util::get_mode_from_str(mode_str);
    if (!mode_res) {
        RCLCPP_ERROR(this->get_node()->get_logger(), "Invalid thruster mode: %s", mode_str.c_str());
        return cif::CallbackReturn::ERROR;
    }
    this->mode = mode_res.value();

    const auto prefix = this->mode == util::ThrusterMode::Can
                            ? "thruster" + std::to_string(this->id) + "/"
                            : "thruster_direct" + std::to_string(this->id) + "/";

    this->command_interface_data.emplace_back(
        prefix + "esc/enabled", util::to_interface_data_ptr(this->esc_enabled));
    this->command_interface_data.emplace_back(
        prefix + "esc/duty_cycle", util::to_interface_data_ptr(this->duty_cycle));
    this->command_interface_data.emplace_back(
        prefix + "servo/enabled", util::to_interface_data_ptr(this->servo_enabled));
    this->command_interface_data.emplace_back(
        prefix + "servo/angle", util::to_interface_data_ptr(this->angle));

    if (this->mode == util::ThrusterMode::Can) {
        this->state_interface_data.emplace_back(
            prefix + "esc/rpm", util::to_interface_data_ptr(this->rpm));
    }

    this->ref_interface_data.emplace_back(
        "servo_enabled", util::to_interface_data_ptr(this->servo_enabled));
    this->ref_interface_data.emplace_back(
        "esc_enabled", util::to_interface_data_ptr(this->esc_enabled));
    this->ref_interface_data.emplace_back("angle", util::to_interface_data_ptr(this->angle));
    this->ref_interface_data.emplace_back(
        "duty_cycle", util::to_interface_data_ptr(this->duty_cycle));

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
        RCLCPP_INFO(
            this->get_node()->get_logger(), "Exported state interface: %s",
            ("thruster" + name.substr(OFFSET)).c_str());
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
    // 状態を取得
    auto res = util::interface_accessor::get_states_from_loaned_interfaces(
        this->state_interfaces_, this->state_interface_data);
    if (!res) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_node()->get_logger(), *this->get_node()->get_clock(), DURATION,
            "Failed to get value of state interfaces");
    }

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