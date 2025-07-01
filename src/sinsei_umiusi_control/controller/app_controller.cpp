#include "sinsei_umiusi_control/controller/app_controller.hpp"

#include "sinsei_umiusi_control/util/constexpr.hpp"

namespace succ = sinsei_umiusi_control::controller;
namespace suc_util = sinsei_umiusi_control::util;
namespace rlc = rclcpp_lifecycle;
namespace hif = hardware_interface;
namespace cif = controller_interface;

auto succ::AppController::command_interface_configuration() const -> cif::InterfaceConfiguration {
    std::vector<std::string> cmd_names(
        std::begin(this->CMD_INTERFACE_NAMES), std::end(this->CMD_INTERFACE_NAMES));

    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        cmd_names,
    };
}

auto succ::AppController::state_interface_configuration() const -> cif::InterfaceConfiguration {
    std::vector<std::string> state_names(
        std::begin(this->STATE_INTERFACE_NAMES), std::end(this->STATE_INTERFACE_NAMES));

    return cif::InterfaceConfiguration{
        cif::interface_configuration_type::INDIVIDUAL,
        state_names,
    };
}

auto succ::AppController::on_init() -> cif::CallbackReturn {
    this->interface_helper = std::make_unique<InterfaceAccessHelper<CMD_SIZE, STATE_SIZE>>(
        this->get_node().get(), this->command_interfaces_, this->CMD_INTERFACE_NAMES,
        this->state_interfaces_, this->STATE_INTERFACE_NAMES);

    return cif::CallbackReturn::SUCCESS;
}

auto succ::AppController::on_configure(const rlc::State & /*pervious_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto succ::AppController::on_activate(const rlc::State & /*previous_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto succ::AppController::on_deactivate(const rlc::State & /*previous_state*/)
    -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto succ::AppController::on_export_reference_interfaces() -> std::vector<hif::CommandInterface> {
    this->reference_interfaces_.resize(6);

    auto interfaces = std::vector<hif::CommandInterface>{};
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name(), "target_orientation.x", &this->target_orientation.x));
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name(), "target_orientation.y", &this->target_orientation.y));
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name(), "target_orientation.z", &this->target_orientation.z));
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name(), "target_velocity.x", &this->target_velocity.x));
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name(), "target_velocity.y", &this->target_velocity.y));
    interfaces.emplace_back(hif::CommandInterface(
        this->get_node()->get_name(), "target_velocity.z", &this->target_velocity.z));
    return interfaces;
}

auto succ::AppController::on_export_state_interfaces() -> std::vector<hif::StateInterface> {
    auto interfaces = std::vector<hif::StateInterface>{};
    interfaces.emplace_back(hif::StateInterface(
        this->get_node()->get_name(), "imu_orientation.x", &this->imu_orientation.x));
    interfaces.emplace_back(hif::StateInterface(
        this->get_node()->get_name(), "imu_orientation.y", &this->imu_orientation.y));
    interfaces.emplace_back(hif::StateInterface(
        this->get_node()->get_name(), "imu_orientation.z", &this->imu_orientation.z));
    interfaces.emplace_back(
        hif::StateInterface(this->get_node()->get_name(), "imu_velocity.x", &this->imu_velocity.x));
    interfaces.emplace_back(
        hif::StateInterface(this->get_node()->get_name(), "imu_velocity.y", &this->imu_velocity.y));
    interfaces.emplace_back(
        hif::StateInterface(this->get_node()->get_name(), "imu_velocity.z", &this->imu_velocity.z));
    return interfaces;
}

auto succ::AppController::on_set_chained_mode(bool /*chained_mode*/) -> bool { return true; };

auto succ::AppController::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) -> cif::return_type {
    return cif::return_type::OK;
}

auto succ::AppController::update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) -> cif::return_type {
    // 姿勢制御の関数を呼び出す
    this->compute_outputs();

    constexpr auto ANGLE1_INDEX =
        suc_util::get_index("thruster_controller1/angle", CMD_INTERFACE_NAMES);
    constexpr auto THRUST1_INDEX =
        suc_util::get_index("thruster_controller1/thrust", CMD_INTERFACE_NAMES);
    constexpr auto ANGLE2_INDEX =
        suc_util::get_index("thruster_controller2/angle", CMD_INTERFACE_NAMES);
    constexpr auto THRUST2_INDEX =
        suc_util::get_index("thruster_controller2/thrust", CMD_INTERFACE_NAMES);
    constexpr auto ANGLE3_INDEX =
        suc_util::get_index("thruster_controller3/angle", CMD_INTERFACE_NAMES);
    constexpr auto THRUST3_INDEX =
        suc_util::get_index("thruster_controller3/thrust", CMD_INTERFACE_NAMES);
    constexpr auto ANGLE4_INDEX =
        suc_util::get_index("thruster_controller4/angle", CMD_INTERFACE_NAMES);
    constexpr auto THRUST4_INDEX =
        suc_util::get_index("thruster_controller4/thrust", CMD_INTERFACE_NAMES);

    constexpr auto ORIENTATION_X_INDEX =
        suc_util::get_index("imu/orientation_raw.x", STATE_INTERFACE_NAMES);
    constexpr auto ORIENTATION_Y_INDEX =
        suc_util::get_index("imu/orientation_raw.y", STATE_INTERFACE_NAMES);
    constexpr auto ORIENTATION_Z_INDEX =
        suc_util::get_index("imu/orientation_raw.z", STATE_INTERFACE_NAMES);
    constexpr auto VELOCITY_X_INDEX =
        suc_util::get_index("imu/velocity_raw.x", STATE_INTERFACE_NAMES);
    constexpr auto VELOCITY_Y_INDEX =
        suc_util::get_index("imu/velocity_raw.y", STATE_INTERFACE_NAMES);
    constexpr auto VELOCITY_Z_INDEX =
        suc_util::get_index("imu/velocity_raw.z", STATE_INTERFACE_NAMES);

    this->interface_helper->set_cmd_value(ANGLE1_INDEX, this->thruster_angles[0]);
    this->interface_helper->set_cmd_value(THRUST1_INDEX, this->thruster_thrusts[0]);
    this->interface_helper->set_cmd_value(ANGLE2_INDEX, this->thruster_angles[1]);
    this->interface_helper->set_cmd_value(THRUST2_INDEX, this->thruster_thrusts[1]);
    this->interface_helper->set_cmd_value(ANGLE3_INDEX, this->thruster_angles[2]);
    this->interface_helper->set_cmd_value(THRUST3_INDEX, this->thruster_thrusts[2]);
    this->interface_helper->set_cmd_value(ANGLE4_INDEX, this->thruster_angles[3]);
    this->interface_helper->set_cmd_value(THRUST4_INDEX, this->thruster_thrusts[3]);

    this->interface_helper->get_state_value(ORIENTATION_X_INDEX, this->imu_orientation.x);
    this->interface_helper->get_state_value(ORIENTATION_Y_INDEX, this->imu_orientation.y);
    this->interface_helper->get_state_value(ORIENTATION_Z_INDEX, this->imu_orientation.z);
    this->interface_helper->get_state_value(VELOCITY_X_INDEX, this->imu_velocity.x);
    this->interface_helper->get_state_value(VELOCITY_Y_INDEX, this->imu_velocity.y);
    this->interface_helper->get_state_value(VELOCITY_Z_INDEX, this->imu_velocity.z);

    return cif::return_type::OK;
}

auto succ::AppController::compute_outputs() -> void {
    // TODO: PID制御などの処理はここに記述する
    // 現在はダミー
    for (size_t i = 0; i < 4; ++i) {
        this->thruster_angles[i].value = 0.0;
        this->thruster_thrusts[i].value = 0.0;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::AppController,
    controller_interface::ChainableControllerInterface)