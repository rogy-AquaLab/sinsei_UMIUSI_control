#include "sinsei_umiusi_control/hardware/thruster_direct.hpp"

namespace suchw = sinsei_umiusi_control::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

auto suchw::ThrusterDirect::on_init(const hif::HardwareInfo & /*info*/) -> hif::CallbackReturn {
    return hif::CallbackReturn::SUCCESS;
}

auto suchw::ThrusterDirect::on_export_state_interfaces()
    -> std::vector<hif::StateInterface::ConstSharedPtr> {
    auto interfaces_to_export = std::vector<hif::StateInterface::ConstSharedPtr>{};
    return interfaces_to_export;
}

auto suchw::ThrusterDirect::on_export_command_interfaces()
    -> std::vector<hif::CommandInterface::SharedPtr> {
    auto interfaces_to_export = std::vector<hif::CommandInterface::SharedPtr>{};
    return interfaces_to_export;
}

auto suchw::ThrusterDirect::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hif::return_type {
    return hif::return_type::OK;
}

auto suchw::ThrusterDirect::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) -> hif::return_type {
    return hif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::hardware::ThrusterDirect, hardware_interface::ActuatorInterface)