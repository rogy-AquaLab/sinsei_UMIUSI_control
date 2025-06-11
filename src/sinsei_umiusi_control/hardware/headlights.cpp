#include "sinsei_umiusi_control/hardware/headlights.hpp"

namespace suchw = sinsei_umiusi_control::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

auto suchw::Headlights::on_init(const hif::HardwareInfo & /*info*/) -> hif::CallbackReturn {
    return hif::CallbackReturn::SUCCESS;
}

auto suchw::Headlights::on_export_state_interfaces()
    -> std::vector<hif::StateInterface::ConstSharedPtr> {
    auto interfaces_to_export = std::vector<hif::StateInterface::ConstSharedPtr>{};
    return interfaces_to_export;
}

auto suchw::Headlights::on_export_command_interfaces()
    -> std::vector<hif::CommandInterface::SharedPtr> {
    auto interfaces_to_export = std::vector<hif::CommandInterface::SharedPtr>{};
    return interfaces_to_export;
}

auto suchw::Headlights::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hif::return_type {
    return hif::return_type::OK;
}

auto suchw::Headlights::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    -> hif::return_type {
    return hif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::hardware::Headlights, hardware_interface::ActuatorInterface)