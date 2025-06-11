#include "sinsei_umiusi_control/hardware/raspi_camera.hpp"

namespace suchw = sinsei_umiusi_control::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

auto suchw::RaspiCamera::on_init(const hif::HardwareInfo & /*info*/) -> hif::CallbackReturn {
    return hif::CallbackReturn::SUCCESS;
}

auto suchw::RaspiCamera::on_export_state_interfaces()
    -> std::vector<hif::StateInterface::ConstSharedPtr> {
    auto interfaces_to_export = std::vector<hif::StateInterface::ConstSharedPtr>{};
    return interfaces_to_export;
}

auto suchw::RaspiCamera::on_export_command_interfaces()
    -> std::vector<hif::CommandInterface::SharedPtr> {
    auto interfaces_to_export = std::vector<hif::CommandInterface::SharedPtr>{};
    return interfaces_to_export;
}

auto suchw::RaspiCamera::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hif::return_type {
    return hif::return_type::OK;
}

auto suchw::RaspiCamera::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    -> hif::return_type {
    return hif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::hardware::RaspiCamera, hardware_interface::ActuatorInterface)