#include "sinsei_umiusi_control/hardware/thruster_direct/servo_direct.hpp"

namespace suchw = sinsei_umiusi_control::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

auto suchw::thruster_direct::ServoDirect::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/) -> hif::return_type {
    return hif::return_type::OK;
}

auto suchw::thruster_direct::ServoDirect::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) -> hif::return_type {
    return hif::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::hardware::thruster_direct::ServoDirect,
    hardware_interface::SystemInterface)