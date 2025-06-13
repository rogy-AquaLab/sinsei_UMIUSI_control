#include "sinsei_umiusi_control/hardware/imu.hpp"

namespace suchw = sinsei_umiusi_control::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

auto suchw::Imu::on_init(const hif::HardwareInfo & /*info*/) -> hif::CallbackReturn {
    return hif::CallbackReturn::SUCCESS;
}

auto suchw::Imu::on_export_state_interfaces() -> std::vector<hif::StateInterface::ConstSharedPtr> {
    auto interfaces_to_export = std::vector<hif::StateInterface::ConstSharedPtr>{};
    return interfaces_to_export;
}

auto suchw::Imu::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hif::return_type {
    return hif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sinsei_umiusi_control::hardware::Imu, hardware_interface::SensorInterface)