#include "sinsei_umiusi_control/hardware/can.hpp"

#include "sinsei_umiusi_control/util/pican.hpp"

namespace suchw = sinsei_umiusi_control::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

auto suchw::Can::on_init(const hif::HardwareInfo & info) -> hif::CallbackReturn {
    this->hif::SystemInterface::on_init(info);

    this->model.emplace(std::make_unique<sinsei_umiusi_control::util::Pican>());

    return hif::CallbackReturn::SUCCESS;
}

auto suchw::Can::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hif::return_type {
    return hif::return_type::OK;
}

auto suchw::Can::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    -> hif::return_type {
    return hif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sinsei_umiusi_control::hardware::Can, hardware_interface::SystemInterface)