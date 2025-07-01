#include "sinsei_umiusi_control/hardware/headlights.hpp"

namespace suchw = sinsei_umiusi_control::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

auto suchw::Headlights::on_init(const hif::HardwareInfo & info) -> hif::CallbackReturn {
    this->hif::SystemInterface::on_init(info);

    // FIXME: ピン番号はパラメーターなどで設定できるようにする
    this->model.emplace(
        std::make_unique<sinsei_umiusi_control::util::Pigpio>(5),  // High Beam
        std::make_unique<sinsei_umiusi_control::util::Pigpio>(6),  // Low Beam
        std::make_unique<sinsei_umiusi_control::util::Pigpio>(25)  // IR
    );

    return hif::CallbackReturn::SUCCESS;
}

auto suchw::Headlights::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hif::return_type {
    if (this->model.has_value()) this->model->on_read();
    return hif::return_type::OK;
}

auto suchw::Headlights::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    -> hif::return_type {
    double high_beam_enabled_raw = get_command("headlights/high_beam_enabled");
    double low_beam_enabled_raw = get_command("headlights/low_beam_enabled");
    double ir_enabled_raw = get_command("headlights/ir_enabled");
    auto high_beam_enabled =
        *reinterpret_cast<sinsei_umiusi_control::cmd::headlights::HighBeamEnabled *>(
            &high_beam_enabled_raw);
    auto low_beam_enabled =
        *reinterpret_cast<sinsei_umiusi_control::cmd::headlights::LowBeamEnabled *>(
            &low_beam_enabled_raw);
    auto ir_enabled =
        *reinterpret_cast<sinsei_umiusi_control::cmd::headlights::IrEnabled *>(&ir_enabled_raw);
    if (this->model.has_value()) {
        this->model->on_write(high_beam_enabled, low_beam_enabled, ir_enabled);
    }

    return hif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::hardware::Headlights, hardware_interface::SystemInterface)