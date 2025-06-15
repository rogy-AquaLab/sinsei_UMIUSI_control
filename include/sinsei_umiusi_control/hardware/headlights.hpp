#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_HEADLIGHTS_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_HEADLIGHTS_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "sinsei_umiusi_control/cmd/headlights.hpp"

namespace suc = sinsei_umiusi_control;

namespace sinsei_umiusi_control::hardware {

class Headlights : public hardware_interface::SystemInterface {
  private:
    suc::cmd::headlights::HighBeamEnabled high_beam_enabled;
    suc::cmd::headlights::LowBeamEnabled low_beam_enabled;
    suc::cmd::headlights::IrEnabled ir_enabled;

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(Headlights)

    Headlights() = default;

    auto read(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
    auto write(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
};

}  // namespace sinsei_umiusi_control::hardware

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_HEADLIGHTS_HPP