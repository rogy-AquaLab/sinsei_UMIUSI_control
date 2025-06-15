#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_INDICATOR_LED_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_INDICATOR_LED_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "sinsei_umiusi_control/cmd/indicator_led.hpp"

namespace suc = sinsei_umiusi_control;

namespace sinsei_umiusi_control::hardware {

class IndicatorLed : public hardware_interface::SystemInterface {
  private:
    // TODO: 不要なためコメントアウト中。将来的に削除する。
    // suc::cmd::indicator_led::Enabled enabled;

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(IndicatorLed)

    IndicatorLed() = default;

    auto read(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
    auto write(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
};

}  // namespace sinsei_umiusi_control::hardware

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_INDICATOR_LED_HPP