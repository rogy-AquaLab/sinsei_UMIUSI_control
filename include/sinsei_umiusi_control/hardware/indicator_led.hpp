#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_INDICATOR_LED_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_INDICATOR_LED_HPP

#include <hardware_interface/system_interface.hpp>
#include <memory>
#include <rclcpp/macros.hpp>

#include "sinsei_umiusi_control/hardware_model/indicator_led_model.hpp"
#include "sinsei_umiusi_control/util/pigpio.hpp"

namespace sinsei_umiusi_control::hardware {

class IndicatorLed : public hardware_interface::SystemInterface {
  private:
    std::optional<hardware_model::IndicatorLedModel> model;

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(IndicatorLed)

    IndicatorLed() = default;

    auto on_init(const hardware_interface::HardwareInfo & info)
        -> hardware_interface::CallbackReturn override;
    auto read(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
    auto write(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
};

}  // namespace sinsei_umiusi_control::hardware

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_INDICATOR_LED_HPP