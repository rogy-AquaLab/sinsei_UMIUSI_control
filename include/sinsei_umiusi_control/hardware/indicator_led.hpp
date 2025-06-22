#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_INDICATOR_LED_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_INDICATOR_LED_HPP

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "sinsei_umiusi_control/cmd/indicator_led.hpp"

namespace suc = sinsei_umiusi_control;

namespace sinsei_umiusi_control::hardware {

class IndicatorLed : public hardware_interface::SystemInterface {
  private:
    static constexpr int INDICATOR_LED_GPIO = 24;

    bool gpio_initialized = false;

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