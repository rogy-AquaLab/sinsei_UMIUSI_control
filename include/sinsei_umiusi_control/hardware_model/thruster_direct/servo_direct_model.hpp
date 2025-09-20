#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_THRUSTER_DIRECT_SERVO_DIRECT_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_THRUSTER_DIRECT_SERVO_DIRECT_MODEL_HPP

#include <memory>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

#include "sinsei_umiusi_control/cmd/thruster/servo.hpp"
#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"

namespace sinsei_umiusi_control::hardware_model::thruster_direct {

class ServoDirectModel {
  private:
    std::unique_ptr<interface::Gpio> gpio;
    interface::Gpio::Pin servo_pin;

  public:
    ServoDirectModel(std::unique_ptr<interface::Gpio> gpio, interface::Gpio::Pin servo_pin);
    auto on_init() -> tl::expected<void, std::string>;
    auto on_read() const -> tl::expected<void, std::string>;
    auto on_write(
        sinsei_umiusi_control::cmd::thruster::servo::Enabled && enabled,
        sinsei_umiusi_control::cmd::thruster::servo::Angle && angle)
        -> tl::expected<void, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model::thruster_direct

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_THRUSTER_DIRECT_SERVO_DIRECT_MODEL_HPP
