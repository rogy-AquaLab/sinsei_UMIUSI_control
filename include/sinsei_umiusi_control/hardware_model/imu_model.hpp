#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_IMU_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_IMU_MODEL_HPP

#include <memory>
#include <rcpputils/tl_expected/expected.hpp>

#include "sinsei_umiusi_control/hardware_model/imu/bno055_model.hpp"
#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"

namespace sinsei_umiusi_control::hardware_model {

class ImuModel {
  private:
    imu::Bno055Model bno055_model;

  public:
    ImuModel(std::unique_ptr<interface::Gpio> gpio);
    auto on_init() -> tl::expected<void, std::string>;
    auto on_read() -> tl::expected<
                       std::tuple<
                           sinsei_umiusi_control::state::imu::Quaternion,
                           sinsei_umiusi_control::state::imu::Acceleration,
                           sinsei_umiusi_control::state::imu::AngularVelocity,
                           sinsei_umiusi_control::state::imu::Temperature>,
                       std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_IMU_MODEL_HPP
