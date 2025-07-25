#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_IMU_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_IMU_MODEL_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/tl_expected/expected.hpp>

#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"

namespace sinsei_umiusi_control::hardware_model {

class ImuModel {
  private:
    std::unique_ptr<interface::Gpio> gpio;

    // ref: https://github.com/adafruit/Adafruit_BNO055/blob/1b1af09/Adafruit_BNO055.h

    static constexpr uint32_t ADDRESS{0x28};

    static constexpr uint32_t ID{0xA0};

    static constexpr uint32_t CHIP_ID_ADDR{0x00};
    static constexpr uint32_t OPR_MODE_ADDR{0x3D};
    static constexpr uint32_t SYS_TRIGGER_ADDR{0x3F};
    static constexpr uint32_t PWR_MODE_ADDR{0x3E};
    static constexpr uint32_t PAGE_ID_ADDR{0x07};
    static constexpr uint32_t EULER_H_LSB_ADDR{0x1A};

    static constexpr std::byte OPERATION_MODE_CONFIG{0x00};
    static constexpr std::byte OPERATION_MODE_NDOF{0x0C};
    static constexpr std::byte POWER_MODE_NORMAL{0x00};

    static constexpr uint32_t TEMP_ADDR{0x34};

    auto read_orientation() -> tl::expected<state::imu::Orientation, std::string>;

  public:
    ImuModel(std::unique_ptr<interface::Gpio> gpio);
    auto on_init() -> tl::expected<void, std::string>;
    auto on_read() -> tl::expected<
                       std::tuple<
                           sinsei_umiusi_control::state::imu::Orientation,
                           sinsei_umiusi_control::state::imu::Velocity,
                           sinsei_umiusi_control::state::imu::Temperature>,
                       std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_IMU_MODEL_HPP