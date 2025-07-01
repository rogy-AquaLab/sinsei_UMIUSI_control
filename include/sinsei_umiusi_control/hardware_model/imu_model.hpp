#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_IMU_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_IMU_MODEL_HPP

#include <cstdint>
#include <memory>
#include <optional>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rcpputils/tl_expected/expected.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"
#include "sinsei_umiusi_control/util/gpio.hpp"

namespace suc = sinsei_umiusi_control;

namespace sinsei_umiusi_control::hardware_model {

class ImuModel {
  private:
    std::unique_ptr<util::Gpio> gpio;

    // ref: https://github.com/adafruit/Adafruit_BNO055/blob/1b1af09/Adafruit_BNO055.h

    static constexpr uint8_t ADDRESS = 0x28;

    static constexpr uint8_t ID = 0xA0;

    static constexpr uint8_t CHIP_ID_ADDR = 0x00;
    static constexpr uint8_t OPR_MODE_ADDR = 0x3D;
    static constexpr uint8_t SYS_TRIGGER_ADDR = 0x3F;
    static constexpr uint8_t PWR_MODE_ADDR = 0x3E;
    static constexpr uint8_t PAGE_ID_ADDR = 0x07;
    static constexpr uint8_t EULER_H_LSB_ADDR = 0x1A;

    static constexpr uint8_t OPERATION_MODE_CONFIG = 0x00;
    static constexpr uint8_t OPERATION_MODE_NDOF = 0X0C;
    static constexpr uint8_t POWER_MODE_NORMAL = 0x00;

    static constexpr uint8_t TEMP_ADDR = 0X34;

    bool read_orientation(state::imu::Orientation & orientation);
    void write_reg(uint8_t reg, uint8_t value);
    std::optional<uint8_t> read_reg(uint8_t reg);

  public:
    ImuModel(std::unique_ptr<util::Gpio> gpio);
    auto begin() -> tl::expected<void, std::string>;
    auto on_read() -> sinsei_umiusi_control::state::imu::ImuState;
};

}  // namespace sinsei_umiusi_control::hardware_model

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_IMU_MODEL_HPP