#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_IMU_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_IMU_MODEL_HPP

#include <cstdint>
#include <memory>
#include <optional>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sinsei_umiusi_control/state/imu.hpp"
#include "sinsei_umiusi_control/util/gpio.hpp"

namespace suc = sinsei_umiusi_control;

namespace sinsei_umiusi_control::hardware_model {

class ImuModel {
  private:
    std::unique_ptr<util::Gpio> gpio;

    static constexpr uint8_t ADDRESS = 0x28;

    static constexpr uint8_t CHIP_ID_ADDR = 0x00;
    static constexpr uint8_t OPR_MODE_ADDR = 0x3D;
    static constexpr uint8_t SYS_TRIGGER_ADDR = 0x3F;
    static constexpr uint8_t PWR_MODE_ADDR = 0x3E;
    static constexpr uint8_t PAGE_ID_ADDR = 0x07;
    static constexpr uint8_t EULER_H_LSB_ADDR = 0x1A;

    static constexpr uint8_t OPERATION_MODE_CONFIG = 0x00;
    static constexpr uint8_t OPERATION_MODE_IMUPLUS = 0x08;
    static constexpr uint8_t POWER_MODE_NORMAL = 0x00;

    static constexpr uint8_t BNO055_TEMP_ADDR = 0X34;

    bool read_euler(float & heading, float & roll, float & pitch);
    void write_reg(uint8_t reg, uint8_t value);
    std::optional<uint8_t> read_reg(uint8_t reg);

  public:
    ImuModel(std::unique_ptr<util::Gpio> gpio, rclcpp::Clock & clock);
    auto on_read() -> sinsei_umiusi_control::state::imu::ImuState;
};

}  // namespace sinsei_umiusi_control::hardware_model

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_IMU_MODEL_HPP