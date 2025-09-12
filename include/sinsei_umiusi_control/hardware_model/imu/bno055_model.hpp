#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_IMU_BNO055_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_IMU_BNO055_MODEL_HPP

#include <cstddef>
#include <memory>

#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"

namespace sinsei_umiusi_control::hardware_model::imu {

// ref: https://github.com/adafruit/Adafruit_BNO055/blob/1b1af09/Adafruit_BNO055.h
class Bno055Model {
  private:
    std::unique_ptr<interface::Gpio> gpio;

  public:
    static constexpr interface::Gpio::Addr ADDRESS{0x28};

    static constexpr interface::Gpio::Addr ID{0xA0};

    /* Page id register definition */
    static constexpr interface::Gpio::Addr PAGE_ID_ADDR{0x07};

    /* PAGE0 REGISTER DEFINITION START*/
    static constexpr interface::Gpio::Addr CHIP_ID_ADDR{0x00};

    /* Mode registers */
    static constexpr interface::Gpio::Addr OPR_MODE_ADDR{0x3D};
    static constexpr interface::Gpio::Addr PWR_MODE_ADDR{0x3E};

    static constexpr interface::Gpio::Addr SYS_TRIGGER_ADDR{0x3F};

    /* Quaternion data registers */
    static constexpr interface::Gpio::Addr QUATERNION_DATA_W_LSB_ADDR{0x20};
    static constexpr interface::Gpio::Addr QUATERNION_DATA_W_MSB_ADDR{0x21};
    static constexpr interface::Gpio::Addr QUATERNION_DATA_X_LSB_ADDR{0x22};
    static constexpr interface::Gpio::Addr QUATERNION_DATA_X_MSB_ADDR{0x23};
    static constexpr interface::Gpio::Addr QUATERNION_DATA_Y_LSB_ADDR{0x24};
    static constexpr interface::Gpio::Addr QUATERNION_DATA_Y_MSB_ADDR{0x25};
    static constexpr interface::Gpio::Addr QUATERNION_DATA_Z_LSB_ADDR{0x26};
    static constexpr interface::Gpio::Addr QUATERNION_DATA_Z_MSB_ADDR{0x27};

    /* Temperature data register */
    static constexpr interface::Gpio::Addr TEMP_ADDR{0x34};

    /** Operation mode settings **/
    static constexpr std::byte OPERATION_MODE_CONFIG{0x00};
    static constexpr std::byte OPERATION_MODE_NDOF{0x0C};

    /** BNO055 power settings */
    static constexpr std::byte POWER_MODE_NORMAL{0x00};

    Bno055Model(std::unique_ptr<interface::Gpio> gpio);

    auto begin() -> tl::expected<void, std::string>;

    auto get_temp() -> tl::expected<state::imu::Temperature, std::string>;

    auto get_quad() -> tl::expected<state::imu::Quaternion, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model::imu

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_IMU_BNO055_MODEL_HPP
