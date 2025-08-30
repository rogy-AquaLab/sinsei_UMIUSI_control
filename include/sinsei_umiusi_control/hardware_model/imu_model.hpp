#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_IMU_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_IMU_MODEL_HPP

#include <cstddef>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/tl_expected/expected.hpp>

#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"

namespace sinsei_umiusi_control::hardware_model {

// ref: https://github.com/adafruit/Adafruit_BNO055/blob/1b1af09/Adafruit_BNO055.h
class ImuModel {
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

  private:
    std::unique_ptr<interface::Gpio> gpio;

    auto read_quat() -> tl::expected<state::imu::Quaternion, std::string>;

  public:
    ImuModel(std::unique_ptr<interface::Gpio> gpio);
    auto on_init() -> tl::expected<void, std::string>;
    auto on_read() -> tl::expected<
                       std::tuple<
                           sinsei_umiusi_control::state::imu::Quaternion,
                           sinsei_umiusi_control::state::imu::Velocity,
                           sinsei_umiusi_control::state::imu::Temperature>,
                       std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_IMU_MODEL_HPP
