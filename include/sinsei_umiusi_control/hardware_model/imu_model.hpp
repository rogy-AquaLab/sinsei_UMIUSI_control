#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_IMU_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_IMU_MODEL_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>
#include <tuple>

#include "sinsei_umiusi_control/hardware_model/interface/i2c.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"

namespace sinsei_umiusi_control::hardware_model {

class ImuModel {
  public:
    using DeviceAddr = interface::I2cDeviceAddr;
    using RegisterAddr = interface::I2cRegisterAddr;

    static constexpr interface::I2cDeviceAddr ADDRESS{0x28};
    static constexpr uint8_t ID{0xA0};

    /* Page id register definition */
    static constexpr interface::I2cRegisterAddr PAGE_ID_ADDR{std::byte{0x07}};

    /* PAGE0 REGISTER DEFINITION START*/
    static constexpr interface::I2cRegisterAddr CHIP_ID_ADDR{std::byte{0x00}};

    /* Accel data registers */
    static constexpr interface::I2cRegisterAddr ACCEL_DATA_X_LSB_ADDR{std::byte{0x08}};
    static constexpr interface::I2cRegisterAddr ACCEL_DATA_X_MSB_ADDR{std::byte{0x09}};
    static constexpr interface::I2cRegisterAddr ACCEL_DATA_Y_LSB_ADDR{std::byte{0x0A}};
    static constexpr interface::I2cRegisterAddr ACCEL_DATA_Y_MSB_ADDR{std::byte{0x0B}};
    static constexpr interface::I2cRegisterAddr ACCEL_DATA_Z_LSB_ADDR{std::byte{0x0C}};
    static constexpr interface::I2cRegisterAddr ACCEL_DATA_Z_MSB_ADDR{std::byte{0x0D}};

    /* Mag data register */
    static constexpr interface::I2cRegisterAddr MAG_DATA_X_LSB_ADDR{std::byte{0x0E}};
    static constexpr interface::I2cRegisterAddr MAG_DATA_X_MSB_ADDR{std::byte{0x0F}};
    static constexpr interface::I2cRegisterAddr MAG_DATA_Y_LSB_ADDR{std::byte{0x10}};
    static constexpr interface::I2cRegisterAddr MAG_DATA_Y_MSB_ADDR{std::byte{0x11}};
    static constexpr interface::I2cRegisterAddr MAG_DATA_Z_LSB_ADDR{std::byte{0x12}};
    static constexpr interface::I2cRegisterAddr MAG_DATA_Z_MSB_ADDR{std::byte{0x13}};

    /* Gyro data registers */
    static constexpr interface::I2cRegisterAddr GYRO_DATA_X_LSB_ADDR{std::byte{0x14}};
    static constexpr interface::I2cRegisterAddr GYRO_DATA_X_MSB_ADDR{std::byte{0x15}};
    static constexpr interface::I2cRegisterAddr GYRO_DATA_Y_LSB_ADDR{std::byte{0x16}};
    static constexpr interface::I2cRegisterAddr GYRO_DATA_Y_MSB_ADDR{std::byte{0x17}};
    static constexpr interface::I2cRegisterAddr GYRO_DATA_Z_LSB_ADDR{std::byte{0x18}};
    static constexpr interface::I2cRegisterAddr GYRO_DATA_Z_MSB_ADDR{std::byte{0x19}};

    /* Euler data registers */
    static constexpr interface::I2cRegisterAddr EULER_DATA_H_LSB_ADDR{std::byte{0x1A}};
    static constexpr interface::I2cRegisterAddr EULER_DATA_H_MSB_ADDR{std::byte{0x1B}};
    static constexpr interface::I2cRegisterAddr EULER_DATA_R_LSB_ADDR{std::byte{0x1C}};
    static constexpr interface::I2cRegisterAddr EULER_DATA_R_MSB_ADDR{std::byte{0x1D}};
    static constexpr interface::I2cRegisterAddr EULER_DATA_P_LSB_ADDR{std::byte{0x1E}};
    static constexpr interface::I2cRegisterAddr EULER_DATA_P_MSB_ADDR{std::byte{0x1F}};

    /* Quaternion data registers */
    static constexpr interface::I2cRegisterAddr QUATERNION_DATA_W_LSB_ADDR{std::byte{0x20}};
    static constexpr interface::I2cRegisterAddr QUATERNION_DATA_W_MSB_ADDR{std::byte{0x21}};
    static constexpr interface::I2cRegisterAddr QUATERNION_DATA_X_LSB_ADDR{std::byte{0x22}};
    static constexpr interface::I2cRegisterAddr QUATERNION_DATA_X_MSB_ADDR{std::byte{0x23}};
    static constexpr interface::I2cRegisterAddr QUATERNION_DATA_Y_LSB_ADDR{std::byte{0x24}};
    static constexpr interface::I2cRegisterAddr QUATERNION_DATA_Y_MSB_ADDR{std::byte{0x25}};
    static constexpr interface::I2cRegisterAddr QUATERNION_DATA_Z_LSB_ADDR{std::byte{0x26}};
    static constexpr interface::I2cRegisterAddr QUATERNION_DATA_Z_MSB_ADDR{std::byte{0x27}};

    /* Linear acceleration data registers */
    static constexpr interface::I2cRegisterAddr LINEAR_ACCEL_DATA_X_LSB_ADDR{std::byte{0x28}};
    static constexpr interface::I2cRegisterAddr LINEAR_ACCEL_DATA_X_MSB_ADDR{std::byte{0x29}};
    static constexpr interface::I2cRegisterAddr LINEAR_ACCEL_DATA_Y_LSB_ADDR{std::byte{0x2A}};
    static constexpr interface::I2cRegisterAddr LINEAR_ACCEL_DATA_Y_MSB_ADDR{std::byte{0x2B}};
    static constexpr interface::I2cRegisterAddr LINEAR_ACCEL_DATA_Z_LSB_ADDR{std::byte{0x2C}};
    static constexpr interface::I2cRegisterAddr LINEAR_ACCEL_DATA_Z_MSB_ADDR{std::byte{0x2D}};

    /* Gravity data registers */
    static constexpr interface::I2cRegisterAddr GRAVITY_DATA_X_LSB_ADDR{std::byte{0x2E}};
    static constexpr interface::I2cRegisterAddr GRAVITY_DATA_X_MSB_ADDR{std::byte{0x2F}};
    static constexpr interface::I2cRegisterAddr GRAVITY_DATA_Y_LSB_ADDR{std::byte{0x30}};
    static constexpr interface::I2cRegisterAddr GRAVITY_DATA_Y_MSB_ADDR{std::byte{0x31}};
    static constexpr interface::I2cRegisterAddr GRAVITY_DATA_Z_LSB_ADDR{std::byte{0x32}};
    static constexpr interface::I2cRegisterAddr GRAVITY_DATA_Z_MSB_ADDR{std::byte{0x33}};

    /* Temperature data register */
    static constexpr interface::I2cRegisterAddr TEMP_ADDR{std::byte{0x34}};

    /* Mode registers */
    static constexpr interface::I2cRegisterAddr OPR_MODE_ADDR{std::byte{0x3D}};
    static constexpr interface::I2cRegisterAddr PWR_MODE_ADDR{std::byte{0x3E}};

    static constexpr interface::I2cRegisterAddr SYS_TRIGGER_ADDR{std::byte{0x3F}};

    /** Operation mode settings **/
    static constexpr std::byte OPERATION_MODE_CONFIG{0x00};
    static constexpr std::byte OPERATION_MODE_NDOF{0x0C};

    /** BNO055 power settings */
    static constexpr std::byte POWER_MODE_NORMAL{0x00};

  private:
    std::unique_ptr<interface::I2c> i2c;

    // 2バイト（LSB、MSB）を結合して符号付き16ビット整数（int16_t）に変換
    static inline auto to_s16(std::byte lsb, std::byte msb) -> int16_t {
        uint16_t u = static_cast<uint16_t>(std::to_integer<uint8_t>(lsb)) |
                     (static_cast<uint16_t>(std::to_integer<uint8_t>(msb)) << 8);
        return static_cast<int16_t>(u);
    }

  public:
    ImuModel(std::unique_ptr<interface::I2c> i2c);
    auto on_init() -> tl::expected<void, std::string>;
    auto on_destroy() -> tl::expected<void, std::string>;
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
