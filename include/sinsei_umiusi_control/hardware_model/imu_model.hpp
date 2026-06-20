#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMU_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMU_MODEL_HPP

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <tuple>

#include <rcpputils/tl_expected/expected.hpp>

#include "sinsei_umiusi_control/hardware_model/interface/i2c.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"

namespace sinsei_umiusi_control::hardware_model {

// ref: https://github.com/adafruit/Adafruit_BNO055/blob/1b1af09/Adafruit_BNO055.h
class ImuModel {
  public:
    using DeviceAddr = interface::I2cDeviceAddr;
    using RegisterAddr = interface::I2cRegisterAddr;
    using ReadResult = std::tuple<
        state::imu::Quaternion, state::imu::Acceleration, state::imu::AngularVelocity,
        state::imu::Temperature>;

    static constexpr DeviceAddr ADDRESS{0x28};
    static constexpr uint8_t CHIP_ID{0xA0};

    /* Page id register definition */
    static constexpr RegisterAddr PAGE_ID_ADDR{std::byte{0x07}};

    /* PAGE0 REGISTER DEFINITION START*/
    static constexpr RegisterAddr CHIP_ID_ADDR{std::byte{0x00}};

    /* Accel data registers */
    static constexpr RegisterAddr ACCEL_DATA_X_LSB_ADDR{std::byte{0x08}};
    static constexpr RegisterAddr ACCEL_DATA_X_MSB_ADDR{std::byte{0x09}};
    static constexpr RegisterAddr ACCEL_DATA_Y_LSB_ADDR{std::byte{0x0A}};
    static constexpr RegisterAddr ACCEL_DATA_Y_MSB_ADDR{std::byte{0x0B}};
    static constexpr RegisterAddr ACCEL_DATA_Z_LSB_ADDR{std::byte{0x0C}};
    static constexpr RegisterAddr ACCEL_DATA_Z_MSB_ADDR{std::byte{0x0D}};

    /* Mag data register */
    static constexpr RegisterAddr MAG_DATA_X_LSB_ADDR{std::byte{0x0E}};
    static constexpr RegisterAddr MAG_DATA_X_MSB_ADDR{std::byte{0x0F}};
    static constexpr RegisterAddr MAG_DATA_Y_LSB_ADDR{std::byte{0x10}};
    static constexpr RegisterAddr MAG_DATA_Y_MSB_ADDR{std::byte{0x11}};
    static constexpr RegisterAddr MAG_DATA_Z_LSB_ADDR{std::byte{0x12}};
    static constexpr RegisterAddr MAG_DATA_Z_MSB_ADDR{std::byte{0x13}};

    /* Gyro data registers */
    static constexpr RegisterAddr GYRO_DATA_X_LSB_ADDR{std::byte{0x14}};
    static constexpr RegisterAddr GYRO_DATA_X_MSB_ADDR{std::byte{0x15}};
    static constexpr RegisterAddr GYRO_DATA_Y_LSB_ADDR{std::byte{0x16}};
    static constexpr RegisterAddr GYRO_DATA_Y_MSB_ADDR{std::byte{0x17}};
    static constexpr RegisterAddr GYRO_DATA_Z_LSB_ADDR{std::byte{0x18}};
    static constexpr RegisterAddr GYRO_DATA_Z_MSB_ADDR{std::byte{0x19}};

    /* Euler data registers */
    static constexpr RegisterAddr EULER_DATA_H_LSB_ADDR{std::byte{0x1A}};
    static constexpr RegisterAddr EULER_DATA_H_MSB_ADDR{std::byte{0x1B}};
    static constexpr RegisterAddr EULER_DATA_R_LSB_ADDR{std::byte{0x1C}};
    static constexpr RegisterAddr EULER_DATA_R_MSB_ADDR{std::byte{0x1D}};
    static constexpr RegisterAddr EULER_DATA_P_LSB_ADDR{std::byte{0x1E}};
    static constexpr RegisterAddr EULER_DATA_P_MSB_ADDR{std::byte{0x1F}};

    /* Quaternion data registers */
    static constexpr RegisterAddr QUATERNION_DATA_W_LSB_ADDR{std::byte{0x20}};
    static constexpr RegisterAddr QUATERNION_DATA_W_MSB_ADDR{std::byte{0x21}};
    static constexpr RegisterAddr QUATERNION_DATA_X_LSB_ADDR{std::byte{0x22}};
    static constexpr RegisterAddr QUATERNION_DATA_X_MSB_ADDR{std::byte{0x23}};
    static constexpr RegisterAddr QUATERNION_DATA_Y_LSB_ADDR{std::byte{0x24}};
    static constexpr RegisterAddr QUATERNION_DATA_Y_MSB_ADDR{std::byte{0x25}};
    static constexpr RegisterAddr QUATERNION_DATA_Z_LSB_ADDR{std::byte{0x26}};
    static constexpr RegisterAddr QUATERNION_DATA_Z_MSB_ADDR{std::byte{0x27}};

    /* Linear acceleration data registers */
    static constexpr RegisterAddr LINEAR_ACCEL_DATA_X_LSB_ADDR{std::byte{0x28}};
    static constexpr RegisterAddr LINEAR_ACCEL_DATA_X_MSB_ADDR{std::byte{0x29}};
    static constexpr RegisterAddr LINEAR_ACCEL_DATA_Y_LSB_ADDR{std::byte{0x2A}};
    static constexpr RegisterAddr LINEAR_ACCEL_DATA_Y_MSB_ADDR{std::byte{0x2B}};
    static constexpr RegisterAddr LINEAR_ACCEL_DATA_Z_LSB_ADDR{std::byte{0x2C}};
    static constexpr RegisterAddr LINEAR_ACCEL_DATA_Z_MSB_ADDR{std::byte{0x2D}};

    /* Gravity data registers */
    static constexpr RegisterAddr GRAVITY_DATA_X_LSB_ADDR{std::byte{0x2E}};
    static constexpr RegisterAddr GRAVITY_DATA_X_MSB_ADDR{std::byte{0x2F}};
    static constexpr RegisterAddr GRAVITY_DATA_Y_LSB_ADDR{std::byte{0x30}};
    static constexpr RegisterAddr GRAVITY_DATA_Y_MSB_ADDR{std::byte{0x31}};
    static constexpr RegisterAddr GRAVITY_DATA_Z_LSB_ADDR{std::byte{0x32}};
    static constexpr RegisterAddr GRAVITY_DATA_Z_MSB_ADDR{std::byte{0x33}};

    /* Temperature data register */
    static constexpr RegisterAddr TEMP_ADDR{std::byte{0x34}};

    /* Mode registers */
    static constexpr RegisterAddr OPR_MODE_ADDR{std::byte{0x3D}};
    static constexpr RegisterAddr PWR_MODE_ADDR{std::byte{0x3E}};
    static constexpr RegisterAddr SYS_TRIGGER_ADDR{std::byte{0x3F}};

    /** Operation mode settings **/
    static constexpr std::byte OPERATION_MODE_CONFIG{0x00};
    static constexpr std::byte OPERATION_MODE_NDOF{0x0C};

    /** BNO055 power settings */
    static constexpr std::byte POWER_MODE_NORMAL{0x00};

    /* 一括読み出しに使用するレジスタ */
    static constexpr RegisterAddr FRAME_START{GYRO_DATA_X_LSB_ADDR};
    static constexpr size_t FRAME_LENGTH =
        std::to_integer<size_t>(TEMP_ADDR.value) - std::to_integer<size_t>(FRAME_START.value) + 1;
    static constexpr size_t OFFSET_GYRO = 0;
    static constexpr size_t OFFSET_QUAT =
        std::to_integer<size_t>(QUATERNION_DATA_W_LSB_ADDR.value) -
        std::to_integer<size_t>(FRAME_START.value);
    static constexpr size_t OFFSET_LINEAR_ACCEL =
        std::to_integer<size_t>(LINEAR_ACCEL_DATA_X_LSB_ADDR.value) -
        std::to_integer<size_t>(FRAME_START.value);
    static constexpr size_t OFFSET_TEMP =
        std::to_integer<size_t>(TEMP_ADDR.value) - std::to_integer<size_t>(FRAME_START.value);

  private:
    std::unique_ptr<interface::I2c> i2c;

    // 連続する2バイト（LSB、MSB）から符号付き16ビット整数（int16_t）を復元
    static inline auto to_s16(const std::byte * bytes) -> int16_t {
        const auto raw = static_cast<uint16_t>(std::to_integer<uint8_t>(bytes[0])) |
                         (static_cast<uint16_t>(std::to_integer<uint8_t>(bytes[1])) << 8);
        return static_cast<int16_t>(raw);
    }

    auto write_reg(RegisterAddr reg, std::byte value) -> tl::expected<void, std::string>;
    auto read_reg(RegisterAddr reg, interface::I2cBufferView buffer)
        -> tl::expected<void, std::string>;
    auto validate_id() -> tl::expected<void, std::string>;
    auto wait_for_device_ready(int max_attempts, std::chrono::milliseconds retry_interval)
        -> tl::expected<void, std::string>;
    auto reset_device() -> tl::expected<void, std::string>;
    auto configure_device() -> tl::expected<void, std::string>;

  public:
    explicit ImuModel(std::unique_ptr<interface::I2c> i2c);

    auto on_init() -> tl::expected<void, std::string>;
    auto on_destroy() -> tl::expected<void, std::string>;
    auto on_read() -> tl::expected<ReadResult, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMU_MODEL_HPP
