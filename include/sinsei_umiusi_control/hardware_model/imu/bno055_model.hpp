#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_IMU_BNO055_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_IMU_BNO055_MODEL_HPP

#include <cstddef>
#include <memory>
#include <tuple>

#include "sinsei_umiusi_control/hardware_model/interface/i2c.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"

namespace sinsei_umiusi_control::hardware_model::imu {

// ref: https://github.com/adafruit/Adafruit_BNO055/blob/1b1af09/Adafruit_BNO055.h
class Bno055Model {
  public:
    using DeviceAddr = interface::I2cDeviceAddr;
    using RegisterAddr = interface::I2cRegisterAddr;

    static constexpr DeviceAddr ADDRESS{0x28};
    static constexpr uint8_t ID{0xA0};

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

  private:
    std::unique_ptr<interface::I2c> i2c;

    static inline auto to_s16(std::byte lsb, std::byte msb) -> int16_t {
        const auto raw =
            static_cast<uint16_t>(std::to_integer<uint8_t>(lsb)) |
            (static_cast<uint16_t>(std::to_integer<uint8_t>(msb)) << 8);
        return static_cast<int16_t>(raw);
    }

    enum class VectorType { Accelerometer, Magnetometer, Gyroscope, Euler, LinearAccel, Gravity };

    constexpr auto get_address(VectorType type) -> RegisterAddr {
        switch (type) {
            case VectorType::Accelerometer:
                return ACCEL_DATA_X_LSB_ADDR;
            case VectorType::Magnetometer:
                return MAG_DATA_X_LSB_ADDR;
            case VectorType::Gyroscope:
                return GYRO_DATA_X_LSB_ADDR;
            case VectorType::Euler:
                return EULER_DATA_H_LSB_ADDR;
            case VectorType::LinearAccel:
                return LINEAR_ACCEL_DATA_X_LSB_ADDR;
            case VectorType::Gravity:
                return GRAVITY_DATA_X_LSB_ADDR;
            default:
                return RegisterAddr{std::byte{0x00}};
        }
    }

    constexpr auto get_scale(VectorType type) -> double {
        switch (type) {
            case VectorType::Magnetometer:
            case VectorType::Gyroscope:
            case VectorType::Euler:
                return 1.0 / 16.0;
            case VectorType::Accelerometer:
            case VectorType::LinearAccel:
            case VectorType::Gravity:
                return 1.0 / 100.0;
            default:
                return 0.0;
        }
    }

    using Vector3 = std::tuple<double, double, double>;

    auto get_vector(VectorType type) -> tl::expected<Vector3, std::string>;

  public:
    explicit Bno055Model(std::unique_ptr<interface::I2c> i2c);

    auto begin() -> tl::expected<void, std::string>;
    auto close() -> tl::expected<void, std::string>;
    auto get_temp() -> tl::expected<state::imu::Temperature, std::string>;
    auto get_quat() -> tl::expected<state::imu::Quaternion, std::string>;
    auto get_acceleration() -> tl::expected<state::imu::Acceleration, std::string>;
    auto get_angular_velocity() -> tl::expected<state::imu::AngularVelocity, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model::imu

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_IMU_BNO055_MODEL_HPP
