#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_IMU_BNO055_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_IMU_BNO055_MODEL_HPP

#include <cassert>
#include <cstddef>
#include <memory>

#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"

namespace sinsei_umiusi_control::hardware_model::imu {

// ref: https://github.com/adafruit/Adafruit_BNO055/blob/1b1af09/Adafruit_BNO055.h
class Bno055Model {
  public:
    static constexpr interface::Gpio::Addr ADDRESS{0x28};

    static constexpr interface::Gpio::Addr ID{0xA0};

    /* Page id register definition */
    static constexpr interface::Gpio::Addr PAGE_ID_ADDR{0x07};

    /* PAGE0 REGISTER DEFINITION START*/
    static constexpr interface::Gpio::Addr CHIP_ID_ADDR{0x00};

    /* Accel data registers */
    static constexpr interface::Gpio::Addr ACCEL_DATA_X_LSB_ADDR{0x08};
    static constexpr interface::Gpio::Addr ACCEL_DATA_X_MSB_ADDR{0x09};
    static constexpr interface::Gpio::Addr ACCEL_DATA_Y_LSB_ADDR{0x0A};
    static constexpr interface::Gpio::Addr ACCEL_DATA_Y_MSB_ADDR{0x0B};
    static constexpr interface::Gpio::Addr ACCEL_DATA_Z_LSB_ADDR{0x0C};
    static constexpr interface::Gpio::Addr ACCEL_DATA_Z_MSB_ADDR{0x0D};

    /* Mag data register */
    static constexpr interface::Gpio::Addr MAG_DATA_X_LSB_ADDR{0x0E};
    static constexpr interface::Gpio::Addr MAG_DATA_X_MSB_ADDR{0x0F};
    static constexpr interface::Gpio::Addr MAG_DATA_Y_LSB_ADDR{0x10};
    static constexpr interface::Gpio::Addr MAG_DATA_Y_MSB_ADDR{0x11};
    static constexpr interface::Gpio::Addr MAG_DATA_Z_LSB_ADDR{0x12};
    static constexpr interface::Gpio::Addr MAG_DATA_Z_MSB_ADDR{0x13};

    /* Gyro data registers */
    static constexpr interface::Gpio::Addr GYRO_DATA_X_LSB_ADDR{0x14};
    static constexpr interface::Gpio::Addr GYRO_DATA_X_MSB_ADDR{0x15};
    static constexpr interface::Gpio::Addr GYRO_DATA_Y_LSB_ADDR{0x16};
    static constexpr interface::Gpio::Addr GYRO_DATA_Y_MSB_ADDR{0x17};
    static constexpr interface::Gpio::Addr GYRO_DATA_Z_LSB_ADDR{0x18};
    static constexpr interface::Gpio::Addr GYRO_DATA_Z_MSB_ADDR{0x19};

    /* Euler data registers */
    static constexpr interface::Gpio::Addr EULER_DATA_H_LSB_ADDR{0x1A};
    static constexpr interface::Gpio::Addr EULER_DATA_H_MSB_ADDR{0x1B};
    static constexpr interface::Gpio::Addr EULER_DATA_R_LSB_ADDR{0x1C};
    static constexpr interface::Gpio::Addr EULER_DATA_R_MSB_ADDR{0x1D};
    static constexpr interface::Gpio::Addr EULER_DATA_P_LSB_ADDR{0x1E};
    static constexpr interface::Gpio::Addr EULER_DATA_P_MSB_ADDR{0x1F};

    /* Quaternion data registers */
    static constexpr interface::Gpio::Addr QUATERNION_DATA_W_LSB_ADDR{0x20};
    static constexpr interface::Gpio::Addr QUATERNION_DATA_W_MSB_ADDR{0x21};
    static constexpr interface::Gpio::Addr QUATERNION_DATA_X_LSB_ADDR{0x22};
    static constexpr interface::Gpio::Addr QUATERNION_DATA_X_MSB_ADDR{0x23};
    static constexpr interface::Gpio::Addr QUATERNION_DATA_Y_LSB_ADDR{0x24};
    static constexpr interface::Gpio::Addr QUATERNION_DATA_Y_MSB_ADDR{0x25};
    static constexpr interface::Gpio::Addr QUATERNION_DATA_Z_LSB_ADDR{0x26};
    static constexpr interface::Gpio::Addr QUATERNION_DATA_Z_MSB_ADDR{0x27};

    /* Linear acceleration data registers */
    static constexpr interface::Gpio::Addr LINEAR_ACCEL_DATA_X_LSB_ADDR{0x28};
    static constexpr interface::Gpio::Addr LINEAR_ACCEL_DATA_X_MSB_ADDR{0x29};
    static constexpr interface::Gpio::Addr LINEAR_ACCEL_DATA_Y_LSB_ADDR{0x2A};
    static constexpr interface::Gpio::Addr LINEAR_ACCEL_DATA_Y_MSB_ADDR{0x2B};
    static constexpr interface::Gpio::Addr LINEAR_ACCEL_DATA_Z_LSB_ADDR{0x2C};
    static constexpr interface::Gpio::Addr LINEAR_ACCEL_DATA_Z_MSB_ADDR{0x2D};

    /* Gravity data registers */
    static constexpr interface::Gpio::Addr GRAVITY_DATA_X_LSB_ADDR{0x2E};
    static constexpr interface::Gpio::Addr GRAVITY_DATA_X_MSB_ADDR{0x2F};
    static constexpr interface::Gpio::Addr GRAVITY_DATA_Y_LSB_ADDR{0x30};
    static constexpr interface::Gpio::Addr GRAVITY_DATA_Y_MSB_ADDR{0x31};
    static constexpr interface::Gpio::Addr GRAVITY_DATA_Z_LSB_ADDR{0x32};
    static constexpr interface::Gpio::Addr GRAVITY_DATA_Z_MSB_ADDR{0x33};

    /* Temperature data register */
    static constexpr interface::Gpio::Addr TEMP_ADDR{0x34};

    /* Mode registers */
    static constexpr interface::Gpio::Addr OPR_MODE_ADDR{0x3D};
    static constexpr interface::Gpio::Addr PWR_MODE_ADDR{0x3E};

    static constexpr interface::Gpio::Addr SYS_TRIGGER_ADDR{0x3F};

    /** Operation mode settings **/
    static constexpr std::byte OPERATION_MODE_CONFIG{0x00};
    static constexpr std::byte OPERATION_MODE_NDOF{0x0C};

    /** BNO055 power settings */
    static constexpr std::byte POWER_MODE_NORMAL{0x00};

  private:
    std::unique_ptr<interface::Gpio> gpio;

    // 2バイト（LSB、MSB）を結合して符号付き16ビット整数（int16_t）に変換
    static inline auto to_s16(std::byte lsb, std::byte msb) -> int16_t {
        uint16_t u = static_cast<uint16_t>(std::to_integer<uint8_t>(lsb)) |
                     (static_cast<uint16_t>(std::to_integer<uint8_t>(msb)) << 8);
        return static_cast<int16_t>(u);
    }

    enum class VectorType { Accelerometer, Magnetometer, Gyroscope, Euler, LinearAccel, Gravity };

    constexpr auto get_address(VectorType type) -> interface::Gpio::Addr {
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
        }

        assert(false && "Unexpected VectorType in get_address()");
        return 0;
    }

    constexpr auto get_scale(VectorType type) -> double {
        switch (type) {
            case VectorType::Magnetometer:  // 1uT = 16 LSB
            case VectorType::Gyroscope:     // 1dps = 16 LSB
            case VectorType::Euler:         // 1 degree = 16 LSB
                return 1.0 / 16.0;

            case VectorType::Accelerometer:  // 1m/s^2 = 100 LSB
            case VectorType::LinearAccel:    // 1m/s^2 = 100 LSB
            case VectorType::Gravity:        // 1m/s^2 = 100 LSB
                return 1.0 / 100.0;
        }

        assert(false && "Unexpected VectorType in get_scale()");
        return 0.0;
    }

    using Vector3 = std::tuple<double, double, double>;

    auto get_vector(VectorType type) -> tl::expected<Vector3, std::string>;

  public:
    Bno055Model(std::unique_ptr<interface::Gpio> gpio);

    auto begin() -> tl::expected<void, std::string>;

    auto close() -> tl::expected<void, std::string>;

    auto get_temp() -> tl::expected<state::imu::Temperature, std::string>;

    auto get_quat() -> tl::expected<state::imu::Quaternion, std::string>;

    auto get_acceleration() -> tl::expected<state::imu::Acceleration, std::string>;

    auto get_angular_velocity() -> tl::expected<state::imu::AngularVelocity, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model::imu

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_IMU_BNO055_MODEL_HPP
