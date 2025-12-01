#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_IMU_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_IMU_MODEL_HPP

#include <memory>
#include <rcpputils/tl_expected/expected.hpp>

#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"

namespace sinsei_umiusi_control::hardware_model {

class ImuModel {
  public:
    static constexpr interface::Gpio::Addr ADDRESS{0x28};

    static constexpr interface::Gpio::Addr ID{0xA0};

    /* Page id register definition */
    static constexpr interface::Gpio::Addr PAGE_ID_ADDR{0x07};

    /* PAGE0 REGISTER DEFINITION START*/
    static constexpr interface::Gpio::Addr CHIP_ID_ADDR{0x00};

    /* Use ACCEL_DATA_X_LSB_ADDR as START_ADDR to read out all values at once. */
    static constexpr interface::Gpio::Addr START_ADDR{0x08};

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
    template <size_t T>
    static inline auto to_s16(const std::array<std::byte, T> & buffer, int lsb_index) -> int16_t {
        static_assert(T > 1, "Buffer size must be greater than 1");
        uint16_t u = static_cast<uint16_t>(std::to_integer<uint8_t>(buffer[lsb_index])) |
                     (static_cast<uint16_t>(std::to_integer<uint8_t>(buffer[lsb_index + 1])) << 8);
        return static_cast<int16_t>(u);
    }

  public:
    ImuModel(std::unique_ptr<interface::Gpio> gpio);
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
