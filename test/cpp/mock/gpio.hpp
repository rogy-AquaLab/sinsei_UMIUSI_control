#ifndef SINSEI_UMIUSI_CONTROL_test_cpp_mock_GPIO_HPP
#define SINSEI_UMIUSI_CONTROL_test_cpp_mock_GPIO_HPP

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "gmock/gmock.h"
#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"

namespace sinsei_umiusi_control::test::mock {

class Gpio : public hardware_model::interface::Gpio {
  public:
    MOCK_METHOD1(
        set_mode_output, tl::expected<void, hardware_model::interface::Gpio::Error>(
                             const std::vector<hardware_model::interface::Gpio::Pin> & pins));
    MOCK_METHOD1(
        set_mode_input, tl::expected<void, hardware_model::interface::Gpio::Error>(
                            const std::vector<hardware_model::interface::Gpio::Pin> & pins));
    MOCK_METHOD2(
        write_digital, tl::expected<void, hardware_model::interface::Gpio::Error>(
                           const hardware_model::interface::Gpio::Pin & pin, bool && enabled));
    MOCK_METHOD2(
        write_pwm_duty,
        tl::expected<void, hardware_model::interface::Gpio::Error>(
            const hardware_model::interface::Gpio::Pin & pin, const double && duty));
    MOCK_METHOD2(
        write_servo_pulsewidth,
        tl::expected<void, hardware_model::interface::Gpio::Error>(
            const hardware_model::interface::Gpio::Pin & pin, const uint16_t && pulsewidth));
    MOCK_METHOD1(
        i2c_open, tl::expected<void, hardware_model::interface::Gpio::Error>(
                      const hardware_model::interface::Gpio::Addr & address));
    MOCK_METHOD0(i2c_close, tl::expected<void, hardware_model::interface::Gpio::Error>());
    MOCK_METHOD1(
        i2c_write_byte,
        tl::expected<void, hardware_model::interface::Gpio::Error>(std::byte && value));
    MOCK_CONST_METHOD0(
        i2c_read_byte, tl::expected<std::byte, hardware_model::interface::Gpio::Error>());
    MOCK_METHOD2(
        i2c_write_byte_data,
        tl::expected<void, hardware_model::interface::Gpio::Error>(
            const hardware_model::interface::Gpio::Addr & reg, std::byte && value));
    MOCK_CONST_METHOD1(
        i2c_read_byte_data, tl::expected<std::byte, hardware_model::interface::Gpio::Error>(
                                const hardware_model::interface::Gpio::Addr & reg));
    MOCK_CONST_METHOD3(
        i2c_read_block_data, tl::expected<void, hardware_model::interface::Gpio::Error>(
                                 const hardware_model::interface::Gpio::Addr & reg,
                                 std::byte * buffer, const size_t length));
};
}  // namespace sinsei_umiusi_control::test::mock

#endif  // SINSEI_UMIUSI_CONTROL_test_cpp_mock_GPIO_HPP
