#ifndef SINSEI_UMIUSI_CONTROL_test_cpp_mock_GPIO_HPP
#define SINSEI_UMIUSI_CONTROL_test_cpp_mock_GPIO_HPP

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "sinsei_umiusi_control/util/gpio_interface.hpp"

namespace sinsei_umiusi_control::test::mock {

class Gpio : public sinsei_umiusi_control::util::GpioInterface {
  public:
    MOCK_METHOD1(
        set_mode_output, tl::expected<void, sinsei_umiusi_control::util::GpioError>(
                             std::vector<util::GpioPin> pins));
    MOCK_METHOD1(
        set_mode_input, tl::expected<void, sinsei_umiusi_control::util::GpioError>(
                            std::vector<util::GpioPin> pins));
    MOCK_METHOD2(
        write_digital, tl::expected<void, sinsei_umiusi_control::util::GpioError>(
                           util::GpioPin pin, bool enabled));
    MOCK_METHOD0(
        write_pwm, tl::expected<void, sinsei_umiusi_control::util::GpioError>());  // TODO: 未実装
    MOCK_METHOD1(
        i2c_open, tl::expected<void, sinsei_umiusi_control::util::GpioError>(uint32_t address));
    MOCK_METHOD0(i2c_close, tl::expected<void, sinsei_umiusi_control::util::GpioError>());
    MOCK_METHOD1(
        i2c_write_byte,
        tl::expected<void, sinsei_umiusi_control::util::GpioError>(std::byte value));
    MOCK_METHOD0(i2c_read_byte, tl::expected<std::byte, sinsei_umiusi_control::util::GpioError>());
    MOCK_METHOD2(
        i2c_write_byte_data,
        tl::expected<void, sinsei_umiusi_control::util::GpioError>(uint32_t reg, std::byte value));
    MOCK_METHOD1(
        i2c_read_byte_data,
        tl::expected<std::byte, sinsei_umiusi_control::util::GpioError>(uint32_t reg));
};
}  // namespace sinsei_umiusi_control::test::mock

#endif  // SINSEI_UMIUSI_CONTROL_test_cpp_mock_GPIO_HPP
