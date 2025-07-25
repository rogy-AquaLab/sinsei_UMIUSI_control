#ifndef SINSEI_UMIUSI_CONTROL_test_cpp_mock_GPIO_HPP
#define SINSEI_UMIUSI_CONTROL_test_cpp_mock_GPIO_HPP

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "sinsei_umiusi_control/hardware_model/interface/gpio_interface.hpp"

namespace sinsei_umiusi_control::test::mock {

class Gpio : public hardware_model::interface::GpioInterface {
  public:
    MOCK_METHOD1(
        set_mode_output, tl::expected<void, hardware_model::interface::GpioError>(
                             std::vector<hardware_model::interface::GpioPin> pins));
    MOCK_METHOD1(
        set_mode_input, tl::expected<void, hardware_model::interface::GpioError>(
                            std::vector<hardware_model::interface::GpioPin> pins));
    MOCK_METHOD2(
        write_digital, tl::expected<void, hardware_model::interface::GpioError>(
                           hardware_model::interface::GpioPin pin, bool enabled));
    MOCK_METHOD0(
        write_pwm, tl::expected<void, hardware_model::interface::GpioError>());  // TODO: 未実装
    MOCK_METHOD1(
        i2c_open, tl::expected<void, hardware_model::interface::GpioError>(uint32_t address));
    MOCK_METHOD0(i2c_close, tl::expected<void, hardware_model::interface::GpioError>());
    MOCK_METHOD1(
        i2c_write_byte, tl::expected<void, hardware_model::interface::GpioError>(std::byte value));
    MOCK_METHOD0(i2c_read_byte, tl::expected<std::byte, hardware_model::interface::GpioError>());
    MOCK_METHOD2(
        i2c_write_byte_data,
        tl::expected<void, hardware_model::interface::GpioError>(uint32_t reg, std::byte value));
    MOCK_METHOD1(
        i2c_read_byte_data,
        tl::expected<std::byte, hardware_model::interface::GpioError>(uint32_t reg));
};
}  // namespace sinsei_umiusi_control::test::mock

#endif  // SINSEI_UMIUSI_CONTROL_test_cpp_mock_GPIO_HPP
