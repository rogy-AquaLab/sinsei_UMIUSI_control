#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"
#include "sinsei_umiusi_control/hardware_model/interface/i2c.hpp"

namespace suchm = sinsei_umiusi_control::hardware_model;

namespace sinsei_umiusi_control::test::util {

namespace gpio_interface {

using hardware_model::interface::GpioError;

TEST(GpioInterfaceTest, gpio_error_to_string) {
    // 各エラーコードに対して、適当な文字列が返されることを確認
    auto unknown_msg = suchm::interface::gpio_error_to_string(GpioError::UnknownError);

    EXPECT_NE(suchm::interface::gpio_error_to_string(GpioError::NotPermitted), unknown_msg);
    EXPECT_NE(suchm::interface::gpio_error_to_string(GpioError::BadGpio), unknown_msg);
    EXPECT_NE(suchm::interface::gpio_error_to_string(GpioError::BadLevel), unknown_msg);
    EXPECT_NE(suchm::interface::gpio_error_to_string(GpioError::BadFlags), unknown_msg);
    EXPECT_NE(suchm::interface::gpio_error_to_string(GpioError::BadHandle), unknown_msg);
    EXPECT_NE(suchm::interface::gpio_error_to_string(GpioError::BadParameter), unknown_msg);
    EXPECT_NE(suchm::interface::gpio_error_to_string(GpioError::BadPulsewidth), unknown_msg);
    EXPECT_NE(suchm::interface::gpio_error_to_string(GpioError::NoHandle), unknown_msg);
}

}  // namespace gpio_interface

namespace i2c_interface {

using hardware_model::interface::I2cError;

TEST(I2cInterfaceTest, i2c_error_to_string) {
    const auto unknown_msg = suchm::interface::i2c_error_to_string(I2cError::UnknownError);

    EXPECT_NE(suchm::interface::i2c_error_to_string(I2cError::NotOpen), unknown_msg);
    EXPECT_NE(suchm::interface::i2c_error_to_string(I2cError::BadBus), unknown_msg);
    EXPECT_NE(suchm::interface::i2c_error_to_string(I2cError::BadAddress), unknown_msg);
    EXPECT_NE(suchm::interface::i2c_error_to_string(I2cError::OpenFailed), unknown_msg);
    EXPECT_NE(suchm::interface::i2c_error_to_string(I2cError::WriteFailed), unknown_msg);
    EXPECT_NE(suchm::interface::i2c_error_to_string(I2cError::ReadFailed), unknown_msg);
}

}  // namespace i2c_interface

}  // namespace sinsei_umiusi_control::test::util
