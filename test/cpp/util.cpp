#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "sinsei_umiusi_control/util/gpio_interface.hpp"

namespace sucutil = sinsei_umiusi_control::util;

namespace sinsei_umiusi_control::test::util {

namespace gpio_interface {

TEST(GpioInterfaceTest, gpio_error_to_string) {
    // 各エラーコードに対して、適当な文字列が返されることを確認
    auto unknown_msg = sucutil::gpio_error_to_string(sucutil::GpioError::UnknownError);

    EXPECT_NE(sucutil::gpio_error_to_string(sucutil::GpioError::NotPermitted), unknown_msg);
    EXPECT_NE(sucutil::gpio_error_to_string(sucutil::GpioError::BadGpio), unknown_msg);
    EXPECT_NE(sucutil::gpio_error_to_string(sucutil::GpioError::BadLevel), unknown_msg);
    EXPECT_NE(sucutil::gpio_error_to_string(sucutil::GpioError::BadFlags), unknown_msg);
    EXPECT_NE(sucutil::gpio_error_to_string(sucutil::GpioError::BadHandle), unknown_msg);
    EXPECT_NE(sucutil::gpio_error_to_string(sucutil::GpioError::BadParameter), unknown_msg);
    EXPECT_NE(sucutil::gpio_error_to_string(sucutil::GpioError::NoHandle), unknown_msg);
    EXPECT_NE(sucutil::gpio_error_to_string(sucutil::GpioError::I2cNotOpen), unknown_msg);
    EXPECT_NE(sucutil::gpio_error_to_string(sucutil::GpioError::I2cBadBus), unknown_msg);
    EXPECT_NE(sucutil::gpio_error_to_string(sucutil::GpioError::I2CBadAddress), unknown_msg);
    EXPECT_NE(sucutil::gpio_error_to_string(sucutil::GpioError::I2cOpenFailed), unknown_msg);
    EXPECT_NE(sucutil::gpio_error_to_string(sucutil::GpioError::I2cWriteFailed), unknown_msg);
    EXPECT_NE(sucutil::gpio_error_to_string(sucutil::GpioError::I2cReadFailed), unknown_msg);
}

}  // namespace gpio_interface

}  // namespace sinsei_umiusi_control::test::util