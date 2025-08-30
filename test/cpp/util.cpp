#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"

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
    EXPECT_NE(suchm::interface::gpio_error_to_string(GpioError::NoHandle), unknown_msg);
    EXPECT_NE(suchm::interface::gpio_error_to_string(GpioError::I2cNotOpen), unknown_msg);
    EXPECT_NE(suchm::interface::gpio_error_to_string(GpioError::I2cBadBus), unknown_msg);
    EXPECT_NE(suchm::interface::gpio_error_to_string(GpioError::I2CBadAddress), unknown_msg);
    EXPECT_NE(suchm::interface::gpio_error_to_string(GpioError::I2cOpenFailed), unknown_msg);
    EXPECT_NE(suchm::interface::gpio_error_to_string(GpioError::I2cWriteFailed), unknown_msg);
    EXPECT_NE(suchm::interface::gpio_error_to_string(GpioError::I2cReadFailed), unknown_msg);
}

}  // namespace gpio_interface

}  // namespace sinsei_umiusi_control::test::util
