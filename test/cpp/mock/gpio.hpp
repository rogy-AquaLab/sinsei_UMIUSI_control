#ifndef SINSEI_UMIUSI_CONTROL_test_cpp_mock_GPIO_HPP
#define SINSEI_UMIUSI_CONTROL_test_cpp_mock_GPIO_HPP

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "gmock/gmock.h"
#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"

namespace sinsei_umiusi_control::test::mock {

class GpioLineRequest : public hardware_model::interface::GpioLineRequest {
  public:
    MOCK_METHOD(
        (tl::expected<void, std::string>), set_values,
        ((const std::vector<hardware_model::interface::GpioValue> & values)), (override));
    MOCK_METHOD(std::size_t, size, (), (const, noexcept, override));
};

class GpioChip : public hardware_model::interface::GpioChip {
  public:
    using GpioLineRequestPtr = std::unique_ptr<hardware_model::interface::GpioLineRequest>;

    MOCK_METHOD(
        (tl::expected<GpioLineRequestPtr, std::string>), request_outputs,
        ((hardware_model::interface::GpioOutputRequest request)),
        (override));
};
}  // namespace sinsei_umiusi_control::test::mock

#endif  // SINSEI_UMIUSI_CONTROL_test_cpp_mock_GPIO_HPP
