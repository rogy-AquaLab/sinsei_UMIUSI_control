#ifndef SINSEI_UMIUSI_CONTROL_test_cpp_mock_I2C_HPP
#define SINSEI_UMIUSI_CONTROL_test_cpp_mock_I2C_HPP

#include <gmock/gmock.h>

#include "sinsei_umiusi_control/hardware_model/interface/i2c.hpp"

namespace sinsei_umiusi_control::test::mock {

class I2c : public hardware_model::interface::I2c {
  public:
    MOCK_METHOD0(open, tl::expected<void, std::string>());
    MOCK_METHOD0(close, tl::expected<void, std::string>());
    MOCK_METHOD2(
        transfer, tl::expected<void, std::string>(
                      const hardware_model::interface::I2cMessage * msgs, std::size_t size));
};

}  // namespace sinsei_umiusi_control::test::mock

#endif  // SINSEI_UMIUSI_CONTROL_test_cpp_mock_I2C_HPP
