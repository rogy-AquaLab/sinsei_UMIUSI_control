#ifndef SINSEI_UMIUSI_CONTROL_test_cpp_mock_I2C_HPP
#define SINSEI_UMIUSI_CONTROL_test_cpp_mock_I2C_HPP

#include <gmock/gmock.h>

#include "sinsei_umiusi_control/hardware_model/interface/i2c.hpp"

namespace sinsei_umiusi_control::test::mock {

class I2c : public hardware_model::interface::I2c {
  public:
    MOCK_METHOD1(
        open, tl::expected<void, hardware_model::interface::I2c::Error>(
                  const hardware_model::interface::I2c::Addr & address));
    MOCK_METHOD0(close, tl::expected<void, hardware_model::interface::I2c::Error>());
    MOCK_METHOD1(
        write_byte, tl::expected<void, hardware_model::interface::I2c::Error>(std::byte && value));
    MOCK_CONST_METHOD0(read_byte, tl::expected<std::byte, hardware_model::interface::I2c::Error>());
    MOCK_METHOD2(
        write_byte_data, tl::expected<void, hardware_model::interface::I2c::Error>(
                             const hardware_model::interface::I2c::Addr & reg, std::byte && value));
    MOCK_CONST_METHOD1(
        read_byte_data, tl::expected<std::byte, hardware_model::interface::I2c::Error>(
                            const hardware_model::interface::I2c::Addr & reg));
    MOCK_CONST_METHOD3(
        read_block_data, tl::expected<void, hardware_model::interface::I2c::Error>(
                             const hardware_model::interface::I2c::Addr & reg, std::byte * buffer,
                             const size_t length));
    MOCK_METHOD1(
        transfer, tl::expected<void, hardware_model::interface::I2c::Error>(
                      const std::vector<hardware_model::interface::I2cMessage> & msgs));
};

}  // namespace sinsei_umiusi_control::test::mock

#endif  // SINSEI_UMIUSI_CONTROL_test_cpp_mock_I2C_HPP
