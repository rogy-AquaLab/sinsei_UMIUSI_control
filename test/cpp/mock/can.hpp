#ifndef SINSEI_UMIUSI_CONTROL_test_cpp_mock_CAN_HPP
#define SINSEI_UMIUSI_CONTROL_test_cpp_mock_CAN_HPP

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rcpputils/tl_expected/expected.hpp>

#include "sinsei_umiusi_control/hardware_model/interface/can.hpp"

namespace sinsei_umiusi_control::test::mock {

class Can : public sinsei_umiusi_control::hardware_model::interface::Can {
  public:
    MOCK_METHOD1(init, tl::expected<void, std::string>(const std::string ifname));
    MOCK_METHOD0(close, tl::expected<void, std::string>());
    MOCK_METHOD1(
        send_frame, tl::expected<void, std::string>(
                        sinsei_umiusi_control::hardware_model::interface::CanFrame && frame));
    MOCK_METHOD0(
        recv_frame,
        tl::expected<sinsei_umiusi_control::hardware_model::interface::CanFrame, std::string>());
};

}  // namespace sinsei_umiusi_control::test::mock

#endif  // SINSEI_UMIUSI_CONTROL_test_cpp_mock_CAN_HPP