#ifndef SINSEI_UMIUSI_CONTROL_UTIL_CAN_INTERFACE_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_CAN_INTERFACE_HPP

#include <cstdint>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

namespace sinsei_umiusi_control::util {

struct CanFrame {
    uint32_t id;
    uint8_t dlc;
    std::array<uint8_t, 8> data;
};

class CanInterface {
  public:
    CanInterface() = default;
    virtual ~CanInterface() = default;

    virtual auto init(const std::string ifname) -> tl::expected<void, std::string> = 0;
    virtual auto close() -> tl::expected<void, std::string> = 0;
    virtual auto send_frame_std(uint32_t id, const uint8_t * data, size_t length)
        -> tl::expected<void, std::string> = 0;
    virtual auto send_frame_ext(uint32_t id, const uint8_t * data, size_t length)
        -> tl::expected<void, std::string> = 0;
    virtual auto recv_frame() -> tl::expected<CanFrame, std::string> = 0;
};

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_CAN_INTERFACE_HPP