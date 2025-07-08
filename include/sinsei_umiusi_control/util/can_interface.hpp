#ifndef SINSEI_UMIUSI_CONTROL_UTIL_CAN_INTERFACE_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_CAN_INTERFACE_HPP

#include <cstdint>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

namespace sinsei_umiusi_control::util {

class CanInterface {
  public:
    CanInterface() = default;
    virtual ~CanInterface() = default;

    virtual auto init(const std::string ifname) -> tl::expected<void, std::string> = 0;
    virtual auto send_stdframe(uint32_t id, const uint8_t * data, size_t length)
        -> tl::expected<void, std::string> = 0;
    virtual auto send_extframe(uint32_t id, const uint8_t * data, size_t length)
        -> tl::expected<void, std::string> = 0;
    virtual auto receive_frame() -> tl::expected<void, std::string> = 0;
};

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_CAN_INTERFACE_HPP