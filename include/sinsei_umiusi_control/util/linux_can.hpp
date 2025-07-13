#ifndef SINSEI_UMIUSI_CONTROL_UTIL_LINUX_CAN_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_LINUX_CAN_HPP

#include <linux/can.h>

#include "sinsei_umiusi_control/util/can_interface.hpp"

namespace sinsei_umiusi_control::util {

class LinuxCan : public CanInterface {
  private:
    int sock;

    auto send_frame(uint32_t id, const uint8_t * data, size_t length, bool is_extended)
        -> tl::expected<void, std::string>;

  public:
    LinuxCan();
    ~LinuxCan();

    auto init(const std::string ifname) -> tl::expected<void, std::string> override;
    auto send_stdframe(uint32_t id, const uint8_t * data, size_t length)
        -> tl::expected<void, std::string> override;
    auto send_extframe(uint32_t id, const uint8_t * data, size_t length)
        -> tl::expected<void, std::string> override;
    auto receive_frame() -> tl::expected<CanFrame, std::string> override;
};

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_LINUX_CAN_HPP