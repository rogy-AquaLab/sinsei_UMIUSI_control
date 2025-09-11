#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_LINUX_CAN_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_LINUX_CAN_HPP

#include <linux/can.h>

#include <optional>

#include "sinsei_umiusi_control/hardware_model/interface/can.hpp"

namespace sinsei_umiusi_control::hardware_model::impl {

using FileDescriptor = int;

class LinuxCan : public interface::Can {
  private:
    using CanFrame = interface::CanFrame;

    std::optional<FileDescriptor> sock;

    auto send_linux_can_frame(can_frame && frame) -> tl::expected<void, std::string>;
    auto recv_linux_can_frame() -> tl::expected<can_frame, std::string>;

  public:
    LinuxCan();

    auto init(const std::string_view ifname) -> tl::expected<void, std::string> override;
    auto close() -> tl::expected<void, std::string> override;
    auto send_frame(CanFrame && frame) -> tl::expected<void, std::string> override;
    auto recv_frame() -> tl::expected<CanFrame, std::string> override;
};

}  // namespace sinsei_umiusi_control::hardware_model::impl

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_LINUX_CAN_HPP
