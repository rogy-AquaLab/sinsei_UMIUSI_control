#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_CAN_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_CAN_HPP

#include <cstdint>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

namespace sinsei_umiusi_control::hardware_model::interface {

struct CanFrame {
    // CAN ID in can frame; 11 bits for standard, 29 bits for extended
    using Id = uint32_t;
    // DLC value in can frame; up to 8
    using DataLength = uint8_t;
    // Data in can frame; 8 bytes
    using Data = std::array<std::byte, 8>;

    Id id;
    DataLength len;
    Data data;
    bool is_extended;
};

class Can {
  public:
    Can() = default;
    virtual ~Can() = default;

    virtual auto init(const std::string_view ifname) -> tl::expected<void, std::string> = 0;
    virtual auto close() -> tl::expected<void, std::string> = 0;
    virtual auto send_frame(CanFrame && frame) -> tl::expected<void, std::string> = 0;
    virtual auto recv_frame() -> tl::expected<CanFrame, std::string> = 0;
};

}  // namespace sinsei_umiusi_control::hardware_model::interface

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_CAN_HPP
