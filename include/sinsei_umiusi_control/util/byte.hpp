#ifndef SINSEI_UMIUSI_CONTROL_UTIL_BYTE_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_BYTE_HPP

#include <array>
#include <cstdint>

namespace sinsei_umiusi_control::util {

// Convert int64_t to 8-byte array in big-endian order
inline auto to_bytes_be(int64_t value) -> std::array<uint8_t, 8> {
    return {
        static_cast<uint8_t>(value >> 24), static_cast<uint8_t>(value >> 16),
        static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
}

// Convert 8-byte array in big-endian order to int32_t
inline auto to_int32_be(std::array<uint8_t, 8> bytes) -> int32_t {
    return (static_cast<int32_t>(bytes[0]) << 24) | (static_cast<int32_t>(bytes[1]) << 16) |
           (static_cast<int32_t>(bytes[2]) << 8) | static_cast<int32_t>(bytes[3]);
}

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_BYTE_HPP