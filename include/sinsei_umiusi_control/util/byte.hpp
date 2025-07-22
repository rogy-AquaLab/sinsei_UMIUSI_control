#ifndef SINSEI_UMIUSI_CONTROL_UTIL_BYTE_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_BYTE_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace sinsei_umiusi_control::util {

// Convert int64_t to 8-byte array in big-endian order
inline auto to_bytes_be(int64_t value) -> std::array<std::byte, 8> {
    return {
        std::byte(value >> 24), std::byte(value >> 16), std::byte(value >> 8), std::byte(value)};
}

// Convert 8-byte array in big-endian order to int32_t
inline auto to_int32_be(std::array<std::byte, 8> bytes) -> int32_t {
    return (std::to_integer<int32_t>(bytes[0]) << 24) | (std::to_integer<int32_t>(bytes[1]) << 16) |
           (std::to_integer<int32_t>(bytes[2]) << 8) | std::to_integer<int32_t>(bytes[3]);
}

// Convert 8-byte array in big-endian order to int64_t
inline auto to_int64_be(std::array<std::byte, 8> bytes) -> int64_t {
    return (std::to_integer<int64_t>(bytes[0]) << 56) | (std::to_integer<int64_t>(bytes[1]) << 48) |
           (std::to_integer<int64_t>(bytes[2]) << 40) | (std::to_integer<int64_t>(bytes[3]) << 32) |
           (std::to_integer<int64_t>(bytes[4]) << 24) | (std::to_integer<int64_t>(bytes[5]) << 16) |
           (std::to_integer<int64_t>(bytes[6]) << 8) | std::to_integer<int64_t>(bytes[7]);
}

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_BYTE_HPP