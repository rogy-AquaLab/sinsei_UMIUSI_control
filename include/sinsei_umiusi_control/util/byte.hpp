#ifndef SINSEI_UMIUSI_CONTROL_UTIL_BYTE_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_BYTE_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace sinsei_umiusi_control::util {

// Convert int64_t to 8-byte array in big-endian order
template <typename Int>
inline auto to_bytes_be(Int value) -> std::array<std::byte, 8>;

template <>
inline auto to_bytes_be(int8_t value) -> std::array<std::byte, 8> {
    return {std::byte(value)};
}

template <>
inline auto to_bytes_be(int16_t value) -> std::array<std::byte, 8> {
    return {std::byte(value >> 8), std::byte(value)};
}

template <>
inline auto to_bytes_be(int32_t value) -> std::array<std::byte, 8> {
    return {
        std::byte(value >> 24), std::byte(value >> 16), std::byte(value >> 8), std::byte(value)};
}

template <>
inline auto to_bytes_be(int64_t value) -> std::array<std::byte, 8> {
    return {std::byte(value >> 56), std::byte(value >> 48), std::byte(value >> 40),
            std::byte(value >> 32), std::byte(value >> 24), std::byte(value >> 16),
            std::byte(value >> 8),  std::byte(value)};
}

// Convert 8-byte array in big-endian order to int16_t
template <size_t OFFSET = 0>
inline auto to_int16_be(const std::array<std::byte, 8> & bytes) -> std::optional<int16_t> {
    static_assert(OFFSET + 2 <= 8, "Offset out of range");
    const auto raw = static_cast<uint16_t>(
        (static_cast<uint16_t>(std::to_integer<uint8_t>(bytes[OFFSET])) << 8) |
        std::to_integer<uint8_t>(bytes[OFFSET + 1]));
    return static_cast<int16_t>(raw);
}

// Convert 8-byte array in big-endian order to int32_t
template <size_t OFFSET = 0>
inline auto to_int32_be(const std::array<std::byte, 8> & bytes) -> std::optional<int32_t> {
    static_assert(OFFSET + 4 <= 8, "Offset out of range");
    const auto raw = (static_cast<uint32_t>(std::to_integer<uint8_t>(bytes[OFFSET])) << 24) |
                     (static_cast<uint32_t>(std::to_integer<uint8_t>(bytes[OFFSET + 1])) << 16) |
                     (static_cast<uint32_t>(std::to_integer<uint8_t>(bytes[OFFSET + 2])) << 8) |
                     std::to_integer<uint8_t>(bytes[OFFSET + 3]);
    return static_cast<int32_t>(raw);
}

// Convert 8-byte array in big-endian order to int64_t
template <size_t OFFSET = 0>
inline auto to_int64_be(const std::array<std::byte, 8> & bytes) -> std::optional<int64_t> {
    static_assert(OFFSET + 8 <= 8, "Offset out of range");
    const auto raw = (static_cast<uint64_t>(std::to_integer<uint8_t>(bytes[OFFSET])) << 56) |
                     (static_cast<uint64_t>(std::to_integer<uint8_t>(bytes[OFFSET + 1])) << 48) |
                     (static_cast<uint64_t>(std::to_integer<uint8_t>(bytes[OFFSET + 2])) << 40) |
                     (static_cast<uint64_t>(std::to_integer<uint8_t>(bytes[OFFSET + 3])) << 32) |
                     (static_cast<uint64_t>(std::to_integer<uint8_t>(bytes[OFFSET + 4])) << 24) |
                     (static_cast<uint64_t>(std::to_integer<uint8_t>(bytes[OFFSET + 5])) << 16) |
                     (static_cast<uint64_t>(std::to_integer<uint8_t>(bytes[OFFSET + 6])) << 8) |
                     std::to_integer<uint8_t>(bytes[OFFSET + 7]);
    return static_cast<int64_t>(raw);
}

inline auto to_int16_be(const std::array<std::byte, 8> & bytes, std::size_t offset)
    -> std::optional<int16_t> {
    if (offset + 2 > bytes.size()) {
        return std::nullopt;
    }
    const auto raw = static_cast<uint16_t>(
        (static_cast<uint16_t>(std::to_integer<uint8_t>(bytes[offset])) << 8) |
        std::to_integer<uint8_t>(bytes[offset + 1]));
    return static_cast<int16_t>(raw);
}

inline auto to_uint32_be(const std::array<std::byte, 8> & bytes, std::size_t offset = 0)
    -> std::optional<uint32_t> {
    if (offset + 4 > bytes.size()) {
        return std::nullopt;
    }
    return (static_cast<uint32_t>(std::to_integer<uint8_t>(bytes[offset])) << 24) |
           (static_cast<uint32_t>(std::to_integer<uint8_t>(bytes[offset + 1])) << 16) |
           (static_cast<uint32_t>(std::to_integer<uint8_t>(bytes[offset + 2])) << 8) |
           std::to_integer<uint8_t>(bytes[offset + 3]);
}

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_BYTE_HPP
