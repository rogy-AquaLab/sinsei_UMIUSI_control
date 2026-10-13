#ifndef SINSEI_UMIUSI_CONTROL_UTIL_BYTE_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_BYTE_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace sinsei_umiusi_control::util {

// Convert integer to 8-byte array in big-endian order
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

// Read `sizeof(UInt)` bytes from `offset` of 8-byte array in big-endian order
template <typename UInt>
inline auto to_uint_be(const std::array<std::byte, 8> & bytes, std::size_t offset)
    -> std::optional<UInt> {
    if (offset + sizeof(UInt) > bytes.size()) {
        return std::nullopt;
    }
    UInt value = 0;
    for (std::size_t i = 0; i < sizeof(UInt); ++i) {
        value = static_cast<UInt>(value << 8) | std::to_integer<uint8_t>(bytes[offset + i]);
    }
    return value;
}

// Convert 1 byte at `offset` of 8-byte array to uint8_t
inline auto to_uint8(const std::array<std::byte, 8> & bytes, std::size_t offset = 0)
    -> std::optional<uint8_t> {
    return to_uint_be<uint8_t>(bytes, offset);
}

// Convert 8-byte array in big-endian order to int16_t
inline auto to_int16_be(const std::array<std::byte, 8> & bytes, std::size_t offset = 0)
    -> std::optional<int16_t> {
    const auto raw = to_uint_be<uint16_t>(bytes, offset);
    if (!raw) {
        return std::nullopt;
    }
    return static_cast<int16_t>(raw.value());
}

// Convert 8-byte array in big-endian order to int32_t
inline auto to_int32_be(const std::array<std::byte, 8> & bytes, std::size_t offset = 0)
    -> std::optional<int32_t> {
    const auto raw = to_uint_be<uint32_t>(bytes, offset);
    if (!raw) {
        return std::nullopt;
    }
    return static_cast<int32_t>(raw.value());
}

// Convert 8-byte array in big-endian order to int64_t
inline auto to_int64_be(const std::array<std::byte, 8> & bytes, std::size_t offset = 0)
    -> std::optional<int64_t> {
    const auto raw = to_uint_be<uint64_t>(bytes, offset);
    if (!raw) {
        return std::nullopt;
    }
    return static_cast<int64_t>(raw.value());
}

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_BYTE_HPP
