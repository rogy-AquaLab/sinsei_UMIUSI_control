#ifndef SINSEI_UMIUSI_CONTROL_UTIL_BYTE_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_BYTE_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <type_traits>

namespace sinsei_umiusi_control::util {

// Convert integer to 8-byte array in big-endian order
template <typename Int>
inline auto to_bytes_be(Int value) -> std::array<std::byte, 8> {
    static_assert(
        std::is_integral_v<Int> && !std::is_same_v<Int, bool>, "Int must be an integer type");

    // 符号付き整数の右シフトを避けるため、符号なしに変換してから分解する
    const auto raw = static_cast<std::make_unsigned_t<Int>>(value);
    auto bytes = std::array<std::byte, 8>{};
    for (std::size_t i = 0; i < sizeof(Int); ++i) {
        bytes[i] = std::byte(static_cast<uint8_t>(raw >> (8 * (sizeof(Int) - 1 - i))));
    }
    return bytes;
}

// Convert `sizeof(Int)` bytes from `offset` of 8-byte array in big-endian order to integer
template <typename Int>
inline auto to_integer_be(const std::array<std::byte, 8> & bytes, std::size_t offset = 0)
    -> std::optional<Int> {
    static_assert(
        std::is_integral_v<Int> && !std::is_same_v<Int, bool>, "Int must be an integer type");

    if (offset + sizeof(Int) > bytes.size()) {
        return std::nullopt;
    }
    // 符号付き整数の左シフトを避けるため、符号なしで組み立ててから変換する
    using UInt = std::make_unsigned_t<Int>;
    UInt raw = 0;
    for (std::size_t i = 0; i < sizeof(Int); ++i) {
        raw = static_cast<UInt>(raw << 8) | std::to_integer<uint8_t>(bytes[offset + i]);
    }
    return static_cast<Int>(raw);
}

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_BYTE_HPP
