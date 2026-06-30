#ifndef SINSEI_UMIUSI_CONTROL_UTIL_STRING_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_STRING_HPP

#include <charconv>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

namespace sinsei_umiusi_control::util {

template <typename T>
auto from_chars_expected(std::string_view str) -> tl::expected<T, std::string> {
    auto value = T{};
    const auto * begin = str.data();
    const auto * end = begin + str.size();

    const auto [ptr, ec] = std::from_chars(begin, end, value);
    if (ec != std::errc{}) {
        if (ec == std::errc::invalid_argument) {
            return tl::make_unexpected("Invalid number format: '" + std::string(str) + "'");
        }
        if (ec == std::errc::result_out_of_range) {
            return tl::make_unexpected("Number out of range: '" + std::string(str) + "'");
        }
        return tl::make_unexpected("Unknown parse error occurred");
    }
    if (ptr != end) {
        return tl::make_unexpected("Trailing characters found in: '" + std::string(str) + "'");
    }

    return value;
}

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_STRING_HPP
