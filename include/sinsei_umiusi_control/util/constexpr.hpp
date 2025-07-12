#ifndef SINSEI_UMIUSI_CONTROL_UTIL_CONSTEXPR_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_CONSTEXPR_HPP

#include <cstddef>

namespace sinsei_umiusi_control::util {

// ref: https://stackoverflow.com/questions/51114180/constexpr-find-for-array-using-c17

// `constexpr`に対応した文字列比較関数
constexpr bool equals(const char * a, const char * b) {
    for (std::size_t i = 0;; ++i) {
        if (a[i] != b[i]) return false;
        if (a[i] == 0) break;
    }
    return true;
}

// `constexpr`に対応した文字列のインデックス取得関数
template <std::size_t size>
constexpr auto get_index(const char * name, const char * const (&array)[size]) -> std::size_t {
    for (size_t i = 0; i < size; ++i) {
        if (equals(array[i], name)) {
            return i;
        }
    }
    return size;
}

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_CONSTEXPR_HPP