#ifndef SINSEI_UMIUSI_CONTROL_UTIL_VECTOR_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_VECTOR_HPP

#include <algorithm>
#include <vector>

namespace sinsei_umiusi_control::util {

// std::vectorの要素の型を変換する
template <typename To, typename From>
inline auto cast_vector(const std::vector<From> & from) -> std::vector<To> {
    std::vector<To> to;
    to.reserve(from.size());
    std::transform(from.begin(), from.end(), std::back_inserter(to), [](const auto & value) {
        return static_cast<To>(value);
    });
    return to;
}

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_VECTOR_HPP
