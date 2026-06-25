#ifndef SINSEI_UMIUSI_CONTROL_UTIL_VECTOR_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_VECTOR_HPP

#include <vector>

namespace sinsei_umiusi_control::util {

// std::vectorの要素の型を変換する
template <typename To, typename From>
inline auto cast_vector(const std::vector<From> & from) -> std::vector<To> {
    std::vector<To> to;
    to.reserve(from.size());
    for (const auto & value : from) {
        to.push_back(static_cast<To>(value));
    }
    return to;
}

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_VECTOR_HPP
