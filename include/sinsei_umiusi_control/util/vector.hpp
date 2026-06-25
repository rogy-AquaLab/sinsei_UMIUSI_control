#ifndef SINSEI_UMIUSI_CONTROL_UTIL_VECTOR_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_VECTOR_HPP

#include <type_traits>
#include <vector>

namespace sinsei_umiusi_control::util {

// 型Tからconst参照と参照を取り除いた型
template <typename T>
using remove_cvref_t = std::remove_cv_t<std::remove_reference_t<T>>;

template <typename From, typename Func>
inline auto map_vector(const std::vector<From> & from, Func && func) {
    using Mapped = std::invoke_result_t<Func &, const From &>;
    using To = remove_cvref_t<Mapped>;

    std::vector<To> to;
    to.reserve(from.size());
    for (const auto & value : from) {
        to.push_back(func(value));
    }
    return to;
}

// std::vectorの要素の型を変換する
template <typename To, typename From>
inline auto cast_vector(const std::vector<From> & from) -> std::vector<To> {
    return map_vector(from, [](const From & value) { return static_cast<To>(value); });
}

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_VECTOR_HPP
