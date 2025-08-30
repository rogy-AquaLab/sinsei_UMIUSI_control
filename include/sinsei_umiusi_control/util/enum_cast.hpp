#ifndef SINSEI_UMIUSI_CONTROL_UTIL_ENUM_CAST_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_ENUM_CAST_HPP

#include <rcpputils/tl_expected/expected.hpp>

namespace sinsei_umiusi_control::util {

enum class EnumCastError { InvalidValue };

template <typename Int, typename Enum>
constexpr auto enum_cast(const Int &) -> tl::expected<Enum, EnumCastError> {
    static_assert(false, "`enum_cast` is not implemented for these types");
}

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_ENUM_CAST_HPP
