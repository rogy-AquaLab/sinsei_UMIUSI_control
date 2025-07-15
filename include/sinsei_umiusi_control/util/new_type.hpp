#ifndef SINSEI_UMIUSI_CONTROL_UTIL_NEW_TYPE_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_NEW_TYPE_HPP

#include <cstring>
#include <type_traits>

namespace sinsei_umiusi_control::util {

template <typename NewType>
inline auto to_new_type(double & value) -> NewType {
    static_assert(
        sizeof(NewType) <= sizeof(double), "NewType must be smaller than or equal to double");
    static_assert(std::is_trivially_copyable<NewType>::value, "NewType must be trivially copyable");
    NewType result;
    std::memcpy(&result, &value, sizeof(NewType));
    return result;
}

template <typename NewType>
inline auto to_double(const NewType & value) -> double {
    static_assert(
        sizeof(NewType) <= sizeof(double), "NewType must be smaller than or equal to double");
    static_assert(std::is_trivially_copyable<NewType>::value, "NewType must be trivially copyable");
    double result;
    std::memcpy(&result, &value, sizeof(double));
    return result;
}

template <typename NewType>
inline auto to_double_ptr(NewType & value) -> double * {
    static_assert(
        sizeof(NewType) <= sizeof(double), "NewType must be smaller than or equal to double");
    return reinterpret_cast<double *>(&value);
}

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_NEW_TYPE_HPP