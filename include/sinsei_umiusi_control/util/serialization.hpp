#ifndef SINSEI_UMIUSI_CONTROL_UTIL_SERIALIZATION_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_SERIALIZATION_HPP

#include <cstring>
#include <type_traits>

namespace sinsei_umiusi_control::util {

using InterfaceData = double;

template <typename T>
inline auto from_interface_data(InterfaceData & value) -> T {
    static_assert(
        sizeof(T) <= sizeof(InterfaceData),
        "T must be smaller than or equal to InterfaceData (double)");
    static_assert(std::is_trivially_copyable<T>::value, "T must be trivially copyable");
    T result;
    std::memcpy(&result, &value, sizeof(T));
    return result;
}

template <typename T>
inline auto to_interface_data(const T & value) -> InterfaceData {
    static_assert(
        sizeof(T) <= sizeof(InterfaceData),
        "T must be smaller than or equal to InterfaceData (double)");
    static_assert(std::is_trivially_copyable<T>::value, "T must be trivially copyable");
    InterfaceData result;
    std::memcpy(&result, &value, sizeof(InterfaceData));
    return result;
}

template <typename T>
inline auto to_interface_data_ptr(T & value) -> InterfaceData * {
    static_assert(
        sizeof(T) <= sizeof(InterfaceData),
        "T must be smaller than or equal to InterfaceData (double)");
    return reinterpret_cast<InterfaceData *>(&value);
}

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_SERIALIZATION_HPP