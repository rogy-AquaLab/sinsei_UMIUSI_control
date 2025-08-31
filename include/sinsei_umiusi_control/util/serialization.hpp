#ifndef SINSEI_UMIUSI_CONTROL_UTIL_SERIALIZATION_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_SERIALIZATION_HPP

#include <cstring>
#include <type_traits>
#include <utility>

namespace sinsei_umiusi_control::util {

// ros2_controlでは、Command / State Interfaceの中身に`double`型しか使えない
using InterfaceData = double;

template <typename T>
inline auto from_interface_data(const InterfaceData & value) -> T {
    static_assert(
        sizeof(T) <= sizeof(InterfaceData),
        "T must be smaller than or equal to InterfaceData (double)");
    static_assert(std::is_trivially_copyable<T>::value, "T must be trivially copyable");
    T result;
    std::memcpy(&result, &value, sizeof(T));
    return result;
}

template <typename T>
inline auto from_interface_data(InterfaceData && value) -> T {
    static_assert(
        sizeof(T) <= sizeof(InterfaceData),
        "T must be smaller than or equal to InterfaceData (double)");
    static_assert(std::is_trivially_copyable<T>::value, "T must be trivially copyable");
    T result;
    auto && src = std::move(value);
    std::memcpy(&result, &src, sizeof(T));
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
inline auto to_interface_data(T && value) -> InterfaceData {
    static_assert(
        sizeof(T) <= sizeof(InterfaceData),
        "T must be smaller than or equal to InterfaceData (double)");
    static_assert(std::is_trivially_copyable<T>::value, "T must be trivially copyable");
    InterfaceData result;
    auto && src = std::forward(value);
    std::memcpy(&result, &src, sizeof(InterfaceData));
    return result;
}

template <typename T>
inline auto to_interface_data_ptr(const T & value) -> const InterfaceData * {
    static_assert(
        sizeof(T) <= sizeof(InterfaceData),
        "T must be smaller than or equal to InterfaceData (double)");
    return reinterpret_cast<const InterfaceData *>(&value);
}

template <typename T>
inline auto to_interface_data_ptr(T && value) -> InterfaceData * {
    static_assert(
        sizeof(T) <= sizeof(InterfaceData),
        "T must be smaller than or equal to InterfaceData (double)");
    return reinterpret_cast<InterfaceData *>(&value);
}

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_SERIALIZATION_HPP
