#ifndef SINSEI_UMIUSI_CONTROL_UTIL_SERIALIZATION_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_SERIALIZATION_HPP

#include <cstring>
#include <type_traits>

namespace sinsei_umiusi_control::util {

using InterfaceData = double;

template <typename NewType>
inline auto from_interface_data(InterfaceData & value) -> NewType {
    static_assert(
        sizeof(NewType) <= sizeof(InterfaceData),
        "NewType must be smaller than or equal to InterfaceData (double)");
    static_assert(std::is_trivially_copyable<NewType>::value, "NewType must be trivially copyable");
    NewType result;
    std::memcpy(&result, &value, sizeof(NewType));
    return result;
}

template <typename NewType>
inline auto to_interface_data(const NewType & value) -> InterfaceData {
    static_assert(
        sizeof(NewType) <= sizeof(InterfaceData),
        "NewType must be smaller than or equal to InterfaceData (double)");
    static_assert(std::is_trivially_copyable<NewType>::value, "NewType must be trivially copyable");
    InterfaceData result;
    std::memcpy(&result, &value, sizeof(InterfaceData));
    return result;
}

template <typename NewType>
inline auto to_interface_data_ptr(NewType & value) -> InterfaceData * {
    static_assert(
        sizeof(NewType) <= sizeof(InterfaceData),
        "NewType must be smaller than or equal to InterfaceData (double)");
    return reinterpret_cast<InterfaceData *>(&value);
}

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_SERIALIZATION_HPP