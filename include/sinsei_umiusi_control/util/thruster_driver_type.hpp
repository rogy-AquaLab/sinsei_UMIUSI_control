#ifndef SINSEI_UMIUSI_CONTROL_UTIL_THRUSTER_DRIVER_TYPE_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_THRUSTER_DRIVER_TYPE_HPP

#include <rcpputils/tl_expected/expected.hpp>
#include <string>

namespace sinsei_umiusi_control::util {

enum class ThrusterDriverType { Can, Direct };

inline auto get_driver_type_from_str(const std::string_view & str)
    -> tl::expected<ThrusterDriverType, std::string> {
    if (str == "can") {
        return ThrusterDriverType::Can;
    } else if (str == "direct") {
        return ThrusterDriverType::Direct;
    } else {
        return tl::make_unexpected("Invalid thruster driver type: " + std::string(str));
    }
}

inline auto get_driver_type_from_str(std::string_view && str)
    -> tl::expected<ThrusterDriverType, std::string> {
    if (str == "can") {
        return ThrusterDriverType::Can;
    } else if (str == "direct") {
        return ThrusterDriverType::Direct;
    } else {
        return tl::make_unexpected("Invalid thruster driver type: " + std::string(str));
    }
}

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_THRUSTER_DRIVER_TYPE_HPP
