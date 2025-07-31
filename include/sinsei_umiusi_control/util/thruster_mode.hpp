#ifndef SINSEI_UMIUSI_CONTROL_UTIL_THRUSTER_MODE_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_THRUSTER_MODE_HPP

#include <rcpputils/tl_expected/expected.hpp>
#include <string>

namespace sinsei_umiusi_control::util {

enum class ThrusterMode { Can, Direct };

inline auto get_mode_from_str(const std::string & str) -> tl::expected<ThrusterMode, std::string> {
    if (str == "can") {
        return ThrusterMode::Can;
    } else if (str == "direct") {
        return ThrusterMode::Direct;
    } else {
        return tl::make_unexpected("Invalid thruster mode: " + str);
    }
}

inline auto get_mode_from_str(std::string && str) -> tl::expected<ThrusterMode, std::string> {
    if (str == "can") {
        return ThrusterMode::Can;
    } else if (str == "direct") {
        return ThrusterMode::Direct;
    } else {
        return tl::make_unexpected("Invalid thruster mode: " + str);
    }
}

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_THRUSTER_MODE_HPP