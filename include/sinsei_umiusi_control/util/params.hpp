#ifndef SINSEI_UMIUSI_CONTROL_UTIL_PARAMS_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_PARAMS_HPP

#include <optional>
#include <string>
#include <unordered_map>

namespace sinsei_umiusi_control::util {

using HardwareParamaters = std::unordered_map<std::string, std::string>;

inline auto find_param(const HardwareParamaters & params, const std::string & key)
    -> std::optional<std::string> {
    auto it = params.find(key);
    if (it == params.end()) {
        return std::nullopt;
    }
    return it->second;
}

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_PARAMS_HPP
