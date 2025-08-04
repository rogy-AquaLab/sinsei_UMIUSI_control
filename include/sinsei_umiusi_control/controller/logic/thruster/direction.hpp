#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_THRUSTER_DIRECTION_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_THRUSTER_DIRECTION_HPP

#include <rcpputils/tl_expected/expected.hpp>
#include <string>
#include <string_view>

namespace sinsei_umiusi_control::controller::logic::thruster {

enum class Direction { RightHanded, LeftHanded };

auto get_direction_from_str(const std::string_view & str) -> tl::expected<Direction, std::string> {
    if (str == "rh") {
        return Direction::RightHanded;
    } else if (str == "lh") {
        return Direction::LeftHanded;
    } else {
        return tl::make_unexpected("Invalid thruster direction: " + std::string(str));
    }
}

}  // namespace sinsei_umiusi_control::controller::logic::thruster

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_THRUSTER_DIRECTION_HPP