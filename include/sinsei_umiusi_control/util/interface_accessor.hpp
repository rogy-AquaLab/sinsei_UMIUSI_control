#ifndef SINSEI_UMIUSI_CONTROL_UTIL_INTERFACE_ACCESSOR_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_INTERFACE_ACCESSOR_HPP

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rcpputils/tl_expected/expected.hpp>
#include <vector>

#include "sinsei_umiusi_control/util/serialization.hpp"

namespace sinsei_umiusi_control::util::interface_accessor {

using InterfaceDataContainer = std::vector<std::pair<std::string, InterfaceData *>>;

auto get_states_from_loaned_interfaces(
    const std::vector<hardware_interface::LoanedStateInterface> & source,
    InterfaceDataContainer & dest) -> bool;

auto set_commands_to_loaned_interfaces(
    std::vector<hardware_interface::LoanedCommandInterface> & dest,
    const InterfaceDataContainer & source) -> bool;

}  // namespace sinsei_umiusi_control::util::interface_accessor

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_INTERFACE_ACCESSOR_HPP
