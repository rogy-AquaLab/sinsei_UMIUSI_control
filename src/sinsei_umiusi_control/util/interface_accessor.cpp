#include "sinsei_umiusi_control/util/interface_accessor.hpp"

#include <rcpputils/tl_expected/expected.hpp>

using namespace sinsei_umiusi_control::util;

auto interface_accessor::get_states_from_loaned_interfaces(
    const std::vector<hardware_interface::LoanedStateInterface> & source,
    InterfaceDataContainer & dest) -> bool {
    bool success = false;

    for (size_t i = 0; i < dest.size(); ++i) {
        auto & [name, data] = dest[i];
        auto res = source.at(i).get_optional();
        if (!res) {
            continue;
        }
        success = true;
        *data = res.value();
    }

    return success;
}

auto interface_accessor::set_commands_to_loaned_interfaces(
    std::vector<hardware_interface::LoanedCommandInterface> & dest,
    const InterfaceDataContainer & source) -> bool {
    bool success = false;

    for (size_t i = 0; i < source.size(); ++i) {
        const auto & [name, data] = source[i];
        auto res = dest.at(i).set_value(*data);
        success = success || res;
    }

    return success;
}
