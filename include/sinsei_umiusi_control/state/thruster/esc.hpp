#ifndef SINSEI_UMIUSI_CONTROL_STATE_THRUSTER_ESC_HPP
#define SINSEI_UMIUSI_CONTROL_STATE_THRUSTER_ESC_HPP

#include "sinsei_umiusi_control/util/thruster_mode.hpp"

namespace sinsei_umiusi_control::state::thruster::esc {

struct Mode {
    util::ThrusterMode value;
};
struct DutyCycle {
    double value;
};
struct Rpm {
    double value;
};
struct Voltage {
    double value;
};
struct WaterLeaked {
    bool value;
};
struct Health {
    bool is_ok;
};

}  // namespace sinsei_umiusi_control::state::thruster::esc

#endif  // SINSEI_UMIUSI_CONTROL_STATE_THRUSTER_ESC_HPP
