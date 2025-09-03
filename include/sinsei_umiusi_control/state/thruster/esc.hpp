#ifndef SINSEI_UMIUSI_CONTROL_STATE_THRUSTER_ESC_HPP
#define SINSEI_UMIUSI_CONTROL_STATE_THRUSTER_ESC_HPP

namespace sinsei_umiusi_control::state::thruster::esc {

struct Enabled {
    bool value;
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

}  // namespace sinsei_umiusi_control::state::thruster::esc

#endif  // SINSEI_UMIUSI_CONTROL_STATE_THRUSTER_ESC_HPP
