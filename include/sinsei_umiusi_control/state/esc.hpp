#ifndef SINSEI_UMIUSI_CONTROL_STATE_ESC_HPP
#define SINSEI_UMIUSI_CONTROL_STATE_ESC_HPP

namespace sinsei_umiusi_control::state::esc {

struct Voltage {
    double value;
};
struct WaterLeaked {
    bool value;
};

}  // namespace sinsei_umiusi_control::state::esc

#endif  // SINSEI_UMIUSI_CONTROL_STATE_ESC_HPP
