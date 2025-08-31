#ifndef SINSEI_UMIUSI_CONTROL_CMD_HEADLIGHTS_HPP
#define SINSEI_UMIUSI_CONTROL_CMD_HEADLIGHTS_HPP

namespace sinsei_umiusi_control::cmd::headlights {

struct HighBeamEnabled {
    bool value;
};
struct LowBeamEnabled {
    bool value;
};
struct IrEnabled {
    bool value;
};

}  // namespace sinsei_umiusi_control::cmd::headlights

#endif  // SINSEI_UMIUSI_CONTROL_CMD_HEADLIGHTS_HPP
