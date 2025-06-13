#ifndef SINSEI_UMIUSI_CONTROL_CMD_APP_HPP
#define SINSEI_UMIUSI_CONTROL_CMD_APP_HPP

namespace sinsei_umiusi_control::cmd::app {

struct Orientation {
    float x;
    float y;
    float z;
};
struct Velocity {
    float x;
    float y;
    float z;
};

}  // namespace sinsei_umiusi_control::cmd::app

#endif  // SINSEI_UMIUSI_CONTROL_CMD_APP_HPP