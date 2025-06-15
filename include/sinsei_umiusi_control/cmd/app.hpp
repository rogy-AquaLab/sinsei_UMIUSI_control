#ifndef SINSEI_UMIUSI_CONTROL_CMD_APP_HPP
#define SINSEI_UMIUSI_CONTROL_CMD_APP_HPP

namespace sinsei_umiusi_control::cmd::app {

struct Orientation {
    double x;
    double y;
    double z;
};
struct Velocity {
    double x;
    double y;
    double z;
};

}  // namespace sinsei_umiusi_control::cmd::app

#endif  // SINSEI_UMIUSI_CONTROL_CMD_APP_HPP