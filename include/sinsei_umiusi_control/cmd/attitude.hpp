#ifndef SINSEI_UMIUSI_CONTROL_CMD_ATTITUDE_HPP
#define SINSEI_UMIUSI_CONTROL_CMD_ATTITUDE_HPP

namespace sinsei_umiusi_control::cmd::attitude {

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

}  // namespace sinsei_umiusi_control::cmd::attitude

#endif  // SINSEI_UMIUSI_CONTROL_CMD_ATTITUDE_HPP
