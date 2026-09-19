#ifndef SINSEI_UMIUSI_CONTROL_CMD_ATTITUDE_HPP
#define SINSEI_UMIUSI_CONTROL_CMD_ATTITUDE_HPP

namespace sinsei_umiusi_control::cmd::attitude {

struct AttitudeTarget {
    // Target roll/pitch attitude (quaternion); yaw component should be 0
    double x;
    double y;
    double z;
    double w;
    double yaw_rate;  // [rad/s]
};
struct Velocity {
    double x;
    double y;
    double z;
};

}  // namespace sinsei_umiusi_control::cmd::attitude

#endif  // SINSEI_UMIUSI_CONTROL_CMD_ATTITUDE_HPP
