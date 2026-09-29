#ifndef SINSEI_UMIUSI_CONTROL_CMD_ATTITUDE_HPP
#define SINSEI_UMIUSI_CONTROL_CMD_ATTITUDE_HPP

namespace sinsei_umiusi_control::cmd::attitude {

struct AttitudeTarget {
    // Target attitude quaternion; feedback control uses its body-up direction and ignores yaw.
    double x;
    double y;
    double z;
    double w;
    double yaw_rate;  // [rad/s]
    // true: hold the heading latched at the false -> true edge. yaw_rate then slews that heading.
    // false: plain yaw rate control, no state. Defaults to false so the behaviour is unchanged.
    bool hold_yaw;
};
struct Velocity {
    double x;
    double y;
    double z;
};

}  // namespace sinsei_umiusi_control::cmd::attitude

#endif  // SINSEI_UMIUSI_CONTROL_CMD_ATTITUDE_HPP
