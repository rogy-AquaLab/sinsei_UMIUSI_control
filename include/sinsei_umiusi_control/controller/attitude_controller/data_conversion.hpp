#ifndef SINSEI_UMIUSI_CONTROL_ATTITUDE_CONTROLLER_DATA_CONVERSION_HPP
#define SINSEI_UMIUSI_CONTROL_ATTITUDE_CONTROLLER_DATA_CONVERSION_HPP

#include <Eigen/Geometry>

#include "sinsei_umiusi_control/cmd/attitude.hpp"
#include "sinsei_umiusi_control/cmd/thruster.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"

namespace sinsei_umiusi_control::controller::attitude_controller {

inline auto to_eigen_quaternion(const state::imu::Quaternion & q) -> Eigen::Quaterniond {
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

inline auto to_eigen_vector(const cmd::attitude::Orientation & o) -> Eigen::Vector3d {
    return Eigen::Vector3d(o.x, o.y, o.z);
}

inline auto to_eigen_vector(const cmd::attitude::Velocity & v) -> Eigen::Vector3d {
    return Eigen::Vector3d(v.x, v.y, v.z);
}

struct ThrusterCmds {
    struct Cmd {
        cmd::thruster::DutyCycle duty_cycle;
        cmd::thruster::Angle angle;
    };

    Cmd lf;  // Left Front
    Cmd lb;  // Left Back
    Cmd rb;  // Right Back
    Cmd rf;  // Right Front
};

/// `LF` `LB` `RB` `RF` の順番で、 `Eigen::Vector` から `ThrusterCmds` を生成する
inline auto from_eigen_vector(const Eigen::Vector<double, 8> & v) -> ThrusterCmds {
    return ThrusterCmds{
        {          // Left Front
         {v[0]},   //   - duty_cycle
         {v[1]}},  //   - angle
        {          // Left Back
         {v[2]},   //   - duty_cycle
         {v[3]}},  //   - angle
        {          // Right Back
         {v[4]},   //   - duty_cycle
         {v[5]}},  //   - angle
        {          // Right Front
         {v(6)},   //   - duty_cycle
         {v(7)}},  //   - angle
    };
}

}  // namespace sinsei_umiusi_control::controller::attitude_controller

#endif  // SINSEI_UMIUSI_CONTROL_ATTITUDE_CONTROLLER_DATA_CONVERSION_HPP
