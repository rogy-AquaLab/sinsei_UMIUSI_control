#ifndef SINSEI_UMIUSI_CONTROL_ATTITUDE_CONTROLLER_DATA_CONVERSION_HPP
#define SINSEI_UMIUSI_CONTROL_ATTITUDE_CONTROLLER_DATA_CONVERSION_HPP

#include <Eigen/Geometry>

#include "sinsei_umiusi_control/cmd/attitude.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"

namespace sinsei_umiusi_control::controller::logic::attitude {

inline auto to_eigen_quaternion(const state::imu::Quaternion & q) -> Eigen::Quaterniond {
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

inline auto to_eigen_vector(const cmd::attitude::Orientation & o) -> Eigen::Vector3d {
    return Eigen::Vector3d(o.x, o.y, o.z);
}

inline auto to_eigen_vector(const cmd::attitude::Velocity & v) -> Eigen::Vector3d {
    return Eigen::Vector3d(v.x, v.y, v.z);
}

}  // namespace sinsei_umiusi_control::controller::logic::attitude

#endif  // SINSEI_UMIUSI_CONTROL_ATTITUDE_CONTROLLER_DATA_CONVERSION_HPP
