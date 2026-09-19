#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_FEED_BACK_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_FEED_BACK_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sinsei_umiusi_control/controller/attitude_controller.hpp"
#include "sinsei_umiusi_control/controller/logic/attitude/attitude_feedback.hpp"
#include "sinsei_umiusi_control/controller/logic/attitude/data_conversion.hpp"
#include "sinsei_umiusi_control/controller/logic/attitude/mixer.hpp"

namespace sinsei_umiusi_control::controller::logic::attitude {

// IMU実測姿勢との誤差から、roll/pitch の姿勢PDとyawレート制御を行う。
class FeedBack : public AttitudeController::Logic {
  private:
    AttitudeFeedback attitude_feedback;

  public:
    auto control_mode() const -> logic::ControlMode override {
        return logic::ControlMode::FeedBack;
    }

    auto init(
        double /*time*/, const AttitudeController::Input & /*input*/,
        const AttitudeController::Output & /*output*/) -> AttitudeController::Output override {
        return {};
    }

    auto update(double /*time*/, double duration, const AttitudeController::Input & input)
        -> AttitudeController::Output override {
        const auto target_attitude = to_eigen_quaternion(input.cmd.target_attitude);
        const auto current_attitude = to_eigen_quaternion(input.state.imu_quaternion);
        const auto target_velocity = to_eigen_vector(input.cmd.target_velocity);
        const auto angular_velocity = Eigen::Vector3d{
            input.state.imu_angular_velocity.x,
            input.state.imu_angular_velocity.y,
            input.state.imu_angular_velocity.z,
        };
        const auto moment = this->attitude_feedback.moment(
            target_attitude, current_attitude, angular_velocity,
            input.cmd.target_attitude.yaw_rate, duration);
        if (!moment || !target_velocity.allFinite()) {
            return hold_current_servo_angles(input.state.servo_estimated_angles);
        }

        const auto u = Eigen::Vector<double, 6>{
            moment->x(),         // rollモーメント要求
            moment->y(),         // pitchモーメント要求
            moment->z(),         // yawモーメント要求
            target_velocity[0],  // 目標並進(x)
            target_velocity[1],  // 目標並進(y)
            target_velocity[2],  // 目標並進(z)
        };

        return mix_to_thrusters(
            u, input.state.servo_estimated_angles, input.state.servo_max_angular_velocities,
            duration);
    }
};

}  // namespace sinsei_umiusi_control::controller::logic::attitude

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_FEED_BACK_HPP
