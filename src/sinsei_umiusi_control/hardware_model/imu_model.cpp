#include "sinsei_umiusi_control/hardware_model/imu_model.hpp"

#include "sinsei_umiusi_control/state/imu.hpp"

using namespace sinsei_umiusi_control::hardware_model;

ImuModel::ImuModel(std::unique_ptr<interface::Gpio> gpio) : bno055_model(std::move(gpio)) {}

auto ImuModel::on_init() -> tl::expected<void, std::string> {
    auto res = this->bno055_model.begin();
    if (!res) {
        return tl::make_unexpected("Failed to initialize BNO055: " + res.error());
    }

    return {};
}

auto ImuModel::on_destroy() -> tl::expected<void, std::string> {
    auto res = this->bno055_model.close();
    if (!res) {
        return tl::make_unexpected("Failed to close BNO055: " + res.error());
    }

    return {};
}

auto ImuModel::on_read() -> tl::expected<
                             std::tuple<
                                 state::imu::Quaternion, state::imu::Acceleration,
                                 state::imu::AngularVelocity, state::imu::Temperature>,
                             std::string> {
    const auto quaternion_res = this->bno055_model.get_quat();
    if (!quaternion_res) {
        return tl::make_unexpected(
            "Failed to read quaternion data from BNO055: " + quaternion_res.error());
    }

    const auto acceleration_res = this->bno055_model.get_acceleration();
    if (!acceleration_res) {
        return tl::make_unexpected(
            "Failed to read acceleration data from BNO055: " + acceleration_res.error());
    }

    const auto angular_velocity_res = this->bno055_model.get_angular_velocity();
    if (!angular_velocity_res) {
        return tl::make_unexpected(
            "Failed to read angular velocity data from BNO055: " + angular_velocity_res.error());
    }

    const auto temp_res = this->bno055_model.get_temp();
    if (!temp_res) {
        return tl::make_unexpected(
            "Failed to read temperature data from BNO055: " + temp_res.error());
    }

    return std::make_tuple(
        quaternion_res.value(), acceleration_res.value(), angular_velocity_res.value(),
        temp_res.value());
}
