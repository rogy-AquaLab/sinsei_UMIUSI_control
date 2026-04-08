#include "sinsei_umiusi_control/hardware_model/imu_model.hpp"

#include "sinsei_umiusi_control/state/imu.hpp"

using namespace sinsei_umiusi_control::hardware_model;

ImuModel::ImuModel(std::unique_ptr<interface::I2c> i2c) : bno055_model(std::move(i2c)) {}

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
    const auto measurements_res = this->bno055_model.read_measurements();
    if (!measurements_res) {
        return tl::make_unexpected(
            "Failed to read BNO055 measurements: " + measurements_res.error());
    }

    const auto & measurements = measurements_res.value();
    return std::make_tuple(
        measurements.quaternion, measurements.acceleration, measurements.angular_velocity,
        measurements.temperature);
}
