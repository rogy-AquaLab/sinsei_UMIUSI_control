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

auto ImuModel::on_read()
    -> tl::expected<
        std::tuple<state::imu::Quaternion, state::imu::Velocity, state::imu::Temperature>,
        std::string> {
    const auto quaternion_res = this->bno055_model.get_quad();
    if (!quaternion_res) {
        return tl::make_unexpected(
            "Failed to read quaternion data from BNO055: " + quaternion_res.error());
    }

    // FIXME: dummy
    const state::imu::Velocity velocity{0.0, 0.0, 0.0};

    const auto temp_res = this->bno055_model.get_temp();
    if (!temp_res) {
        return tl::make_unexpected(
            "Failed to read temperature data from BNO055: " + temp_res.error());
    }

    return std::make_tuple(quaternion_res.value(), velocity, temp_res.value());
}
