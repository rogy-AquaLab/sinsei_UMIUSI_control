#include "sinsei_umiusi_control/hardware_model/can/vesc_model.hpp"

#include <algorithm>

#include "sinsei_umiusi_control/util/can_interface.hpp"

namespace suc = sinsei_umiusi_control;
namespace suchm = suc::hardware_model;

suchm::can::VescModel::VescModel(uint8_t id) : id(id) {}

auto suchm::can::VescModel::to_bytes_be(int32_t value) -> std::array<uint8_t, 8> {
    return {
        static_cast<uint8_t>(value >> 24), static_cast<uint8_t>(value >> 16),
        static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
}

auto suchm::can::VescModel::to_int32_be(std::array<uint8_t, 8> bytes) -> int32_t {
    return (static_cast<int32_t>(bytes[0]) << 24) | (static_cast<int32_t>(bytes[1]) << 16) |
           (static_cast<int32_t>(bytes[2]) << 8) | static_cast<int32_t>(bytes[3]);
}

auto suchm::can::VescModel::make_frame(
    VescSimpleCommandID command_id, const std::array<uint8_t, 8> & data) -> util::CanFrame {
    auto can_id = (static_cast<uint32_t>(command_id) & 0xFF) << 8 | (this->id & 0xFF);
    return util::CanFrame{can_id, 4, data, true};
}

auto suchm::can::VescModel::make_duty_frame(double duty)
    -> tl::expected<util::CanFrame, std::string> {
    if (duty < -1.0 || duty > 1.0) {
        return tl::make_unexpected(
            "Duty must be between -1.0 and 1.0 (duty: " + std::to_string(duty) + ")");
    }
    auto scaled_duty = static_cast<int32_t>(duty * SET_DUTY_SCALE);
    auto bytes = this->to_bytes_be(scaled_duty);
    return this->make_frame(VescSimpleCommandID::CAN_PACKET_SET_DUTY, bytes);
}

auto suchm::can::VescModel::make_rpm_frame(int8_t rpm)
    -> tl::expected<util::CanFrame, std::string> {
    auto scaled_rpm = static_cast<int32_t>(rpm * SET_RPM_SCALE);
    auto bytes = this->to_bytes_be(scaled_rpm);
    return this->make_frame(VescSimpleCommandID::CAN_PACKET_SET_RPM, bytes);
}

auto suchm::can::VescModel::make_servo_frame(double value)
    -> tl::expected<util::CanFrame, std::string> {
    if (value < 0.0 || value > 1.0) {
        return tl::make_unexpected(
            "Servo value must be between 0.0 and 1.0 (value: " + std::to_string(value) + ")");
    }
    auto scaled_value = static_cast<int32_t>(value * SET_SERVO_SCALE);
    auto bytes = to_bytes_be(scaled_value);
    return this->make_frame(VescSimpleCommandID::CAN_PACKET_SET_SERVO, bytes);
}

auto suchm::can::VescModel::make_servo_angle_frame(double deg)
    -> tl::expected<util::CanFrame, std::string> {
    // 0.0 ~ 180.0度の角度を0.0 ~ 1.0に変換
    if (deg < 0.0 || deg > 180.0) {
        return tl::make_unexpected(
            "Servo angle must be between 0.0 and 180.0 degrees (deg: " + std::to_string(deg) + ")");
    }
    return this->make_servo_frame(deg / 180.0);
}

auto suchm::can::VescModel::handle_frame(const util::CanFrame & frame)
    -> tl::expected<suc::state::thruster::Rpm, std::string> {
    const auto cmd_id = static_cast<uint8_t>((frame.id >> 8) & 0xFF);
    const auto vesc_id = static_cast<uint8_t>(frame.id & 0xFF);

    if (vesc_id != this->id) {
        return tl::make_unexpected(
            "Received CAN frame for different VESC ID (expected: " + std::to_string(this->id) +
            ", received: " + std::to_string(vesc_id) + ")");
    }
    if (frame.dlc != 8) {
        return tl::make_unexpected(
            "Received CAN frame with invalid DLC (expected: 8, received: " +
            std::to_string(frame.dlc) + ")");
    }

    std::array<uint8_t, 8> bytes;
    std::copy_n(frame.data.begin(), 8, bytes.begin());

    auto rpm = suc::state::thruster::Rpm{};

    switch (cmd_id) {
        case static_cast<uint8_t>(VescStatusCommandID::CAN_PACKET_STATUS): {
            auto scaled_erpm = to_int32_be(bytes);
            auto erpm = static_cast<double>(scaled_erpm) / ERPM_SCALE;
            static constexpr double BLDC_POLE_PAIR = BLDC_POLES / 2.0;
            // ERPMを極対数で割ってRPMに変換
            rpm.value = erpm / BLDC_POLE_PAIR;
            break;
        }
        default:
            return tl::make_unexpected(
                "Received CAN frame with unknown command ID: " + std::to_string(cmd_id));
    }

    return rpm;
}