#include "sinsei_umiusi_control/hardware_model/can/vesc_model.hpp"

#include "sinsei_umiusi_control/util/byte.hpp"
#include "sinsei_umiusi_control/util/can_interface.hpp"

namespace suc = sinsei_umiusi_control;
namespace suchm = suc::hardware_model;

suchm::can::VescModel::VescModel(uint8_t id) : id(id) {}

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
    auto bytes = suc::util::to_bytes_be(scaled_duty);
    return this->make_frame(VescSimpleCommandID::CAN_PACKET_SET_DUTY, bytes);
}

auto suchm::can::VescModel::make_rpm_frame(int8_t rpm)
    -> tl::expected<util::CanFrame, std::string> {
    auto scaled_rpm = static_cast<int32_t>(rpm * SET_RPM_SCALE);
    auto bytes = suc::util::to_bytes_be(scaled_rpm);
    return this->make_frame(VescSimpleCommandID::CAN_PACKET_SET_RPM, bytes);
}

auto suchm::can::VescModel::make_servo_frame(double value)
    -> tl::expected<util::CanFrame, std::string> {
    if (value < 0.0 || value > 1.0) {
        return tl::make_unexpected(
            "Servo value must be between 0.0 and 1.0 (value: " + std::to_string(value) + ")");
    }
    auto scaled_value = static_cast<int32_t>(value * SET_SERVO_SCALE);
    auto bytes = suc::util::to_bytes_be(scaled_value);
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

auto suchm::can::VescModel::id_matches(const util::CanFrame & frame) -> bool {
    const auto vesc_id = static_cast<uint8_t>(frame.id & 0xFF);
    return vesc_id == this->id;
}

auto suchm::can::VescModel::get_cmd_id(const util::CanFrame & frame)
    -> tl::expected<VescStatusCommandID, std::string> {
    if (frame.len != 8) {
        return tl::make_unexpected(
            "Received CAN frame with invalid length (expected: 8, received: " +
            std::to_string(frame.len) + ")");
    }

    // TODO: refactor
    const auto cmd_id = static_cast<uint8_t>((frame.id >> 8) & 0xFF);
    switch (cmd_id) {
        case static_cast<uint8_t>(VescStatusCommandID::CAN_PACKET_STATUS):
            return VescStatusCommandID::CAN_PACKET_STATUS;
        case static_cast<uint8_t>(VescStatusCommandID::CAN_PACKET_STATUS_2):
            return VescStatusCommandID::CAN_PACKET_STATUS_2;
        case static_cast<uint8_t>(VescStatusCommandID::CAN_PACKET_STATUS_3):
            return VescStatusCommandID::CAN_PACKET_STATUS_3;
        case static_cast<uint8_t>(VescStatusCommandID::CAN_PACKET_STATUS_4):
            return VescStatusCommandID::CAN_PACKET_STATUS_4;
        case static_cast<uint8_t>(VescStatusCommandID::CAN_PACKET_STATUS_5):
            return VescStatusCommandID::CAN_PACKET_STATUS_5;
        case static_cast<uint8_t>(VescStatusCommandID::CAN_PACKET_STATUS_6):
            return VescStatusCommandID::CAN_PACKET_STATUS_6;
        default:
            return tl::make_unexpected(
                "Received CAN frame with unknown command ID: " + std::to_string(cmd_id));
    }
}

auto suchm::can::VescModel::get_rpm(const util::CanFrame & frame)
    -> tl::expected<std::optional<sinsei_umiusi_control::state::thruster::Rpm>, std::string> {
    if (!this->id_matches(frame)) {
        return std::nullopt;
    }

    auto cmd_id = this->get_cmd_id(frame);
    if (!cmd_id) {
        return tl::make_unexpected("Failed to get command ID: " + cmd_id.error());
    }

    if (cmd_id.value() != VescStatusCommandID::CAN_PACKET_STATUS) {
        // このフレームはERPMの情報を含んでいない
        return std::nullopt;
    }

    auto scaled_erpm = suc::util::to_int32_be(frame.data);
    auto erpm = static_cast<double>(scaled_erpm) / ERPM_SCALE;
    static constexpr double BLDC_POLE_PAIR = BLDC_POLES / 2.0;
    // ERPMを極対数で割ってRPMに変換
    return suc::state::thruster::Rpm{erpm / BLDC_POLE_PAIR};
}