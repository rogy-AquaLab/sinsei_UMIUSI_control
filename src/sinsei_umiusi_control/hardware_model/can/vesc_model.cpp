#include "sinsei_umiusi_control/hardware_model/can/vesc_model.hpp"

namespace suc = sinsei_umiusi_control;
namespace suchm = suc::hardware_model;

suchm::can::VescModel::VescModel(std::shared_ptr<suc::util::CanInterface> can, uint8_t id)
: can(std::move(can)), id(id) {}

auto suchm::can::VescModel::encode_int32_be(int32_t value) -> std::array<uint8_t, 4> {
    return {
        static_cast<uint8_t>(value >> 24), static_cast<uint8_t>(value >> 16),
        static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
}

auto suchm::can::VescModel::send_command_packet(
    VescSimpleCommandID command_id,
    const std::array<uint8_t, 4> & data) -> tl::expected<void, std::string> {
    auto can_id = (static_cast<uint32_t>(command_id) & 0xFF) << 8 | (this->id & 0xFF);
    return can->send_extframe(can_id, data.data(), data.size());
}

auto suchm::can::VescModel::set_duty(double duty) -> tl::expected<void, std::string> {
    if (duty < -1.0 || duty > 1.0) {
        return tl::make_unexpected("Duty must be between -1.0 and 1.0");
    }
    auto scaled_duty = static_cast<int32_t>(duty * DUTY_SCALE);
    auto data = this->encode_int32_be(scaled_duty);
    return this->send_command_packet(VescSimpleCommandID::CAN_PACKET_SET_DUTY, data);
}

auto suchm::can::VescModel::set_servo(double value) -> tl::expected<void, std::string> {
    if (value < 0.0 || value > 1.0) {
        return tl::make_unexpected("Servo value must be between 0.0 and 1.0");
    }
    auto scaled_value = static_cast<int32_t>(value * SET_SERVO_SCALE);
    auto data = encode_int32_be(scaled_value);
    return this->send_command_packet(VescSimpleCommandID::CAN_PACKET_SET_SERVO, data);
}

auto suchm::can::VescModel::set_servo_angle(double angle) -> tl::expected<void, std::string> {
    // 0.0 ~ 180.0度の角度を0.0 ~ 1.0に変換
    return this->set_servo(angle / 180.0);
}