#include "sinsei_umiusi_control/hardware_model/can/vesc_model.hpp"

#include <algorithm>

namespace suc = sinsei_umiusi_control;
namespace suchm = suc::hardware_model;

suchm::can::VescModel::VescModel(std::shared_ptr<suc::util::CanInterface> can, uint8_t id)
: can(std::move(can)), id(id) {}

auto suchm::can::VescModel::to_bytes_be(int32_t value) -> std::array<uint8_t, 4> {
    return {
        static_cast<uint8_t>(value >> 24), static_cast<uint8_t>(value >> 16),
        static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
}

auto suchm::can::VescModel::to_int32_be(std::array<uint8_t, 4> bytes) -> int32_t {
    return (static_cast<int32_t>(bytes[0]) << 24) | (static_cast<int32_t>(bytes[1]) << 16) |
           (static_cast<int32_t>(bytes[2]) << 8) | static_cast<int32_t>(bytes[3]);
}

auto suchm::can::VescModel::send_command_packet(
    VescSimpleCommandID command_id,
    const std::array<uint8_t, 4> & data) -> tl::expected<void, std::string> {
    auto can_id = (static_cast<uint32_t>(command_id) & 0xFF) << 8 | (this->id & 0xFF);
    return this->can->send_frame_ext(can_id, data.data(), data.size());
}

auto suchm::can::VescModel::recv_status_frame(VescStatusCommandID expected_cmd_id)
    -> tl::expected<std::array<uint8_t, 8>, std::string> {
    auto frame_result = this->can->recv_frame();
    if (!frame_result) {
        return tl::make_unexpected("Failed to recv CAN frame: " + frame_result.error());
    }

    const auto & frame = *frame_result;

    const auto cmd_id = static_cast<uint8_t>((frame.id >> 8) & 0xFF);
    const auto vesc_id = static_cast<uint8_t>(frame.id & 0xFF);

    if (vesc_id != this->id) {
        return tl::make_unexpected("Unexpected VESC ID: " + std::to_string(vesc_id));
    }

    if (cmd_id != static_cast<uint8_t>(expected_cmd_id)) {
        return tl::make_unexpected("Unexpected Command ID: " + std::to_string(cmd_id));
    }

    if (frame.dlc != 8) {
        return tl::make_unexpected("Unexpected DLC (must be 8)");
    }

    return frame.data;
}

auto suchm::can::VescModel::set_duty(double duty) -> tl::expected<void, std::string> {
    if (duty < -1.0 || duty > 1.0) {
        return tl::make_unexpected(
            "Duty must be between -1.0 and 1.0 (duty: " + std::to_string(duty) + ")");
    }
    auto scaled_duty = static_cast<int32_t>(duty * DUTY_SCALE);
    auto bytes = this->to_bytes_be(scaled_duty);
    return this->send_command_packet(VescSimpleCommandID::CAN_PACKET_SET_DUTY, bytes);
}

auto suchm::can::VescModel::set_rpm(int8_t rpm) -> tl::expected<void, std::string> {
    auto scaled_rpm = static_cast<int32_t>(rpm * SET_RPM_SCALE);
    auto bytes = this->to_bytes_be(scaled_rpm);
    return this->send_command_packet(VescSimpleCommandID::CAN_PACKET_SET_RPM, bytes);
}

auto suchm::can::VescModel::set_servo(double value) -> tl::expected<void, std::string> {
    if (value < 0.0 || value > 1.0) {
        return tl::make_unexpected(
            "Servo value must be between 0.0 and 1.0 (value: " + std::to_string(value) + ")");
    }
    auto scaled_value = static_cast<int32_t>(value * SET_SERVO_SCALE);
    auto bytes = to_bytes_be(scaled_value);
    return this->send_command_packet(VescSimpleCommandID::CAN_PACKET_SET_SERVO, bytes);
}

auto suchm::can::VescModel::set_servo_angle(double deg) -> tl::expected<void, std::string> {
    // 0.0 ~ 180.0度の角度を0.0 ~ 1.0に変換
    if (deg < 0.0 || deg > 180.0) {
        return tl::make_unexpected(
            "Servo angle must be between 0.0 and 180.0 degrees (deg: " + std::to_string(deg) + ")");
    }
    return this->set_servo(deg / 180.0);
}

auto suchm::can::VescModel::get_erpm() -> tl::expected<double, std::string> {
    auto res = recv_status_frame(VescStatusCommandID::CAN_PACKET_STATUS);
    if (!res) return tl::make_unexpected(res.error());

    std::array<uint8_t, 4> bytes;
    std::copy_n(res.value().begin(), 4, bytes.begin());

    auto scaled_erpm = to_int32_be(bytes);
    return static_cast<double>(scaled_erpm) / ERPM_SCALE;
}

auto suchm::can::VescModel::get_rpm() -> tl::expected<double, std::string> {
    auto erpm_res = this->get_erpm();
    if (!erpm_res) return tl::make_unexpected(erpm_res.error());

    static constexpr double BLDC_POLE_PAIR = BLDC_POLES / 2.0;
    // ERPMを極対数で割ってRPMに変換
    return erpm_res.value() / BLDC_POLE_PAIR;
}