#include "sinsei_umiusi_control/hardware_model/can_model.hpp"

#include "sinsei_umiusi_control/util/can_interface.hpp"

namespace suc = sinsei_umiusi_control;
namespace suchm = suc::hardware_model;

suchm::CanModel::CanModel(std::unique_ptr<suc::util::CanInterface> can_interface)
: can_interface(std::move(can_interface)) {}

auto suchm::CanModel::encode_int32_be(int32_t value) -> std::array<uint8_t, 4> {
    return {
        static_cast<uint8_t>(value >> 24), static_cast<uint8_t>(value >> 16),
        static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
}

auto suchm::CanModel::send_command_packet(
    VescSimpleCommandID command_id,
    const std::array<uint8_t, 4> & data) -> tl::expected<void, std::string> {
    auto can_id = (static_cast<uint32_t>(command_id) & 0xFF) << 8 | (VESC_ID & 0xFF);
    return can_interface->send_extframe(can_id, data.data(), data.size());
}

auto suchm::CanModel::set_vesc_duty(double duty) -> tl::expected<void, std::string> {
    if (duty < -1.0 || duty > 1.0) {
        return tl::make_unexpected("Duty must be between -1.0 and 1.0");
    }
    auto scaled_duty = static_cast<int32_t>(duty * VESC_DUTY_SCALE);
    auto data = this->encode_int32_be(scaled_duty);
    return this->send_command_packet(VescSimpleCommandID::CAN_PACKET_SET_DUTY, data);
}

auto suchm::CanModel::set_vesc_current(double current) -> tl::expected<void, std::string> {
    auto scaled_current = static_cast<int32_t>(current * VESC_CURRENT_SCALE);
    auto data = encode_int32_be(scaled_current);
    return this->send_command_packet(VescSimpleCommandID::CAN_PACKET_SET_CURRENT, data);
}

auto suchm::CanModel::set_vesc_current_brake(double current) -> tl::expected<void, std::string> {
    auto scaled_current = static_cast<int32_t>(current * VESC_SET_CURRENT_BRAKE_SCALE);
    auto data = encode_int32_be(scaled_current);
    return this->send_command_packet(VescSimpleCommandID::CAN_PACKET_SET_CURRENT_BRAKE, data);
}

auto suchm::CanModel::set_vesc_rpm(double rpm) -> tl::expected<void, std::string> {
    auto scaled_rpm = static_cast<int32_t>(rpm * VESC_SET_RPM_SCALE);
    auto data = encode_int32_be(scaled_rpm);
    return this->send_command_packet(VescSimpleCommandID::CAN_PACKET_SET_RPM, data);
}

auto suchm::CanModel::set_vesc_pos(double pos) -> tl::expected<void, std::string> {
    auto scaled_pos = static_cast<int32_t>(pos * VESC_SET_POS_SCALE);
    auto data = encode_int32_be(scaled_pos);
    return this->send_command_packet(VescSimpleCommandID::CAN_PACKET_SET_POS, data);
}

auto suchm::CanModel::set_vesc_current_rel(double current) -> tl::expected<void, std::string> {
    auto scaled_current = static_cast<int32_t>(current * VESC_SET_CURRENT_REL_SCALE);
    auto data = encode_int32_be(scaled_current);
    return this->send_command_packet(VescSimpleCommandID::CAN_PACKET_SET_CURRENT_REL, data);
}

auto suchm::CanModel::set_vesc_current_brake_rel(double current)
    -> tl::expected<void, std::string> {
    auto scaled_current = static_cast<int32_t>(current * VESC_SET_CURRENT_BRAKE_REL_SCALE);
    auto data = encode_int32_be(scaled_current);
    return this->send_command_packet(VescSimpleCommandID::CAN_PACKET_SET_CURRENT_BRAKE_REL, data);
}

auto suchm::CanModel::set_vesc_current_hand_brake(double current)
    -> tl::expected<void, std::string> {
    auto scaled_current = static_cast<int32_t>(current * VESC_SET_CURRENT_HANDBRAKE_SCALE);
    auto data = encode_int32_be(scaled_current);
    return this->send_command_packet(VescSimpleCommandID::CAN_PACKET_SET_CURRENT_HANDBRAKE, data);
}

auto suchm::CanModel::set_vesc_current_hand_brake_rel(double current)
    -> tl::expected<void, std::string> {
    auto scaled_current = static_cast<int32_t>(current * VESC_SET_CURRENT_HANDBRAKE_REL_SCALE);
    auto data = encode_int32_be(scaled_current);
    return this->send_command_packet(
        VescSimpleCommandID::CAN_PACKET_SET_CURRENT_HANDBRAKE_REL, data);
}

auto suchm::CanModel::set_vesc_servo(double value) -> tl::expected<void, std::string> {
    if (value < 0.0 || value > 1.0) {
        return tl::make_unexpected("Servo value must be between 0.0 and 1.0");
    }
    auto scaled_value = static_cast<int32_t>(value * VESC_SET_SERVO_SCALE);
    auto data = encode_int32_be(scaled_value);
    return this->send_command_packet(VescSimpleCommandID::CAN_PACKET_SET_SERVO, data);
}

auto suchm::CanModel::set_vesc_servo_angle(double angle) -> tl::expected<void, std::string> {
    // TODO: 実装する
}

auto suchm::CanModel::on_read()
    -> tl::expected<
        std::tuple<
            std::array<suc::state::thruster::ServoCurrent, 4>,
            std::array<suc::state::thruster::Rpm, 4>, suc::state::main_power::BatteryCurrent,
            suc::state::main_power::BatteryVoltage, suc::state::main_power::Temperature,
            suc::state::main_power::WaterLeaked>,
        std::string> {
    // Implement the reading logic here
    return tl::make_unexpected("Not implemented");
}

auto suchm::CanModel::on_write(
    std::array<suc::cmd::thruster::EscEnabled, 4> thruster_esc_enabled,
    std::array<suc::cmd::thruster::ServoEnabled, 4> thruster_servo_enabled,
    std::array<suc::cmd::thruster::Angle, 4> thruster_angle,
    std::array<suc::cmd::thruster::Thrust, 4> thruster_thrust,
    suc::cmd::main_power::Enabled main_power_enabled,
    suc::cmd::led_tape::Color led_tape_color) -> tl::expected<void, std::string> {
    // Implement the writing logic here
    return tl::make_unexpected("Not implemented");
}