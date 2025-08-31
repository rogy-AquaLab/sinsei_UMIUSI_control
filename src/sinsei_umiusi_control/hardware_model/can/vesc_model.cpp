#include "sinsei_umiusi_control/hardware_model/can/vesc_model.hpp"

#include <rcpputils/tl_expected/expected.hpp>

#include "sinsei_umiusi_control/hardware_model/interface/can.hpp"
#include "sinsei_umiusi_control/util/byte.hpp"

using namespace sinsei_umiusi_control::hardware_model;

can::VescModel::VescModel(can::VescModel::Id id) : id(id) {}

auto can::VescModel::make_frame(
    VescSimpleCommandID && command_id,
    interface::CanFrame::Data && data) const -> interface::CanFrame {
    const auto id =
        (static_cast<interface::CanFrame::Id>(command_id) & 0xFF) << 8 | (this->id & 0xFF);
    return interface::CanFrame{
        std::move(id),    // id
        4,                // len
        std::move(data),  // data
        true,             // is_extended
    };
}

auto can::VescModel::make_duty_frame(double && duty) const
    -> tl::expected<interface::CanFrame, std::string> {
    if (duty < -1.0 || duty > 1.0) {
        return tl::make_unexpected(
            "Duty must be between -1.0 and 1.0 (duty: " + std::to_string(duty) + ")");
    }
    auto && scaled_duty = static_cast<int32_t>(std::move(duty) * SET_DUTY_SCALE);
    auto && bytes = util::to_bytes_be(std::move(scaled_duty));
    return this->make_frame(VescSimpleCommandID::CAN_PACKET_SET_DUTY, std::move(bytes));
}

auto can::VescModel::make_rpm_frame(int8_t && rpm) const
    -> tl::expected<interface::CanFrame, std::string> {
    auto && scaled_rpm = static_cast<int32_t>(std::move(rpm) * SET_RPM_SCALE);
    auto && bytes = util::to_bytes_be(std::move(scaled_rpm));
    return this->make_frame(VescSimpleCommandID::CAN_PACKET_SET_RPM, std::move(bytes));
}

auto can::VescModel::make_servo_frame(double && value) const
    -> tl::expected<interface::CanFrame, std::string> {
    if (value < 0.0 || value > 1.0) {
        return tl::make_unexpected(
            "Servo value must be between 0.0 and 1.0 (value: " + std::to_string(value) + ")");
    }
    auto && scaled_value = static_cast<int32_t>(std::move(value) * SET_SERVO_SCALE);
    auto && bytes = util::to_bytes_be(std::move(scaled_value));
    return this->make_frame(VescSimpleCommandID::CAN_PACKET_SET_SERVO, std::move(bytes));
}

auto can::VescModel::make_servo_angle_frame(double && deg) const
    -> tl::expected<interface::CanFrame, std::string> {
    // -90.0 ~ 90.0度の角度を0.0 ~ 1.0に変換
    if (deg < -90.0 || deg > 90.0) {
        return tl::make_unexpected(
            "Servo angle must be between -90.0 ~ 90.0 degrees (deg: " + std::to_string(deg) + ")");
    }
    return this->make_servo_frame((std::move(deg) + 90.0) / 180.0);
}

auto can::VescModel::id_matches(const interface::CanFrame & frame) const -> bool {
    const auto vesc_id = static_cast<can::VescModel::Id>(frame.id & 0xFF);
    return vesc_id == this->id;
}

auto can::VescModel::get_packet_status(const interface::CanFrame & frame) const
    -> tl::expected<std::optional<VescModel::AnyPacketStatus>, std::string> {
    if (!this->id_matches(frame)) {
        return std::nullopt;
    }

    if (frame.len != 8) {
        return tl::make_unexpected(
            "Received CAN frame with invalid length (expected: 8, received: " +
            std::to_string(frame.len) + ")");
    }

    const auto cmd_id = (frame.id >> 8) & 0xFF;

    switch (cmd_id) {
        case PacketStatus::ID: {
            auto && scaled_erpm = util::to_int32_be(frame.data);
            auto && scaled_current = util::to_int16_be(frame.data, 4);
            auto && scaled_duty = util::to_int16_be(frame.data, 6);
            if (!scaled_erpm || !scaled_current || !scaled_duty) {
                return tl::make_unexpected("Failed to parse CAN_PACKET_STATUS");
            }
            return PacketStatus{
                static_cast<double>(scaled_erpm.value()) / PacketStatus::ERPM_SCALE,
                static_cast<double>(scaled_current.value()) / PacketStatus::CURRENT_SCALE,
                static_cast<double>(scaled_duty.value()) / PacketStatus::DUTY_SCALE,
            };
        }
        case PacketStatus2::ID: {
            auto && scaled_amp_hour = util::to_int32_be(frame.data);
            auto && scaled_amp_hour_charge = util::to_int32_be(frame.data, 4);
            if (!scaled_amp_hour || !scaled_amp_hour_charge) {
                return tl::make_unexpected("Failed to parse CAN_PACKET_STATUS_2");
            }
            return PacketStatus2{
                static_cast<double>(scaled_amp_hour.value()) / PacketStatus2::AMP_HOUR_SCALE,
                static_cast<double>(scaled_amp_hour_charge.value()) /
                    PacketStatus2::AMP_HOUR_CHARGE_SCALE,
            };
        }
        case PacketStatus3::ID: {
            auto && scaled_watt_hour = util::to_int32_be(frame.data);
            auto && scaled_watt_hour_charge = util::to_int32_be(frame.data, 4);
            if (!scaled_watt_hour || !scaled_watt_hour_charge) {
                return tl::make_unexpected("Failed to parse CAN_PACKET_STATUS_3");
            }
            return PacketStatus3{
                static_cast<double>(scaled_watt_hour.value()) / PacketStatus3::WATT_HOUR_SCALE,
                static_cast<double>(scaled_watt_hour_charge.value()) /
                    PacketStatus3::WATT_HOUR_CHARGE_SCALE,
            };
        }
        case PacketStatus4::ID: {
            auto && scaled_temp_fet = util::to_int16_be(frame.data);
            auto && scaled_temp_motor = util::to_int16_be(frame.data, 2);
            auto && scaled_current_in = util::to_int16_be(frame.data, 4);
            auto && scaled_pid_pos = util::to_int16_be(frame.data, 6);
            if (!scaled_temp_fet || !scaled_temp_motor || !scaled_current_in || !scaled_pid_pos) {
                return tl::make_unexpected("Failed to parse CAN_PACKET_STATUS_4");
            }
            return PacketStatus4{
                static_cast<double>(scaled_temp_fet.value()) / PacketStatus4::TEMP_FET_SCALE,
                static_cast<double>(scaled_temp_motor.value()) / PacketStatus4::TEMP_MOTOR_SCALE,
                static_cast<double>(scaled_current_in.value()) / PacketStatus4::CURRENT_IN_SCALE,
                static_cast<double>(scaled_pid_pos.value()) / PacketStatus4::PID_POS_SCALE,
            };
        }
        case PacketStatus5::ID: {
            auto && scaled_tacometer = util::to_int32_be(frame.data);
            auto && scaled_volts_in = util::to_int16_be(frame.data, 4);
            if (!scaled_tacometer || !scaled_volts_in) {
                return tl::make_unexpected("Failed to parse CAN_PACKET_STATUS_5");
            }
            return PacketStatus5{
                static_cast<double>(scaled_tacometer.value()) / PacketStatus5::TACOMETER_SCALE,
                static_cast<double>(scaled_volts_in.value()) / PacketStatus5::VOLTS_IN_SCALE,
            };
        }
        case PacketStatus6::ID: {
            auto && scaled_adc1 = util::to_int16_be(frame.data);
            auto && scaled_adc2 = util::to_int16_be(frame.data, 2);
            auto && scaled_adc3 = util::to_int16_be(frame.data, 4);
            auto && scaled_ppm = util::to_int16_be(frame.data, 6);
            if (!scaled_adc1 || !scaled_adc2 || !scaled_adc3 || !scaled_ppm) {
                return tl::make_unexpected("Failed to parse CAN_PACKET_STATUS_6");
            }
            return PacketStatus6{
                static_cast<double>(scaled_adc1.value()) / PacketStatus6::ADC1_SCALE,
                static_cast<double>(scaled_adc2.value()) / PacketStatus6::ADC2_SCALE,
                static_cast<double>(scaled_adc3.value()) / PacketStatus6::ADC3_SCALE,
                static_cast<double>(scaled_ppm.value()) / PacketStatus6::PPM_SCALE,
            };
        }
        default:
            return tl::make_unexpected(
                "Received CAN frame with unknown command ID: " + std::to_string(cmd_id));
    }
}
