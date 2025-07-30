#include "sinsei_umiusi_control/hardware_model/can/vesc_model.hpp"

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
    // 0.0 ~ 180.0度の角度を0.0 ~ 1.0に変換
    if (deg < 0.0 || deg > 180.0) {
        return tl::make_unexpected(
            "Servo angle must be between 0.0 and 180.0 degrees (deg: " + std::to_string(deg) + ")");
    }
    return this->make_servo_frame(std::move(deg) / 180.0);
}

auto can::VescModel::id_matches(const interface::CanFrame & frame) const -> bool {
    const auto vesc_id = static_cast<can::VescModel::Id>(frame.id & 0xFF);
    return vesc_id == this->id;
}

auto can::VescModel::get_cmd_id(const interface::CanFrame & frame)
    -> tl::expected<VescStatusCommandID, std::string> {
    if (frame.len != 8) {
        return tl::make_unexpected(
            "Received CAN frame with invalid length (expected: 8, received: " +
            std::to_string(frame.len) + ")");
    }

    const auto cmd_id = (frame.id >> 8) & 0xFF;
    return util::enum_cast<interface::CanFrame::Id, VescStatusCommandID>(cmd_id).map_error(
        [&cmd_id](const auto &) {
            return "Received CAN frame with unknown command ID: " + std::to_string(cmd_id);
        });
}

auto can::VescModel::get_rpm(const interface::CanFrame & frame) const
    -> tl::expected<std::optional<state::thruster::Rpm>, std::string> {
    if (!this->id_matches(frame)) {
        return std::nullopt;
    }

    const auto cmd_id = this->get_cmd_id(frame);
    if (!cmd_id) {
        return tl::make_unexpected("Failed to get command ID: " + cmd_id.error());
    }

    if (cmd_id.value() != VescStatusCommandID::CAN_PACKET_STATUS) {
        // このフレームはERPMの情報を含んでいない
        return std::nullopt;
    }

    auto && scaled_erpm = util::to_int32_be(frame.data);
    auto && erpm = static_cast<double>(scaled_erpm) / ERPM_SCALE;
    static constexpr double BLDC_POLE_PAIR = BLDC_POLES / 2.0;
    // ERPMを極対数で割ってRPMに変換
    return state::thruster::Rpm{erpm / BLDC_POLE_PAIR};
}

auto can::VescModel::get_water_leaked(const interface::CanFrame & frame) const
    -> tl::expected<std::optional<state::esc::WaterLeaked>, std::string> {
    if (!this->id_matches(frame)) {
        return std::nullopt;
    }

    const auto cmd_id = this->get_cmd_id(frame);
    if (!cmd_id) {
        return tl::make_unexpected("Failed to get command ID: " + cmd_id.error());
    }

    if (cmd_id.value() != VescStatusCommandID::CAN_PACKET_STATUS_6) {
        // このフレームはADC1の情報を含んでいない
        return std::nullopt;
    }

    auto && scaled_adc1 = util::to_int16_be(frame.data);
    auto && adc1_voltage = static_cast<double>(scaled_adc1) / ADC1_SCALE;
    auto && water_leaked = adc1_voltage > WATER_LEAKED_VOLTAGE_THRESHOLD;
    return state::esc::WaterLeaked{water_leaked};
}