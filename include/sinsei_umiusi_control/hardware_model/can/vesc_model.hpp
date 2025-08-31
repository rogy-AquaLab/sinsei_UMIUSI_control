#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_CAN_VESC_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_CAN_VESC_MODEL_HPP

#include <cstdint>
#include <optional>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>
#include <variant>

#include "sinsei_umiusi_control/hardware_model/interface/can.hpp"

// ref: https://github.com/vedderb/bldc/blob/822d270/documentation/comm_can.md
// ref: https://github.com/rogy-AquaLab/sinsei-UMIUSI/blob/fa11563/ESP32_CAN/lib/vesc_can_esp32/vesc_can_esp32.hpp

namespace sinsei_umiusi_control::hardware_model::can {

enum class VescSimpleCommandID : uint32_t {
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT = 1,
    CAN_PACKET_SET_CURRENT_BRAKE = 2,
    CAN_PACKET_SET_RPM = 3,
    CAN_PACKET_SET_POS = 4,
    CAN_PACKET_SET_CURRENT_REL = 10,
    CAN_PACKET_SET_CURRENT_BRAKE_REL = 11,
    CAN_PACKET_SET_CURRENT_HANDBRAKE = 12,
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL = 13,
    CAN_PACKET_SET_SERVO = 69,
};

struct PacketStatus {
    static constexpr uint32_t ID = 9;

    double erpm;
    double current;
    double duty;

    static constexpr double ERPM_SCALE = 1;
    static constexpr double CURRENT_SCALE = 10;
    static constexpr double DUTY_SCALE = 1000;
};

struct PacketStatus2 {
    static constexpr uint32_t ID = 14;

    double amp_hour;
    double amp_hour_charge;

    static constexpr double AMP_HOUR_SCALE = 10000;
    static constexpr double AMP_HOUR_CHARGE_SCALE = 10000;
};

struct PacketStatus3 {
    static constexpr uint32_t ID = 15;

    double watt_hour;
    double watt_hour_charge;

    static constexpr double WATT_HOUR_SCALE = 10000;
    static constexpr double WATT_HOUR_CHARGE_SCALE = 10000;
};

struct PacketStatus4 {
    static constexpr uint32_t ID = 16;

    double temp_fet;
    double temp_motor;
    double current_in;
    double pid_pos;

    static constexpr double TEMP_FET_SCALE = 10;
    static constexpr double TEMP_MOTOR_SCALE = 10;
    static constexpr double CURRENT_IN_SCALE = 10;
    static constexpr double PID_POS_SCALE = 50;
};

struct PacketStatus5 {
    static constexpr uint32_t ID = 27;

    double tacometer;
    double volts_in;

    static constexpr double TACOMETER_SCALE = 6;
    static constexpr double VOLTS_IN_SCALE = 10;
};

struct PacketStatus6 {
    static constexpr uint32_t ID = 58;

    double adc1;
    double adc2;
    double adc3;
    double ppm;

    static constexpr double ADC1_SCALE = 1000;
    static constexpr double ADC2_SCALE = 1000;
    static constexpr double ADC3_SCALE = 1000;
    static constexpr double PPM_SCALE = 1000;
};

enum class VescAdcChannel : uint8_t { ADC1, ADC2, ADC3 };

class VescModel {
  public:
    using Id = uint8_t;

    static constexpr int STATUS_COMMAND_NUM = 6;

    // std::array<bool, STATUS_COMMAND_NUM> transmit_status_command;

    static constexpr double SET_DUTY_SCALE = 100000;
    // static constexpr double SET_CURRENT_SCALE = 1000;
    // static constexpr double SET_CURRENT_BRAKE_SCALE = 1000;
    static constexpr double SET_RPM_SCALE = 1;
    // static constexpr double SET_POS_SCALE = 1000000;
    // static constexpr double SET_CURRENT_REL_SCALE = 100000;
    // static constexpr double SET_CURRENT_BRAKE_REL_SCALE = 100000;
    // static constexpr double SET_CURRENT_HANDBRAKE_SCALE = 1000;
    // static constexpr double SET_CURRENT_HANDBRAKE_REL_SCALE = 100000;

    // 元々のファームウェアにはないが、lispBMにより追加で実装
    static constexpr double SET_SERVO_SCALE = 10000;

  private:
    Id id;

    auto make_frame(VescSimpleCommandID && command_id, interface::CanFrame::Data && data) const
        -> interface::CanFrame;

    auto make_servo_frame(double && value) const
        -> tl::expected<interface::CanFrame, std::string>;  // lispBMにより実装。0 ~ 1.0

    auto id_matches(const interface::CanFrame & frame) const -> bool;

  public:
    VescModel(Id id);

    auto make_duty_frame(double && duty) const -> tl::expected<interface::CanFrame, std::string>;
    auto make_rpm_frame(int8_t && rpm) const -> tl::expected<interface::CanFrame, std::string>;
    auto make_servo_angle_frame(double && deg) const
        -> tl::expected<interface::CanFrame, std::string>;

    using AnyPacketStatus = std::variant<
        PacketStatus, PacketStatus2, PacketStatus3, PacketStatus4, PacketStatus5, PacketStatus6>;

    auto get_packet_status(const interface::CanFrame & frame) const
        -> tl::expected<std::optional<AnyPacketStatus>, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model::can

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_CAN_VESC_MODEL_HPP
