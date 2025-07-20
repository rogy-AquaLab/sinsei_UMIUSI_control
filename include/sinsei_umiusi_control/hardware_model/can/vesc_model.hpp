#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_CAN_VESC_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_CAN_VESC_MODEL_HPP

#include <array>
#include <cstdint>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

#include "sinsei_umiusi_control/state/thruster.hpp"
#include "sinsei_umiusi_control/util/can_interface.hpp"

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

enum class VescStatusCommandID : uint32_t {
    CAN_PACKET_STATUS = 9,
    CAN_PACKET_STATUS_2 = 14,
    CAN_PACKET_STATUS_3 = 15,
    CAN_PACKET_STATUS_4 = 16,
    CAN_PACKET_STATUS_5 = 27,
    CAN_PACKET_STATUS_6 = 58,
};

enum class VescAdcChannel : uint8_t { ADC1, ADC2, ADC3 };

class VescModel {
  private:
    uint8_t id;

    static constexpr double BLDC_POLES = 14.0;

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

    // CAN_PACKET_STATUS
    static constexpr double ERPM_SCALE = 1;
    static constexpr double CURRENT_SCALE = 10;
    static constexpr double DUTY_SCALE = 1000;

    // CAN_PACKET_STATUS2
    static constexpr double AMP_HOUR_SCALE = 10000;
    static constexpr double AMP_HOUR_CHARGE_SCALE = 10000;

    // CAN_PACKET_STATUS3
    static constexpr double WATT_HOUR_SCALE = 10000;
    static constexpr double WATT_HOUR_CHARGE_SCALE = 10000;

    // CAN_PACKET_STATUS4
    static constexpr double TEMP_FET_SCALE = 10;
    static constexpr double TEMP_MOTOR_SCALE = 10;
    static constexpr double CURRENT_IN_SCALE = 10;
    static constexpr double PID_POS_SCALE = 50;

    // CAN_PACKET_STATUS5
    static constexpr double TACOMETER_SCALE = 6;
    static constexpr double VOLTS_IN_SCALE = 10;

    // CAN_PACKET_STATUS6
    static constexpr double ADC1_SCALE = 1000;
    static constexpr double ADC2_SCALE = 1000;
    static constexpr double ADC3_SCALE = 1000;
    static constexpr double PPM_SCALE = 1000;

    auto make_frame(VescSimpleCommandID command_id, const std::array<uint8_t, 8> & data)
        -> util::CanFrame;

    auto make_servo_frame(double value)
        -> tl::expected<util::CanFrame, std::string>;  // lispBMにより実装。0 ~ 1.0

  public:
    VescModel(uint8_t id);

    auto make_duty_frame(double duty) -> tl::expected<util::CanFrame, std::string>;
    auto make_rpm_frame(int8_t rpm) -> tl::expected<util::CanFrame, std::string>;
    auto make_servo_angle_frame(double deg) -> tl::expected<util::CanFrame, std::string>;

    auto handle_frame(const util::CanFrame & frame)
        -> tl::expected<sinsei_umiusi_control::state::thruster::Rpm, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model::can

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_CAN_VESC_MODEL_HPP