#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP

#include <memory>

#include "sinsei_umiusi_control/cmd/led_tape.hpp"
#include "sinsei_umiusi_control/cmd/main_power.hpp"
#include "sinsei_umiusi_control/cmd/thruster.hpp"
#include "sinsei_umiusi_control/state/main_power.hpp"
#include "sinsei_umiusi_control/state/thruster.hpp"
#include "sinsei_umiusi_control/util/can_interface.hpp"
#include "sinsei_umiusi_control/util/gpio.hpp"

namespace suc = sinsei_umiusi_control;

// -- VESC --
// ref: https://github.com/vedderb/bldc/blob/822d270/documentation/comm_can.md
// ref: https://github.com/rogy-AquaLab/sinsei-UMIUSI/blob/fa11563/VESC_ESP32_CAN/lib/vesc_can_esp32/vesc_can_esp32.hpp

namespace sinsei_umiusi_control::hardware_model {

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

enum class AdcChannel : uint8_t { ADC1, ADC2, ADC3 };

class CanModel {
  private:
    std::unique_ptr<util::CanInterface> can_interface;

    static constexpr int VESC_STATUS_COMMAND_NUM = 6;

    const uint8_t VESC_ID = {0x00};
    std::array<bool, VESC_STATUS_COMMAND_NUM> transmit_status_command;

    static constexpr double VESC_SET_DUTY_SCALE = 100000;
    static constexpr double VESC_SET_CURRENT_SCALE = 1000;
    static constexpr double VESC_SET_CURRENT_BRAKE_SCALE = 1000;
    static constexpr double VESC_SET_RPM_SCALE = 1;
    static constexpr double VESC_SET_POS_SCALE = 1000000;
    static constexpr double VESC_SET_CURRENT_REL_SCALE = 100000;
    static constexpr double VESC_SET_CURRENT_BRAKE_REL_SCALE = 100000;
    static constexpr double VESC_SET_CURRENT_HANDBRAKE_SCALE = 1000;
    static constexpr double VESC_SET_CURRENT_HANDBRAKE_REL_SCALE = 100000;

    static constexpr double VESC_SET_SERVO_SCALE = 10000;  // lispBMにより実装

    // CAN_PACKET_STATUS
    static constexpr double VESC_ERPM_SCALE = 1;
    static constexpr double VESC_CURRENT_SCALE = 10;
    static constexpr double VESC_DUTY_SCALE = 1000;

    // CAN_PACKET_STATUS2
    static constexpr double VESC_AMP_HOUR_SCALE = 10000;
    static constexpr double VESC_AMP_HOUR_CHARGE_SCALE = 10000;

    // CAN_PACKET_STATUS3
    static constexpr double VESC_WATT_HOUR_SCALE = 10000;
    static constexpr double VESC_WATT_HOUR_CHARGE_SCALE = 10000;

    // CAN_PACKET_STATUS4
    static constexpr double VESC_TEMP_FET_SCALE = 10;
    static constexpr double VESC_TEMP_MOTOR_SCALE = 10;
    static constexpr double VESC_CURRENT_IN_SCALE = 10;
    static constexpr double VESC_PID_POS_SCALE = 50;

    // CAN_PACKET_STATUS5
    static constexpr double VESC_TACOMETER_SCALE = 6;
    static constexpr double VESC_VOLTS_IN_SCALE = 10;

    // CAN_PACKET_STATUS6
    static constexpr double VESC_ADC1_SCALE = 1000;
    static constexpr double VESC_ADC2_SCALE = 1000;
    static constexpr double VESC_ADC3_SCALE = 1000;
    static constexpr double VESC_PPM_SCALE = 1000;

    auto encode_int32_be(int32_t value) -> std::array<uint8_t, 4>;

    auto send_command_packet(VescSimpleCommandID command_id, const std::array<uint8_t, 4> & data)
        -> tl::expected<void, std::string>;

    auto set_vesc_duty(double duty) -> tl::expected<void, std::string>;
    auto set_vesc_current(double current) -> tl::expected<void, std::string>;
    auto set_vesc_current_brake(double current) -> tl::expected<void, std::string>;
    auto set_vesc_rpm(double rpm) -> tl::expected<void, std::string>;
    auto set_vesc_pos(double pos) -> tl::expected<void, std::string>;
    auto set_vesc_current_rel(double current) -> tl::expected<void, std::string>;
    auto set_vesc_current_brake_rel(double current) -> tl::expected<void, std::string>;
    auto set_vesc_current_hand_brake(double current) -> tl::expected<void, std::string>;
    auto set_vesc_current_hand_brake_rel(double current) -> tl::expected<void, std::string>;
    auto set_vesc_servo(double value)
        -> tl::expected<void, std::string>;  // lispBMにより実装。0 ~ 1.0
    auto set_vesc_servo_angle(double angle) -> tl::expected<void, std::string>;

    auto get_vesc_eprm() -> tl::expected<double, std::string>;
    auto get_vesc_current() -> tl::expected<double, std::string>;
    auto get_vesc_duty() -> tl::expected<double, std::string>;

    auto get_vesc_amp_hours() -> tl::expected<double, std::string>;
    auto get_vesc_amp_hours_charge() -> tl::expected<double, std::string>;

    auto get_vesc_watt_hours() -> tl::expected<double, std::string>;
    auto get_vesc_watt_hours_charge() -> tl::expected<double, std::string>;

    auto get_vesc_temp_fet() -> tl::expected<double, std::string>;
    auto get_vesc_temp_motor() -> tl::expected<double, std::string>;
    auto get_vesc_current_in() -> tl::expected<double, std::string>;
    auto get_vesc_pid_pos() -> tl::expected<double, std::string>;

    auto get_vesc_tachometer() -> tl::expected<double, std::string>;
    auto get_vesc_volt_in() -> tl::expected<double, std::string>;

    auto get_vesc_adc_data(AdcChannel adc) -> tl::expected<double, std::string>;
    auto get_vesc_ppm() -> tl::expected<double, std::string>;

  public:
    CanModel(std::unique_ptr<util::CanInterface> can_interface);
    auto on_read()
        -> tl::expected<
            std::tuple<
                std::array<suc::state::thruster::ServoCurrent, 4>,
                std::array<suc::state::thruster::Rpm, 4>, suc::state::main_power::BatteryCurrent,
                suc::state::main_power::BatteryVoltage, suc::state::main_power::Temperature,
                suc::state::main_power::WaterLeaked>,
            std::string>;
    auto on_write(
        std::array<suc::cmd::thruster::EscEnabled, 4> thruster_esc_enabled,
        std::array<suc::cmd::thruster::ServoEnabled, 4> thruster_servo_enabled,
        std::array<suc::cmd::thruster::Angle, 4> thruster_angle,
        std::array<suc::cmd::thruster::Thrust, 4> thruster_thrust,
        suc::cmd::main_power::Enabled main_power_enabled,
        suc::cmd::led_tape::Color led_tape_color) -> tl::expected<void, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_CAN_MODEL_HPP