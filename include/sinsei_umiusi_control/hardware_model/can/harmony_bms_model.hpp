#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_HARMONY_BMS_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_HARMONY_BMS_MODEL_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

#include "sinsei_umiusi_control/hardware_model/interface/can.hpp"

// VESC BMS CANプロトコルに基づいたHarmony 16 BMSに対応
// ref: https://github.com/vedderb/bldc/blob/4fd8279ea45a17c0d69357438ae2f7237a32514f/datatypes.h
// ref: https://github.com/vedderb/bldc/blob/4fd8279ea45a17c0d69357438ae2f7237a32514f/bms.c

namespace sinsei_umiusi_control::hardware_model::can {

class HarmonyBmsModel {
  public:
    using Id = uint8_t;

    static constexpr std::size_t CELL_COUNT = 12;
    // 0〜2: セル、3: MOSFET、4: 周囲温度、5〜9: Harmony16基板上の追加温度センサー
    static constexpr std::size_t TEMPERATURE_COUNT = 10;
    static constexpr std::size_t STATUS_LENGTH = 40;

    enum class PacketId : uint8_t {
        Voltage = 38,
        Current = 39,
        Counters = 40,
        CellVoltage = 41,
        Balancing = 42,
        Temperatures = 43,
        Humidity = 44,
        Summary = 45,
        ChargeTotals = 53,
        DischargeTotals = 54,
        Status1 = 64,
        Status2 = 65,
        Status3 = 66,
        Status4 = 67,
        Status5 = 68,
    };

    enum class PowerSwitchState : uint8_t {
        Unknown = 0,
        Initializing = 1,
        Off = 2,
        Precharge = 3,
        On = 4,
        Fault = 5,
    };

    enum FaultFlag : uint32_t {
        FaultNone = 0,
        FaultPrecharge = 1U << 0,
        FaultShortCircuit = 1U << 1,
        FaultSwitchOverTemperature = 1U << 2,
        FaultChargeOvercurrent = 1U << 3,
    };

    struct State {
        State();

        double pack_voltage;
        double charger_voltage;
        double input_current;
        double measured_current;
        double net_consumed_charge;
        double net_consumed_energy;
        std::array<double, CELL_COUNT> cell_voltages;
        std::array<bool, CELL_COUNT> cell_balancing{};
        uint8_t cell_count = 0;
        std::array<double, TEMPERATURE_COUNT> temperatures;
        uint8_t temperature_count = 0;
        double humidity_sensor_temperature;
        double relative_humidity;
        double balance_ic_temperature;
        double state_of_charge;
        double state_of_health;
        double cell_voltage_min;
        double cell_voltage_max;
        double cell_temperature_max;
        bool charging = false;
        bool balancing = false;
        bool charge_allowed = false;
        uint8_t data_version = 0;
        double total_charged_charge;
        double total_charged_energy;
        double total_discharged_charge;
        double total_discharged_energy;
        std::array<char, STATUS_LENGTH> status{};
        bool status_updated = false;
        PowerSwitchState power_switch_state = PowerSwitchState::Unknown;
        uint32_t fault_flags = FaultNone;
    };

  private:
    Id id;
    State state;
    std::size_t contiguous_cells = 0;
    std::size_t contiguous_temperatures = 0;
    uint8_t status_received_mask = 0;

    auto id_matches(const interface::CanFrame & frame) const -> bool;
    auto update_status_flags() -> void;

  public:
    explicit HarmonyBmsModel(Id id);
    auto get_id() const -> Id;

    auto decode(const interface::CanFrame & frame)
        -> tl::expected<std::optional<State>, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model::can

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_HARMONY_BMS_MODEL_HPP
