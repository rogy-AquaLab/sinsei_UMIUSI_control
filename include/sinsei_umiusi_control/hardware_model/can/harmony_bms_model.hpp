#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_HARMONY_BMS_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_HARMONY_BMS_MODEL_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

#include "sinsei_umiusi_control/hardware_model/interface/can.hpp"

// VESC BMS CAN protocol used by the Harmony16-compatible circuit.
// ref: https://github.com/vedderb/bldc/blob/master/datatypes.h
// ref: https://github.com/vedderb/bldc/blob/master/bms.c

namespace sinsei_umiusi_control::hardware_model::can {

class HarmonyBmsModel {
  public:
    using Id = uint8_t;

    static constexpr std::size_t CELL_COUNT = 12;
    // Harmony firmware exposes five built-in temperatures and up to four external ones.
    static constexpr std::size_t TEMPERATURE_COUNT = 9;
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
        double pack_voltage = 0.0;
        double charger_voltage = 0.0;
        double input_current = 0.0;
        double measured_current = 0.0;
        double net_consumed_charge = 0.0;
        double net_consumed_energy = 0.0;
        std::array<double, CELL_COUNT> cell_voltages{};
        std::array<bool, CELL_COUNT> cell_balancing{};
        uint8_t cell_count = 0;
        std::array<double, TEMPERATURE_COUNT> temperatures{};
        uint8_t temperature_count = 0;
        double humidity_sensor_temperature = 0.0;
        double relative_humidity = 0.0;
        double balance_ic_temperature = 0.0;
        double state_of_charge = 0.0;
        double state_of_health = 0.0;
        double cell_voltage_min = 0.0;
        double cell_voltage_max = 0.0;
        double cell_temperature_max = 0.0;
        bool charging = false;
        bool balancing = false;
        bool charge_allowed = false;
        uint8_t data_version = 0;
        double total_charged_charge = 0.0;
        double total_charged_energy = 0.0;
        double total_discharged_charge = 0.0;
        double total_discharged_energy = 0.0;
        std::array<char, STATUS_LENGTH> status{};
        PowerSwitchState power_switch_state = PowerSwitchState::Unknown;
        uint32_t fault_flags = FaultNone;
    };

  private:
    Id id;
    State state;
    std::size_t contiguous_cells = 0;
    std::size_t contiguous_temperatures = 0;

    auto update_status_flags() -> void;

  public:
    explicit HarmonyBmsModel(Id id);
    auto get_id() const -> Id;

    auto decode(const interface::CanFrame & frame)
        -> tl::expected<std::optional<State>, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model::can

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_HARMONY_BMS_MODEL_HPP
