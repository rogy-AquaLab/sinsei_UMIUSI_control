#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_HARMONY_BMS_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_HARMONY_BMS_MODEL_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

#include "sinsei_umiusi_control/hardware_model/interface/can.hpp"
#include "sinsei_umiusi_control/state/bms.hpp"

// VESC BMS CANプロトコルに基づいたHarmony 16 BMSに対応
// ref: https://github.com/vedderb/bldc/blob/4fd8279ea45a17c0d69357438ae2f7237a32514f/datatypes.h
// ref: https://github.com/vedderb/bldc/blob/4fd8279ea45a17c0d69357438ae2f7237a32514f/bms.c

namespace sinsei_umiusi_control::hardware_model::can {

class HarmonyBmsModel {
  public:
    using Id = uint8_t;

    // 0〜2: セル、3: MOSFET、4: 周囲温度、5〜: 基板上の追加温度センサー
    static constexpr std::size_t MOSFET_TEMPERATURE_INDEX = 3;
    static constexpr std::size_t AMBIENT_TEMPERATURE_INDEX = 4;
    static constexpr std::size_t ADDITIONAL_TEMPERATURE_OFFSET = 5;
    static constexpr std::size_t TEMPERATURE_COUNT =
        ADDITIONAL_TEMPERATURE_OFFSET + state::bms::ADDITIONAL_TEMPERATURE_COUNT;
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

    struct State {
        State();

        state::bms::Voltages voltages;
        state::bms::Currents currents;
        state::bms::CapacityState capacity;
        state::bms::CellVoltageRange cell_voltage_range;
        state::bms::Status status;
        state::bms::CellCount cell_count;
        std::array<state::bms::Cell, state::bms::CELL_COUNT> cells;
        state::bms::Temperature balance_ic_temperature;
        state::bms::Temperature mosfet_temperature;
        state::bms::Temperature ambient_temperature;
        std::array<state::bms::Temperature, state::bms::ADDITIONAL_TEMPERATURE_COUNT>
            additional_temperatures;

        std::string status_text;
        // ステータス文字列を全て受信したかどうか
        bool status_updated;
    };

  private:
    Id id;
    State state;
    // 複数フレームに分割された温度とステータス文字列を組み立てるためのバッファ
    std::array<double, TEMPERATURE_COUNT> temperatures{};
    std::size_t temperature_count = 0;
    std::array<char, STATUS_LENGTH> status_buffer{};
    std::size_t contiguous_cells = 0;
    std::size_t contiguous_temperatures = 0;
    uint8_t status_received_mask = 0;

    auto id_matches(const interface::CanFrame & frame) const -> bool;
    auto update_temperatures() -> void;
    auto update_status() -> void;

  public:
    explicit HarmonyBmsModel(Id id);
    auto get_id() const -> Id;

    auto decode(const interface::CanFrame & frame)
        -> tl::expected<std::optional<State>, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model::can

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_HARMONY_BMS_MODEL_HPP
