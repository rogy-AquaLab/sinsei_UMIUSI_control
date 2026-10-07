#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_HARMONY_BMS_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_HARMONY_BMS_MODEL_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>
#include <variant>

#include "sinsei_umiusi_control/hardware_model/interface/can.hpp"
#include "sinsei_umiusi_control/state/bms.hpp"
#include "sinsei_umiusi_control/util/bms_status.hpp"

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

    struct PacketVoltage {
        static constexpr PacketId ID = PacketId::Voltage;

        double pack;
        double charger;
    };

    struct PacketCurrent {
        static constexpr PacketId ID = PacketId::Current;

        double input;
        double measured;
    };

    // 1フレームに最大3セル分の電圧を含む
    struct PacketCellVoltage {
        static constexpr PacketId ID = PacketId::CellVoltage;
        static constexpr std::size_t MAX_VALUE_COUNT = 3;
        static constexpr double VOLTAGE_SCALE = 1000;

        uint8_t offset;      // 先頭のセルの番号
        uint8_t cell_count;  // BMSが報告したセル数
        std::array<double, MAX_VALUE_COUNT> voltages;
        uint8_t value_count;  // このフレームに含まれる電圧の数
    };

    struct PacketBalancing {
        static constexpr PacketId ID = PacketId::Balancing;

        std::array<bool, state::bms::CELL_COUNT> balancing;
    };

    // 1フレームに最大3個分の温度を含む
    struct PacketTemperatures {
        static constexpr PacketId ID = PacketId::Temperatures;
        static constexpr std::size_t MAX_VALUE_COUNT = 3;
        static constexpr double TEMPERATURE_SCALE = 100;

        uint8_t offset;  // 先頭の温度の番号
        std::array<double, MAX_VALUE_COUNT> temperatures;
        uint8_t value_count;  // このフレームに含まれる温度の数
    };

    // 湿度センサーは互換基板に搭載されていないため、バランスICの温度のみ取り出す
    struct PacketHumidity {
        static constexpr PacketId ID = PacketId::Humidity;
        static constexpr double TEMPERATURE_SCALE = 100;

        double balance_ic_temperature;
    };

    struct PacketSummary {
        static constexpr PacketId ID = PacketId::Summary;
        static constexpr double CELL_VOLTAGE_SCALE = 1000;
        static constexpr double RATIO_SCALE = 255;

        double cell_voltage_min;
        double cell_voltage_max;
        double state_of_charge;
        double state_of_health;
        bool charging;
        bool balancing;
        bool charge_allowed;
    };

    // Status1〜5の5フレームが順に揃った時のみ返す
    struct PacketStatus {
        util::BmsFaults faults;
        util::BmsPowerSwitchState power_switch_state;
        std::string text;  // ログ出力用
    };

    // std::monostate: このBMSのフレームだが、返す値がないもの
    // (使用しない累積値のパケット、組み立て途中のステータス文字列)
    using AnyPacket = std::variant<
        std::monostate, PacketVoltage, PacketCurrent, PacketCellVoltage, PacketBalancing,
        PacketTemperatures, PacketHumidity, PacketSummary, PacketStatus>;

  private:
    Id id;
    // 複数フレームに分割されたステータス文字列を組み立てるためのバッファ
    std::array<char, STATUS_LENGTH> status_buffer{};
    std::size_t next_status_chunk = 0;

    auto id_matches(const interface::CanFrame & frame) const -> bool;
    // フレーム長が`min_length`以上`max_length`以下で、`min_length`から`step`刻みかを確認する
    static auto validate_frame_length(
        const interface::CanFrame & frame, uint8_t min_length, uint8_t max_length,
        uint8_t step = 1) -> tl::expected<void, std::string>;
    auto decode_status_chunk(const interface::CanFrame & frame, std::size_t chunk)
        -> std::optional<PacketStatus>;

  public:
    explicit HarmonyBmsModel(Id id);
    auto get_id() const -> Id;

    // 別のノードのフレームの場合はstd::nulloptを返す
    auto decode(const interface::CanFrame & frame)
        -> tl::expected<std::optional<AnyPacket>, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model::can

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_HARMONY_BMS_MODEL_HPP
