#include "sinsei_umiusi_control/hardware_model/can/harmony_bms_model.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>

#include "sinsei_umiusi_control/util/byte.hpp"

using namespace sinsei_umiusi_control::hardware_model;

namespace {

auto contains(const std::string & value, const std::string & token) -> bool {
    return value.find(token) != std::string::npos;
}

auto invalid_length(const interface::CanFrame & frame, const std::string & expected)
    -> tl::unexpected<std::string> {
    const auto packet_id = (frame.id >> 8) & 0xFFU;
    return tl::make_unexpected(
        "Received Harmony BMS packet " + std::to_string(packet_id) +
        " with invalid length (expected: " + expected + ", received: " + std::to_string(frame.len) +
        ")");
}

auto parse_failed(const interface::CanFrame & frame) -> tl::unexpected<std::string> {
    const auto packet_id = (frame.id >> 8) & 0xFFU;
    return tl::make_unexpected("Failed to parse Harmony BMS packet " + std::to_string(packet_id));
}

// 先頭2バイトにオフセットと総数、以降に2バイトずつ値を持つフレームか
auto is_valid_indexed_frame_length(const interface::CanFrame & frame) -> bool {
    return frame.len >= 4 && frame.len <= 8 && (frame.len % 2) == 0;
}

}  // namespace

can::HarmonyBmsModel::HarmonyBmsModel(Id id) : id(id) {}

auto can::HarmonyBmsModel::get_id() const -> Id { return this->id; }

auto can::HarmonyBmsModel::id_matches(const interface::CanFrame & frame) const -> bool {
    const auto bms_id = static_cast<Id>(frame.id & 0xFFU);
    return frame.is_extended && bms_id == this->id;
}

auto can::HarmonyBmsModel::decode_status_chunk(const interface::CanFrame & frame, std::size_t chunk)
    -> std::optional<PacketStatus> {
    // Status1から順に受信した場合のみ組み立てる
    // 途中を取りこぼした場合は、前の周期のチャンクが混ざらないよう次のStatus1まで無視する
    if (chunk == 0) {
        this->status_buffer.fill('\0');
        this->next_status_chunk = 0;
    }
    if (chunk != this->next_status_chunk) {
        this->next_status_chunk = 0;
        return std::nullopt;
    }

    const auto destination = chunk * 8;
    std::fill_n(this->status_buffer.begin() + destination, 8, '\0');
    for (std::size_t i = 0; i < frame.len; ++i) {
        this->status_buffer[destination + i] = std::to_integer<char>(frame.data[i]);
    }
    this->next_status_chunk = chunk + 1;

    constexpr std::size_t STATUS_CHUNK_COUNT = STATUS_LENGTH / 8;
    if (this->next_status_chunk < STATUS_CHUNK_COUNT) {
        return std::nullopt;
    }
    this->next_status_chunk = 0;

    const auto end = std::find(this->status_buffer.begin(), this->status_buffer.end(), '\0');
    const auto text = std::string(this->status_buffer.begin(), end);
    const auto faults = util::BmsFaults{
        contains(text, "FLT_PCHG"),
        contains(text, "FLT_PSW_SHORT"),
        contains(text, "FLT_PSW_OT"),
        contains(text, "FLT_CHG_OC"),
    };

    auto power_switch_state = util::BmsPowerSwitchState::Unknown;
    if (util::has_bms_fault(faults)) {
        power_switch_state = util::BmsPowerSwitchState::Fault;
    } else if (contains(text, "PSW_PCHG")) {
        power_switch_state = util::BmsPowerSwitchState::Precharge;
    } else if (contains(text, "PSW_ON")) {
        power_switch_state = util::BmsPowerSwitchState::On;
    } else if (contains(text, "PSW_OFF")) {
        power_switch_state = util::BmsPowerSwitchState::Off;
    } else if (contains(text, "PSW_WAIT")) {
        power_switch_state = util::BmsPowerSwitchState::Initializing;
    }

    return PacketStatus{faults, power_switch_state, text};
}

auto can::HarmonyBmsModel::decode(const interface::CanFrame & frame)
    -> tl::expected<std::optional<AnyPacket>, std::string> {
    if (!this->id_matches(frame)) {
        return std::nullopt;
    }

    const auto packet_id = static_cast<PacketId>((frame.id >> 8) & 0xFFU);
    switch (packet_id) {
        case PacketVoltage::ID: {
            if (frame.len != 8) {
                return invalid_length(frame, "8");
            }
            const auto pack = util::to_float32_auto_be(frame.data, 0);
            const auto charger = util::to_float32_auto_be(frame.data, 4);
            if (!pack || !charger) {
                return parse_failed(frame);
            }
            return PacketVoltage{pack.value(), charger.value()};
        }
        case PacketCurrent::ID: {
            if (frame.len != 8) {
                return invalid_length(frame, "8");
            }
            const auto input = util::to_float32_auto_be(frame.data, 0);
            const auto measured = util::to_float32_auto_be(frame.data, 4);
            if (!input || !measured) {
                return parse_failed(frame);
            }
            return PacketCurrent{input.value(), measured.value()};
        }
        case PacketId::Counters:
        case PacketId::ChargeTotals:
        case PacketId::DischargeTotals: {
            // 累積値は使用しない
            if (frame.len != 8) {
                return invalid_length(frame, "8");
            }
            return std::monostate{};
        }
        case PacketCellVoltage::ID: {
            if (!is_valid_indexed_frame_length(frame)) {
                return invalid_length(frame, "4, 6 or 8");
            }
            const auto offset = util::to_uint8(frame.data, 0);
            const auto cell_count = util::to_uint8(frame.data, 1);
            if (!offset || !cell_count) {
                return parse_failed(frame);
            }
            auto packet = PacketCellVoltage{};
            packet.offset = offset.value();
            packet.cell_count = cell_count.value();
            packet.value_count = static_cast<uint8_t>((frame.len - 2) / 2);
            for (std::size_t i = 0; i < packet.value_count; ++i) {
                const auto scaled_voltage = util::to_int16_be(frame.data, 2 + i * 2);
                if (!scaled_voltage) {
                    return parse_failed(frame);
                }
                packet.voltages[i] =
                    static_cast<double>(scaled_voltage.value()) / PacketCellVoltage::VOLTAGE_SCALE;
            }
            return packet;
        }
        case PacketBalancing::ID: {
            if (frame.len != 8) {
                return invalid_length(frame, "8");
            }
            // 先頭1バイトがセル数、残り7バイトが各セルのバランシング状態のビット列
            const auto raw = util::to_uint_be<uint64_t>(frame.data, 0);
            if (!raw) {
                return parse_failed(frame);
            }
            const auto cell_count = static_cast<std::size_t>(raw.value() >> 56);
            const auto bits = raw.value() & 0x00FFFFFFFFFFFFFFULL;
            auto packet = PacketBalancing{};
            for (std::size_t i = 0; i < packet.balancing.size(); ++i) {
                packet.balancing[i] = i < cell_count && ((bits >> i) & 1U) != 0U;
            }
            return packet;
        }
        case PacketTemperatures::ID: {
            if (!is_valid_indexed_frame_length(frame)) {
                return invalid_length(frame, "4, 6 or 8");
            }
            const auto offset = util::to_uint8(frame.data, 0);
            if (!offset) {
                return parse_failed(frame);
            }
            auto packet = PacketTemperatures{};
            packet.offset = offset.value();
            packet.value_count = static_cast<uint8_t>((frame.len - 2) / 2);
            for (std::size_t i = 0; i < packet.value_count; ++i) {
                const auto scaled_temperature = util::to_int16_be(frame.data, 2 + i * 2);
                if (!scaled_temperature) {
                    return parse_failed(frame);
                }
                packet.temperatures[i] = static_cast<double>(scaled_temperature.value()) /
                                         PacketTemperatures::TEMPERATURE_SCALE;
            }
            return packet;
        }
        case PacketHumidity::ID: {
            if (frame.len != 6 && frame.len != 8) {
                return invalid_length(frame, "6 or 8");
            }
            const auto scaled_temperature = util::to_int16_be(frame.data, 4);
            if (!scaled_temperature) {
                return parse_failed(frame);
            }
            return PacketHumidity{
                static_cast<double>(scaled_temperature.value()) /
                PacketHumidity::TEMPERATURE_SCALE};
        }
        case PacketSummary::ID: {
            if (frame.len != 8) {
                return invalid_length(frame, "8");
            }
            const auto scaled_cell_voltage_min = util::to_int16_be(frame.data, 0);
            const auto scaled_cell_voltage_max = util::to_int16_be(frame.data, 2);
            const auto scaled_state_of_charge = util::to_uint8(frame.data, 4);
            const auto scaled_state_of_health = util::to_uint8(frame.data, 5);
            const auto flags = util::to_uint8(frame.data, 7);
            if (!scaled_cell_voltage_min || !scaled_cell_voltage_max || !scaled_state_of_charge ||
                !scaled_state_of_health || !flags) {
                return parse_failed(frame);
            }
            return PacketSummary{
                static_cast<double>(scaled_cell_voltage_min.value()) /
                    PacketSummary::CELL_VOLTAGE_SCALE,
                static_cast<double>(scaled_cell_voltage_max.value()) /
                    PacketSummary::CELL_VOLTAGE_SCALE,
                static_cast<double>(scaled_state_of_charge.value()) / PacketSummary::RATIO_SCALE,
                static_cast<double>(scaled_state_of_health.value()) / PacketSummary::RATIO_SCALE,
                (flags.value() & (1U << 0)) != 0U,
                (flags.value() & (1U << 1)) != 0U,
                (flags.value() & (1U << 2)) != 0U,
            };
        }
        case PacketId::Status1:
        case PacketId::Status2:
        case PacketId::Status3:
        case PacketId::Status4:
        case PacketId::Status5: {
            if (frame.len == 0 || frame.len > 8) {
                return invalid_length(frame, "1 to 8");
            }
            const auto chunk =
                static_cast<std::size_t>(packet_id) - static_cast<std::size_t>(PacketId::Status1);
            auto status = this->decode_status_chunk(frame, chunk);
            if (!status) {
                return std::monostate{};
            }
            return std::move(status.value());
        }
        default:
            return tl::make_unexpected(
                "Received Harmony BMS frame with unknown packet ID: " +
                std::to_string(static_cast<uint32_t>(packet_id)));
    }
}
