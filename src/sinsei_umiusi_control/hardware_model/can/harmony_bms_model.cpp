#include "sinsei_umiusi_control/hardware_model/can/harmony_bms_model.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>

#include "sinsei_umiusi_control/util/byte.hpp"
#include "sinsei_umiusi_control/util/string.hpp"

using namespace sinsei_umiusi_control::hardware_model;

namespace {

auto parse_failed(const interface::CanFrame & frame) -> tl::unexpected<std::string> {
    const auto packet_id = (frame.id >> 8) & 0xFFU;
    return tl::make_unexpected("Failed to parse Harmony BMS packet " + std::to_string(packet_id));
}

}  // namespace

can::HarmonyBmsModel::HarmonyBmsModel(Id id) : id(id) {}

auto can::HarmonyBmsModel::get_id() const -> Id { return this->id; }

auto can::HarmonyBmsModel::id_matches(const interface::CanFrame & frame) const -> bool {
    const auto bms_id = static_cast<Id>(frame.id & 0xFFU);
    return frame.is_extended && bms_id == this->id;
}

auto can::HarmonyBmsModel::validate_frame_length(
    const interface::CanFrame & frame, uint8_t min_length, uint8_t max_length,
    uint8_t step) -> tl::expected<void, std::string> {
    if (frame.len >= min_length && frame.len <= max_length &&
        (frame.len - min_length) % step == 0) {
        return {};
    }

    // 許容される長さを"8", "1 to 8", "4, 6 or 8"の形式で表す
    auto expected = std::to_string(min_length);
    if (step == 1 && min_length != max_length) {
        expected += " to " + std::to_string(max_length);
    } else {
        for (auto length = min_length + step; length <= max_length; length += step) {
            expected += (length + step > max_length ? " or " : ", ") + std::to_string(length);
        }
    }
    const auto packet_id = (frame.id >> 8) & 0xFFU;
    return tl::make_unexpected(
        "Received Harmony BMS packet " + std::to_string(packet_id) +
        " with invalid length (expected: " + expected + ", received: " + std::to_string(frame.len) +
        ")");
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
        util::contains(text, "FLT_PCHG"),
        util::contains(text, "FLT_PSW_SHORT"),
        util::contains(text, "FLT_PSW_OT"),
        util::contains(text, "FLT_CHG_OC"),
    };

    auto power_switch_state = util::BmsPowerSwitchState::Unknown;
    if (util::has_bms_fault(faults)) {
        power_switch_state = util::BmsPowerSwitchState::Fault;
    } else if (util::contains(text, "PSW_PCHG")) {
        power_switch_state = util::BmsPowerSwitchState::Precharge;
    } else if (util::contains(text, "PSW_ON")) {
        power_switch_state = util::BmsPowerSwitchState::On;
    } else if (util::contains(text, "PSW_OFF")) {
        power_switch_state = util::BmsPowerSwitchState::Off;
    } else if (util::contains(text, "PSW_WAIT")) {
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
            const auto validation_res = validate_frame_length(frame, 8, 8);
            if (!validation_res) {
                return tl::make_unexpected(validation_res.error());
            }
            const auto pack = util::to_vesc_float32_auto_be(frame.data, 0);
            const auto charger = util::to_vesc_float32_auto_be(frame.data, 4);
            if (!pack || !charger) {
                return parse_failed(frame);
            }
            return PacketVoltage{pack.value(), charger.value()};
        }
        case PacketCurrent::ID: {
            const auto validation_res = validate_frame_length(frame, 8, 8);
            if (!validation_res) {
                return tl::make_unexpected(validation_res.error());
            }
            const auto input = util::to_vesc_float32_auto_be(frame.data, 0);
            const auto measured = util::to_vesc_float32_auto_be(frame.data, 4);
            if (!input || !measured) {
                return parse_failed(frame);
            }
            return PacketCurrent{input.value(), measured.value()};
        }
        case PacketCounters::ID: {
            const auto validation_res = validate_frame_length(frame, 8, 8);
            if (!validation_res) {
                return tl::make_unexpected(validation_res.error());
            }
            const auto amp_hour = util::to_vesc_float32_auto_be(frame.data, 0);
            const auto watt_hour = util::to_vesc_float32_auto_be(frame.data, 4);
            if (!amp_hour || !watt_hour) {
                return parse_failed(frame);
            }
            return PacketCounters{amp_hour.value(), watt_hour.value()};
        }
        case PacketCellVoltage::ID: {
            const auto validation_res = validate_frame_length(frame, 4, 8, 2);
            if (!validation_res) {
                return tl::make_unexpected(validation_res.error());
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
            const auto validation_res = validate_frame_length(frame, 8, 8);
            if (!validation_res) {
                return tl::make_unexpected(validation_res.error());
            }
            // 先頭1バイトがセル数、残り7バイトが各セルのバランシング状態のビット列
            const auto raw = util::to_uint_be<uint64_t>(frame.data, 0);
            if (!raw) {
                return parse_failed(frame);
            }
            const auto bits = raw.value() & 0x00FFFFFFFFFFFFFFULL;
            auto packet = PacketBalancing{};
            packet.cell_count = static_cast<uint8_t>(raw.value() >> 56);
            for (std::size_t i = 0; i < packet.balancing.size(); ++i) {
                packet.balancing[i] = i < packet.cell_count && ((bits >> i) & 1U) != 0U;
            }
            return packet;
        }
        case PacketTemperatures::ID: {
            const auto validation_res = validate_frame_length(frame, 4, 8, 2);
            if (!validation_res) {
                return tl::make_unexpected(validation_res.error());
            }
            const auto offset = util::to_uint8(frame.data, 0);
            const auto temperature_count = util::to_uint8(frame.data, 1);
            if (!offset || !temperature_count) {
                return parse_failed(frame);
            }
            auto packet = PacketTemperatures{};
            packet.offset = offset.value();
            packet.temperature_count = temperature_count.value();
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
            const auto validation_res = validate_frame_length(frame, 6, 8, 2);
            if (!validation_res) {
                return tl::make_unexpected(validation_res.error());
            }
            const auto scaled_temperature = util::to_int16_be(frame.data, 0);
            const auto scaled_humidity = util::to_int16_be(frame.data, 2);
            const auto scaled_balance_ic_temperature = util::to_int16_be(frame.data, 4);
            if (!scaled_temperature || !scaled_humidity || !scaled_balance_ic_temperature) {
                return parse_failed(frame);
            }
            auto packet = PacketHumidity{
                static_cast<double>(scaled_temperature.value()) / PacketHumidity::TEMPERATURE_SCALE,
                static_cast<double>(scaled_humidity.value()) / PacketHumidity::HUMIDITY_SCALE,
                static_cast<double>(scaled_balance_ic_temperature.value()) /
                    PacketHumidity::TEMPERATURE_SCALE,
                std::nullopt,
            };
            if (frame.len == 8) {
                const auto scaled_pressure = util::to_int16_be(frame.data, 6);
                if (!scaled_pressure) {
                    return parse_failed(frame);
                }
                packet.pressure =
                    static_cast<double>(scaled_pressure.value()) / PacketHumidity::PRESSURE_SCALE;
            }
            return packet;
        }
        case PacketSummary::ID: {
            const auto validation_res = validate_frame_length(frame, 8, 8);
            if (!validation_res) {
                return tl::make_unexpected(validation_res.error());
            }
            const auto scaled_cell_voltage_min = util::to_int16_be(frame.data, 0);
            const auto scaled_cell_voltage_max = util::to_int16_be(frame.data, 2);
            const auto scaled_state_of_charge = util::to_uint8(frame.data, 4);
            const auto scaled_state_of_health = util::to_uint8(frame.data, 5);
            const auto cell_temperature_max = util::to_uint8(frame.data, 6);
            const auto flags = util::to_uint8(frame.data, 7);
            if (!scaled_cell_voltage_min || !scaled_cell_voltage_max || !scaled_state_of_charge ||
                !scaled_state_of_health || !cell_temperature_max || !flags) {
                return parse_failed(frame);
            }
            return PacketSummary{
                static_cast<double>(scaled_cell_voltage_min.value()) /
                    PacketSummary::CELL_VOLTAGE_SCALE,
                static_cast<double>(scaled_cell_voltage_max.value()) /
                    PacketSummary::CELL_VOLTAGE_SCALE,
                static_cast<double>(scaled_state_of_charge.value()) / PacketSummary::RATIO_SCALE,
                static_cast<double>(scaled_state_of_health.value()) / PacketSummary::RATIO_SCALE,
                // セルの最高温度は符号付き1バイトで格納されている
                static_cast<double>(static_cast<int8_t>(cell_temperature_max.value())),
                (flags.value() & (1U << 0)) != 0U,
                (flags.value() & (1U << 1)) != 0U,
                (flags.value() & (1U << 2)) != 0U,
                static_cast<uint8_t>((flags.value() >> 4) & 0x0FU),
            };
        }
        case PacketChargeTotals::ID: {
            const auto validation_res = validate_frame_length(frame, 8, 8);
            if (!validation_res) {
                return tl::make_unexpected(validation_res.error());
            }
            const auto amp_hour = util::to_vesc_float32_auto_be(frame.data, 0);
            const auto watt_hour = util::to_vesc_float32_auto_be(frame.data, 4);
            if (!amp_hour || !watt_hour) {
                return parse_failed(frame);
            }
            return PacketChargeTotals{amp_hour.value(), watt_hour.value()};
        }
        case PacketDischargeTotals::ID: {
            const auto validation_res = validate_frame_length(frame, 8, 8);
            if (!validation_res) {
                return tl::make_unexpected(validation_res.error());
            }
            const auto amp_hour = util::to_vesc_float32_auto_be(frame.data, 0);
            const auto watt_hour = util::to_vesc_float32_auto_be(frame.data, 4);
            if (!amp_hour || !watt_hour) {
                return parse_failed(frame);
            }
            return PacketDischargeTotals{amp_hour.value(), watt_hour.value()};
        }
        case PacketId::Status1:
        case PacketId::Status2:
        case PacketId::Status3:
        case PacketId::Status4:
        case PacketId::Status5: {
            const auto validation_res = validate_frame_length(frame, 1, 8);
            if (!validation_res) {
                return tl::make_unexpected(validation_res.error());
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
