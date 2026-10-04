#include "sinsei_umiusi_control/hardware_model/can/harmony_bms_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#include "sinsei_umiusi_control/util/byte.hpp"

using namespace sinsei_umiusi_control::hardware_model;

namespace {

auto byte_at(const interface::CanFrame::Data & data, std::size_t offset) -> uint8_t {
    return std::to_integer<uint8_t>(data[offset]);
}

// VESC独自のfloat32_auto形式で格納された4バイトをdoubleへ変換する
auto float32_auto(const interface::CanFrame::Data & data, std::size_t offset) -> double {
    const auto raw = sinsei_umiusi_control::util::to_uint32_be(data, offset).value();
    auto exponent = static_cast<int>((raw >> 23) & 0xFFU);
    const auto significand_raw = raw & 0x7FFFFFU;
    double significand = 0.0;
    if (exponent != 0 || significand_raw != 0) {
        significand = static_cast<double>(significand_raw) / (8388608.0 * 2.0) + 0.5;
        exponent -= 126;
    }
    if ((raw & 0x80000000U) != 0U) {
        significand = -significand;
    }
    return std::ldexp(significand, exponent);
}

auto require_length(const interface::CanFrame & frame, uint8_t expected)
    -> tl::expected<void, std::string> {
    if (frame.len != expected) {
        const auto packet_id = (frame.id >> 8) & 0xFFU;
        return tl::make_unexpected(
            "Harmony BMS packet " + std::to_string(packet_id) + " has invalid length: expected " +
            std::to_string(expected) + ", received " + std::to_string(frame.len));
    }
    return {};
}

auto contains(const std::string & value, const std::string & token) -> bool {
    return value.find(token) != std::string::npos;
}

auto to_temperature(double raw, std::size_t index, std::size_t received_count)
    -> sinsei_umiusi_control::state::bms::Temperature {
    // まだ受信していない温度はNaNとする
    return sinsei_umiusi_control::state::bms::Temperature{
        index < received_count ? raw : std::numeric_limits<double>::quiet_NaN()};
}

}  // namespace

can::HarmonyBmsModel::State::State() {
    // 未受信の値はNaNとする
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    this->voltages = state::bms::Voltages{nan, nan};
    this->currents = state::bms::Currents{nan, nan};
    this->capacity = state::bms::CapacityState{nan, nan};
    this->cell_voltage_range = state::bms::CellVoltageRange{nan, nan};
    this->status = state::bms::Status{
        util::BmsFaults{false, false, false, false}, util::BmsPowerSwitchState::Unknown, false,
        false, false};
    this->cell_count = state::bms::CellCount{0};
    this->cells.fill(state::bms::Cell{nan, false});
    this->balance_ic_temperature = state::bms::Temperature{nan};
    this->mosfet_temperature = state::bms::Temperature{nan};
    this->ambient_temperature = state::bms::Temperature{nan};
    this->additional_temperatures.fill(state::bms::Temperature{nan});
    this->status_updated = false;
}

can::HarmonyBmsModel::HarmonyBmsModel(Id id) : id(id) {}

auto can::HarmonyBmsModel::get_id() const -> Id { return this->id; }

auto can::HarmonyBmsModel::id_matches(const interface::CanFrame & frame) const -> bool {
    const auto bms_id = static_cast<Id>(frame.id & 0xFFU);
    return frame.is_extended && bms_id == this->id;
}

auto can::HarmonyBmsModel::update_temperatures() -> void {
    this->state.mosfet_temperature = to_temperature(
        this->temperatures[MOSFET_TEMPERATURE_INDEX], MOSFET_TEMPERATURE_INDEX,
        this->temperature_count);
    this->state.ambient_temperature = to_temperature(
        this->temperatures[AMBIENT_TEMPERATURE_INDEX], AMBIENT_TEMPERATURE_INDEX,
        this->temperature_count);
    for (std::size_t i = 0; i < this->state.additional_temperatures.size(); ++i) {
        const auto index = ADDITIONAL_TEMPERATURE_OFFSET + i;
        this->state.additional_temperatures[i] =
            to_temperature(this->temperatures[index], index, this->temperature_count);
    }
}

auto can::HarmonyBmsModel::update_status() -> void {
    const auto end = std::find(this->status_buffer.begin(), this->status_buffer.end(), '\0');
    this->state.status_text = std::string(this->status_buffer.begin(), end);
    const auto & status_text = this->state.status_text;

    auto & status = this->state.status;
    status.faults = util::BmsFaults{
        contains(status_text, "FLT_PCHG"),
        contains(status_text, "FLT_PSW_SHORT"),
        contains(status_text, "FLT_PSW_OT"),
        contains(status_text, "FLT_CHG_OC"),
    };

    if (util::has_bms_fault(status.faults)) {
        status.power_switch_state = util::BmsPowerSwitchState::Fault;
    } else if (contains(status_text, "PSW_PCHG")) {
        status.power_switch_state = util::BmsPowerSwitchState::Precharge;
    } else if (contains(status_text, "PSW_ON")) {
        status.power_switch_state = util::BmsPowerSwitchState::On;
    } else if (contains(status_text, "PSW_OFF")) {
        status.power_switch_state = util::BmsPowerSwitchState::Off;
    } else if (contains(status_text, "PSW_WAIT")) {
        status.power_switch_state = util::BmsPowerSwitchState::Initializing;
    } else {
        status.power_switch_state = util::BmsPowerSwitchState::Unknown;
    }
}

auto can::HarmonyBmsModel::decode(const interface::CanFrame & frame)
    -> tl::expected<std::optional<State>, std::string> {
    if (!this->id_matches(frame)) {
        return std::nullopt;
    }

    this->state.status_updated = false;

    const auto packet_id = static_cast<uint8_t>((frame.id >> 8) & 0xFFU);
    switch (static_cast<PacketId>(packet_id)) {
        case PacketId::Voltage: {
            const auto length = require_length(frame, 8);
            if (!length) {
                return tl::make_unexpected(length.error());
            }
            this->state.voltages.pack = float32_auto(frame.data, 0);
            this->state.voltages.charger = float32_auto(frame.data, 4);
            break;
        }
        case PacketId::Current: {
            const auto length = require_length(frame, 8);
            if (!length) {
                return tl::make_unexpected(length.error());
            }
            this->state.currents.input = float32_auto(frame.data, 0);
            this->state.currents.measured = float32_auto(frame.data, 4);
            break;
        }
        case PacketId::Counters:
        case PacketId::ChargeTotals:
        case PacketId::DischargeTotals: {
            // 累積値は使用しない
            const auto length = require_length(frame, 8);
            if (!length) {
                return tl::make_unexpected(length.error());
            }
            break;
        }
        case PacketId::CellVoltage: {
            if (frame.len < 4 || frame.len > 8 || (frame.len % 2) != 0) {
                return tl::make_unexpected(
                    "Harmony BMS cell-voltage packet has invalid length: " +
                    std::to_string(frame.len));
            }
            auto offset = static_cast<std::size_t>(byte_at(frame.data, 0));
            const auto count =
                std::min<std::size_t>(byte_at(frame.data, 1), state::bms::CELL_COUNT);
            if (offset == 0) {
                this->contiguous_cells = 0;
            }
            const auto contiguous = offset == this->contiguous_cells;
            for (std::size_t data_offset = 2; data_offset + 1 < frame.len; data_offset += 2) {
                if (offset < state::bms::CELL_COUNT) {
                    this->state.cells[offset].voltage =
                        static_cast<double>(util::to_int16_be(frame.data, data_offset).value()) /
                        1000.0;
                }
                ++offset;
            }
            if (contiguous) {
                this->contiguous_cells = offset;
                if (this->contiguous_cells >= count) {
                    this->state.cell_count = state::bms::CellCount{static_cast<uint8_t>(count)};
                }
            }
            break;
        }
        case PacketId::Balancing: {
            const auto length = require_length(frame, 8);
            if (!length) {
                return tl::make_unexpected(length.error());
            }
            const auto count =
                std::min<std::size_t>(byte_at(frame.data, 0), state::bms::CELL_COUNT);
            uint64_t bits = 0;
            for (std::size_t i = 1; i < 8; ++i) {
                bits = (bits << 8) | byte_at(frame.data, i);
            }
            for (std::size_t i = 0; i < this->state.cells.size(); ++i) {
                this->state.cells[i].balancing = i < count && ((bits >> i) & 1U) != 0U;
            }
            break;
        }
        case PacketId::Temperatures: {
            if (frame.len < 4 || frame.len > 8 || (frame.len % 2) != 0) {
                return tl::make_unexpected(
                    "Harmony BMS temperature packet has invalid length: " +
                    std::to_string(frame.len));
            }
            auto offset = static_cast<std::size_t>(byte_at(frame.data, 0));
            const auto count = std::min<std::size_t>(byte_at(frame.data, 1), TEMPERATURE_COUNT);
            if (offset == 0) {
                this->contiguous_temperatures = 0;
            }
            const auto contiguous = offset == this->contiguous_temperatures;
            for (std::size_t data_offset = 2; data_offset + 1 < frame.len; data_offset += 2) {
                if (offset < TEMPERATURE_COUNT) {
                    this->temperatures[offset] =
                        static_cast<double>(util::to_int16_be(frame.data, data_offset).value()) /
                        100.0;
                }
                ++offset;
            }
            if (contiguous) {
                this->contiguous_temperatures = offset;
                if (this->contiguous_temperatures >= count) {
                    this->temperature_count = count;
                }
            }
            this->update_temperatures();
            break;
        }
        case PacketId::Humidity: {
            if (frame.len != 6 && frame.len != 8) {
                return tl::make_unexpected(
                    "Harmony BMS humidity packet has invalid length: " + std::to_string(frame.len));
            }
            // 湿度センサーは互換基板に搭載されていないため、バランスICの温度のみ取り出す
            this->state.balance_ic_temperature = state::bms::Temperature{
                static_cast<double>(util::to_int16_be<4>(frame.data).value()) / 100.0};
            break;
        }
        case PacketId::Summary: {
            const auto length = require_length(frame, 8);
            if (!length) {
                return tl::make_unexpected(length.error());
            }
            this->state.cell_voltage_range.min =
                static_cast<double>(util::to_int16_be<0>(frame.data).value()) / 1000.0;
            this->state.cell_voltage_range.max =
                static_cast<double>(util::to_int16_be<2>(frame.data).value()) / 1000.0;
            this->state.capacity.state_of_charge =
                static_cast<double>(byte_at(frame.data, 4)) / 255.0;
            this->state.capacity.state_of_health =
                static_cast<double>(byte_at(frame.data, 5)) / 255.0;
            const auto flags = byte_at(frame.data, 7);
            this->state.status.charging = (flags & (1U << 0)) != 0U;
            this->state.status.balancing = (flags & (1U << 1)) != 0U;
            this->state.status.charge_allowed = (flags & (1U << 2)) != 0U;
            break;
        }
        case PacketId::Status1:
        case PacketId::Status2:
        case PacketId::Status3:
        case PacketId::Status4:
        case PacketId::Status5: {
            if (frame.len == 0 || frame.len > 8) {
                return tl::make_unexpected(
                    "Harmony BMS status packet has invalid length: " + std::to_string(frame.len));
            }
            const auto chunk = packet_id - static_cast<uint8_t>(PacketId::Status1);
            const auto destination = static_cast<std::size_t>(chunk) * 8;
            if (chunk == 0) {
                this->status_buffer.fill('\0');
                this->status_received_mask = 0;
            }
            std::fill_n(this->status_buffer.begin() + destination, 8, '\0');
            for (std::size_t i = 0; i < frame.len; ++i) {
                this->status_buffer[destination + i] = static_cast<char>(byte_at(frame.data, i));
            }
            this->status_received_mask |= static_cast<uint8_t>(1U << chunk);
            constexpr uint8_t ALL_STATUS_CHUNKS_RECEIVED = 0x1FU;
            if (chunk == 4 && this->status_received_mask == ALL_STATUS_CHUNKS_RECEIVED) {
                this->state.status_updated = true;
                this->update_status();
                this->status_received_mask = 0;
            }
            break;
        }
        default:
            return tl::make_unexpected(
                "Harmony BMS received unknown packet ID: " + std::to_string(packet_id));
    }

    return this->state;
}
