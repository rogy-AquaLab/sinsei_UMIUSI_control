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

auto contains(const std::string & value, const std::string & token) -> bool {
    return value.find(token) != std::string::npos;
}

}  // namespace

can::HarmonyBmsModel::State::State() {
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    this->pack_voltage = nan;
    this->charger_voltage = nan;
    this->input_current = nan;
    this->measured_current = nan;
    this->net_consumed_charge = nan;
    this->net_consumed_energy = nan;
    this->cell_voltages.fill(nan);
    this->temperatures.fill(nan);
    this->humidity_sensor_temperature = nan;
    this->relative_humidity = nan;
    this->balance_ic_temperature = nan;
    this->state_of_charge = nan;
    this->state_of_health = nan;
    this->cell_voltage_min = nan;
    this->cell_voltage_max = nan;
    this->cell_temperature_max = nan;
    this->total_charged_charge = nan;
    this->total_charged_energy = nan;
    this->total_discharged_charge = nan;
    this->total_discharged_energy = nan;
}

can::HarmonyBmsModel::HarmonyBmsModel(Id id) : id(id) {}

auto can::HarmonyBmsModel::get_id() const -> Id { return this->id; }

auto can::HarmonyBmsModel::id_matches(const interface::CanFrame & frame) const -> bool {
    const auto bms_id = static_cast<Id>(frame.id & 0xFFU);
    return frame.is_extended && bms_id == this->id;
}

auto can::HarmonyBmsModel::update_status_flags() -> void {
    const auto end = std::find(this->state.status.begin(), this->state.status.end(), '\0');
    const auto status_text = std::string(this->state.status.begin(), end);

    uint32_t flags = FAULT_NONE;
    if (contains(status_text, "FLT_PCHG")) {
        flags |= FAULT_PRECHARGE;
    }
    if (contains(status_text, "FLT_PSW_SHORT")) {
        flags |= FAULT_SHORT_CIRCUIT;
    }
    if (contains(status_text, "FLT_PSW_OT")) {
        flags |= FAULT_SWITCH_OVER_TEMPERATURE;
    }
    if (contains(status_text, "FLT_CHG_OC")) {
        flags |= FAULT_CHARGE_OVERCURRENT;
    }
    this->state.fault_flags = flags;

    if (flags != FAULT_NONE) {
        this->state.power_switch_state = PowerSwitchState::Fault;
    } else if (contains(status_text, "PSW_PCHG")) {
        this->state.power_switch_state = PowerSwitchState::Precharge;
    } else if (contains(status_text, "PSW_ON")) {
        this->state.power_switch_state = PowerSwitchState::On;
    } else if (contains(status_text, "PSW_OFF")) {
        this->state.power_switch_state = PowerSwitchState::Off;
    } else if (contains(status_text, "PSW_WAIT")) {
        this->state.power_switch_state = PowerSwitchState::Initializing;
    } else {
        this->state.power_switch_state = PowerSwitchState::Unknown;
    }
}

auto can::HarmonyBmsModel::decode(const interface::CanFrame & frame)
    -> tl::expected<std::optional<State>, std::string> {
    if (!this->id_matches(frame)) {
        return std::nullopt;
    }

    this->state.status_updated = false;

    const auto packet_id = static_cast<uint8_t>((frame.id >> 8) & 0xFFU);
    const auto require_length = [&frame,
                                 packet_id](uint8_t expected) -> tl::expected<void, std::string> {
        if (frame.len != expected) {
            return tl::make_unexpected(
                "Harmony BMS packet " + std::to_string(packet_id) +
                " has invalid length: expected " + std::to_string(expected) + ", received " +
                std::to_string(frame.len));
        }
        return {};
    };

    switch (static_cast<PacketId>(packet_id)) {
        case PacketId::Voltage: {
            const auto length = require_length(8);
            if (!length) {
                return tl::make_unexpected(length.error());
            }
            this->state.pack_voltage = float32_auto(frame.data, 0);
            this->state.charger_voltage = float32_auto(frame.data, 4);
            break;
        }
        case PacketId::Current: {
            const auto length = require_length(8);
            if (!length) {
                return tl::make_unexpected(length.error());
            }
            this->state.input_current = float32_auto(frame.data, 0);
            this->state.measured_current = float32_auto(frame.data, 4);
            break;
        }
        case PacketId::Counters: {
            const auto length = require_length(8);
            if (!length) {
                return tl::make_unexpected(length.error());
            }
            this->state.net_consumed_charge = float32_auto(frame.data, 0);
            this->state.net_consumed_energy = float32_auto(frame.data, 4);
            break;
        }
        case PacketId::CellVoltage: {
            if (frame.len < 4 || frame.len > 8 || (frame.len % 2) != 0) {
                return tl::make_unexpected(
                    "Harmony BMS cell-voltage packet has invalid length: " +
                    std::to_string(frame.len));
            }
            auto offset = static_cast<std::size_t>(byte_at(frame.data, 0));
            const auto count = std::min<std::size_t>(byte_at(frame.data, 1), CELL_COUNT);
            if (offset == 0) {
                this->contiguous_cells = 0;
            }
            const auto contiguous = offset == this->contiguous_cells;
            for (std::size_t data_offset = 2; data_offset + 1 < frame.len; data_offset += 2) {
                if (offset < CELL_COUNT) {
                    this->state.cell_voltages[offset] =
                        static_cast<double>(util::to_int16_be(frame.data, data_offset).value()) /
                        1000.0;
                }
                ++offset;
            }
            if (contiguous) {
                this->contiguous_cells = offset;
                if (this->contiguous_cells >= count) {
                    this->state.cell_count = static_cast<uint8_t>(count);
                }
            }
            break;
        }
        case PacketId::Balancing: {
            const auto length = require_length(8);
            if (!length) {
                return tl::make_unexpected(length.error());
            }
            const auto count = std::min<std::size_t>(byte_at(frame.data, 0), CELL_COUNT);
            uint64_t bits = 0;
            for (std::size_t i = 1; i < 8; ++i) {
                bits = (bits << 8) | byte_at(frame.data, i);
            }
            this->state.cell_balancing.fill(false);
            for (std::size_t i = 0; i < count; ++i) {
                this->state.cell_balancing[i] = ((bits >> i) & 1U) != 0U;
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
                    this->state.temperatures[offset] =
                        static_cast<double>(util::to_int16_be(frame.data, data_offset).value()) /
                        100.0;
                }
                ++offset;
            }
            if (contiguous) {
                this->contiguous_temperatures = offset;
                if (this->contiguous_temperatures >= count) {
                    this->state.temperature_count = static_cast<uint8_t>(count);
                }
            }
            break;
        }
        case PacketId::Humidity: {
            if (frame.len != 6 && frame.len != 8) {
                return tl::make_unexpected(
                    "Harmony BMS humidity packet has invalid length: " + std::to_string(frame.len));
            }
            this->state.humidity_sensor_temperature =
                static_cast<double>(util::to_int16_be<0>(frame.data).value()) / 100.0;
            this->state.relative_humidity =
                static_cast<double>(util::to_int16_be<2>(frame.data).value()) / 100.0;
            this->state.balance_ic_temperature =
                static_cast<double>(util::to_int16_be<4>(frame.data).value()) / 100.0;
            break;
        }
        case PacketId::Summary: {
            const auto length = require_length(8);
            if (!length) {
                return tl::make_unexpected(length.error());
            }
            this->state.cell_voltage_min =
                static_cast<double>(util::to_int16_be<0>(frame.data).value()) / 1000.0;
            this->state.cell_voltage_max =
                static_cast<double>(util::to_int16_be<2>(frame.data).value()) / 1000.0;
            this->state.state_of_charge = static_cast<double>(byte_at(frame.data, 4)) / 255.0;
            this->state.state_of_health = static_cast<double>(byte_at(frame.data, 5)) / 255.0;
            this->state.cell_temperature_max =
                static_cast<double>(static_cast<int8_t>(byte_at(frame.data, 6)));
            const auto flags = byte_at(frame.data, 7);
            this->state.charging = (flags & (1U << 0)) != 0U;
            this->state.balancing = (flags & (1U << 1)) != 0U;
            this->state.charge_allowed = (flags & (1U << 2)) != 0U;
            this->state.data_version = static_cast<uint8_t>((flags >> 4) & 0x0FU);
            break;
        }
        case PacketId::ChargeTotals:
        case PacketId::DischargeTotals: {
            const auto length = require_length(8);
            if (!length) {
                return tl::make_unexpected(length.error());
            }
            const auto charge = float32_auto(frame.data, 0);
            const auto energy = float32_auto(frame.data, 4);
            if (static_cast<PacketId>(packet_id) == PacketId::ChargeTotals) {
                this->state.total_charged_charge = charge;
                this->state.total_charged_energy = energy;
            } else {
                this->state.total_discharged_charge = charge;
                this->state.total_discharged_energy = energy;
            }
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
                this->state.status.fill('\0');
                this->status_received_mask = 0;
            }
            std::fill_n(this->state.status.begin() + destination, 8, '\0');
            for (std::size_t i = 0; i < frame.len; ++i) {
                this->state.status[destination + i] = static_cast<char>(byte_at(frame.data, i));
            }
            this->status_received_mask |= static_cast<uint8_t>(1U << chunk);
            constexpr uint8_t ALL_STATUS_CHUNKS_RECEIVED = 0x1FU;
            if (chunk == 4 && this->status_received_mask == ALL_STATUS_CHUNKS_RECEIVED) {
                this->state.status_updated = true;
                this->update_status_flags();
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
