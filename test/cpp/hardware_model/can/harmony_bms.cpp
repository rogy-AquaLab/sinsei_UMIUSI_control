#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>

#include "sinsei_umiusi_control/hardware_model/can/harmony_bms_model.hpp"
#include "sinsei_umiusi_control/hardware_model/interface/can.hpp"

namespace suchm = sinsei_umiusi_control::hardware_model;

namespace sinsei_umiusi_control::test::hardware_model::can::harmony_bms {

namespace {

constexpr uint8_t BMS_ID = 3;

auto make_frame(
    suchm::can::HarmonyBmsModel::PacketId packet_id, suchm::interface::CanFrame::Data data,
    uint8_t length = 8, uint8_t node_id = BMS_ID) -> suchm::interface::CanFrame {
    return {
        (static_cast<uint32_t>(packet_id) << 8) | node_id,
        length,
        data,
        true,
    };
}

auto float_bytes(float value) -> std::array<std::byte, 4> {
    uint32_t raw = 0;
    std::memcpy(&raw, &value, sizeof(raw));
    return {
        std::byte{static_cast<uint8_t>(raw >> 24)},
        std::byte{static_cast<uint8_t>(raw >> 16)},
        std::byte{static_cast<uint8_t>(raw >> 8)},
        std::byte{static_cast<uint8_t>(raw)},
    };
}

auto two_floats(float first, float second) -> suchm::interface::CanFrame::Data {
    const auto first_bytes = float_bytes(first);
    const auto second_bytes = float_bytes(second);
    return {
        first_bytes[0],  first_bytes[1],  first_bytes[2],  first_bytes[3],
        second_bytes[0], second_bytes[1], second_bytes[2], second_bytes[3],
    };
}

}  // namespace

TEST(HarmonyBmsModelTest, ExposesConfiguredId) {
    const auto model = suchm::can::HarmonyBmsModel(BMS_ID);
    EXPECT_EQ(model.get_id(), BMS_ID);
}

TEST(HarmonyBmsModelTest, InitializesUnreceivedMeasurementsToNaN) {
    const auto state = suchm::can::HarmonyBmsModel::State{};
    EXPECT_TRUE(std::isnan(state.voltages.pack));
    EXPECT_TRUE(std::isnan(state.voltages.charger));
    EXPECT_TRUE(std::isnan(state.currents.input));
    EXPECT_TRUE(std::isnan(state.currents.measured));
    EXPECT_TRUE(std::isnan(state.capacity.state_of_charge));
    EXPECT_TRUE(std::isnan(state.capacity.state_of_health));
    EXPECT_TRUE(std::isnan(state.cell_voltage_range.min));
    EXPECT_TRUE(std::isnan(state.cell_voltage_range.max));
    EXPECT_TRUE(std::isnan(state.balance_ic_temperature.value));
    EXPECT_TRUE(std::isnan(state.mosfet_temperature.value));
    EXPECT_TRUE(std::isnan(state.ambient_temperature.value));
    for (const auto & cell : state.cells) {
        EXPECT_TRUE(std::isnan(cell.voltage));
        EXPECT_FALSE(cell.balancing);
    }
    for (const auto & temperature : state.additional_temperatures) {
        EXPECT_TRUE(std::isnan(temperature.value));
    }
    EXPECT_EQ(state.cell_count.value, 0);
    EXPECT_FALSE(sinsei_umiusi_control::util::has_bms_fault(state.status.faults));
    EXPECT_EQ(
        state.status.power_switch_state, sinsei_umiusi_control::util::BmsPowerSwitchState::Unknown);
}

TEST(HarmonyBmsModelTest, IgnoresFramesFromOtherNodes) {
    auto model = suchm::can::HarmonyBmsModel(BMS_ID);
    const auto result = model.decode(make_frame(
        suchm::can::HarmonyBmsModel::PacketId::Voltage, two_floats(48.0F, 50.0F), 8, 11));

    ASSERT_TRUE(result);
    EXPECT_FALSE(result.value().has_value());
}

TEST(HarmonyBmsModelTest, RejectsUnknownPacketFromConfiguredNode) {
    auto model = suchm::can::HarmonyBmsModel(BMS_ID);
    const auto unknown_packet_id = static_cast<suchm::can::HarmonyBmsModel::PacketId>(0x7F);
    const auto result = model.decode(make_frame(unknown_packet_id, {}));

    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Harmony BMS received unknown packet ID: 127");
}

TEST(HarmonyBmsModelTest, DecodesVoltageCurrentAndSummary) {
    auto model = suchm::can::HarmonyBmsModel(BMS_ID);

    auto result = model.decode(
        make_frame(suchm::can::HarmonyBmsModel::PacketId::Voltage, two_floats(48.0F, 50.0F)));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_DOUBLE_EQ(result.value()->voltages.pack, 48.0);
    EXPECT_DOUBLE_EQ(result.value()->voltages.charger, 50.0);

    result = model.decode(
        make_frame(suchm::can::HarmonyBmsModel::PacketId::Current, two_floats(12.5F, -12.25F)));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_DOUBLE_EQ(result.value()->currents.input, 12.5);
    EXPECT_DOUBLE_EQ(result.value()->currents.measured, -12.25);

    result = model.decode(make_frame(
        suchm::can::HarmonyBmsModel::PacketId::Summary,
        {std::byte{0x0E}, std::byte{0x74},  // 3.700 V
         std::byte{0x10}, std::byte{0x04},  // 4.100 V
         std::byte{0x80}, std::byte{0xFF}, std::byte{42}, std::byte{0x17}}));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_DOUBLE_EQ(result.value()->cell_voltage_range.min, 3.7);
    EXPECT_DOUBLE_EQ(result.value()->cell_voltage_range.max, 4.1);
    EXPECT_NEAR(result.value()->capacity.state_of_charge, 128.0 / 255.0, 1e-12);
    EXPECT_DOUBLE_EQ(result.value()->capacity.state_of_health, 1.0);
    EXPECT_TRUE(result.value()->status.charging);
    EXPECT_TRUE(result.value()->status.balancing);
    EXPECT_TRUE(result.value()->status.charge_allowed);
}

TEST(HarmonyBmsModelTest, AcceptsUnusedCounterPackets) {
    auto model = suchm::can::HarmonyBmsModel(BMS_ID);

    for (const auto packet_id : {
             suchm::can::HarmonyBmsModel::PacketId::Counters,
             suchm::can::HarmonyBmsModel::PacketId::ChargeTotals,
             suchm::can::HarmonyBmsModel::PacketId::DischargeTotals,
         }) {
        const auto result = model.decode(make_frame(packet_id, two_floats(1.0F, 2.0F)));
        ASSERT_TRUE(result);
        EXPECT_TRUE(result.value());
    }
}

TEST(HarmonyBmsModelTest, DecodesBalanceIcTemperatureFromHumidityPacket) {
    auto model = suchm::can::HarmonyBmsModel(BMS_ID);

    const auto result = model.decode(make_frame(
        suchm::can::HarmonyBmsModel::PacketId::Humidity,
        {std::byte{0x09}, std::byte{0xC4},  // 25.00 degC
         std::byte{0x13}, std::byte{0x88},  // 50.00 %RH
         std::byte{0x0B}, std::byte{0xB8},  // 30.00 degC
         std::byte{0}, std::byte{0}}));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_DOUBLE_EQ(result.value()->balance_ic_temperature.value, 30.0);
}

TEST(HarmonyBmsModelTest, ReassemblesCellsTemperaturesAndBalancing) {
    auto model = suchm::can::HarmonyBmsModel(BMS_ID);

    auto result = model.decode(make_frame(
        suchm::can::HarmonyBmsModel::PacketId::CellVoltage,
        {std::byte{0}, std::byte{4}, std::byte{0x0E}, std::byte{0x74}, std::byte{0x0E},
         std::byte{0xD8}, std::byte{0x0F}, std::byte{0x3C}}));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_EQ(result.value()->cell_count.value, 0);

    result = model.decode(make_frame(
        suchm::can::HarmonyBmsModel::PacketId::CellVoltage,
        {std::byte{3}, std::byte{4}, std::byte{0x0F}, std::byte{0xA0}}, 4));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_EQ(result.value()->cell_count.value, 4);
    EXPECT_DOUBLE_EQ(result.value()->cells[0].voltage, 3.7);
    EXPECT_DOUBLE_EQ(result.value()->cells[3].voltage, 4.0);

    result = model.decode(make_frame(
        suchm::can::HarmonyBmsModel::PacketId::Balancing,
        {std::byte{4}, std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0},
         std::byte{0}, std::byte{0x05}}));
    ASSERT_TRUE(result);
    EXPECT_TRUE(result.value()->cells[0].balancing);
    EXPECT_FALSE(result.value()->cells[1].balancing);
    EXPECT_TRUE(result.value()->cells[2].balancing);

    for (uint8_t offset = 0; offset < 9; offset += 3) {
        result = model.decode(make_frame(
            suchm::can::HarmonyBmsModel::PacketId::Temperatures,
            {std::byte{offset}, std::byte{10}, std::byte{0x09}, std::byte{0xC4}, std::byte{0x0A},
             std::byte{0x28}, std::byte{0x0A}, std::byte{0x8C}}));
        ASSERT_TRUE(result);
        ASSERT_TRUE(result.value());
        EXPECT_TRUE(std::isnan(result.value()->mosfet_temperature.value));
    }
    result = model.decode(make_frame(
        suchm::can::HarmonyBmsModel::PacketId::Temperatures,
        {std::byte{9}, std::byte{10}, std::byte{0x0D}, std::byte{0x7A}}, 4));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    // オフセット0, 3, 6のフレームは25.00, 26.00, 27.00 degCの順、最後のフレームは34.50 degC
    EXPECT_DOUBLE_EQ(result.value()->mosfet_temperature.value, 25.0);
    EXPECT_DOUBLE_EQ(result.value()->ambient_temperature.value, 26.0);
    EXPECT_DOUBLE_EQ(result.value()->additional_temperatures[0].value, 27.0);
    EXPECT_DOUBLE_EQ(result.value()->additional_temperatures[4].value, 34.5);
}

TEST(HarmonyBmsModelTest, ReassemblesStatusAndMapsHarmonyFaults) {
    auto model = suchm::can::HarmonyBmsModel(BMS_ID);
    const auto text = std::string("PSW_ON | FLT_PSW_OT");

    std::optional<suchm::can::HarmonyBmsModel::State> state;
    for (std::size_t chunk = 0; chunk < 5; ++chunk) {
        auto data = suchm::interface::CanFrame::Data{};
        const auto begin = chunk * 8;
        for (std::size_t i = 0; i < data.size() && begin + i < text.size(); ++i) {
            data[i] = std::byte{static_cast<uint8_t>(text[begin + i])};
        }
        const auto packet_id = static_cast<suchm::can::HarmonyBmsModel::PacketId>(
            static_cast<uint8_t>(suchm::can::HarmonyBmsModel::PacketId::Status1) + chunk);
        const auto result = model.decode(make_frame(packet_id, data));
        ASSERT_TRUE(result);
        ASSERT_TRUE(result.value());
        EXPECT_EQ(result.value()->status_updated, chunk == 4);
        state = result.value();
    }

    ASSERT_TRUE(state);
    EXPECT_EQ(
        state->status.power_switch_state, sinsei_umiusi_control::util::BmsPowerSwitchState::Fault);
    EXPECT_FALSE(state->status.faults.precharge);
    EXPECT_FALSE(state->status.faults.short_circuit);
    EXPECT_TRUE(state->status.faults.switch_over_temperature);
    EXPECT_FALSE(state->status.faults.charge_overcurrent);
    EXPECT_EQ(state->status_text, text);
}

TEST(HarmonyBmsModelTest, DoesNotUpdateStatusWhenAChunkIsMissing) {
    auto model = suchm::can::HarmonyBmsModel(BMS_ID);

    for (const auto packet_id : {
             suchm::can::HarmonyBmsModel::PacketId::Status1,
             suchm::can::HarmonyBmsModel::PacketId::Status2,
             suchm::can::HarmonyBmsModel::PacketId::Status4,
             suchm::can::HarmonyBmsModel::PacketId::Status5,
         }) {
        const auto result = model.decode(make_frame(packet_id, {}));
        ASSERT_TRUE(result);
        ASSERT_TRUE(result.value());
        EXPECT_FALSE(result.value()->status_updated);
    }
}

TEST(HarmonyBmsModelTest, AllowsDuplicateAndInterleavedFrames) {
    auto model = suchm::can::HarmonyBmsModel(BMS_ID);

    const auto decode_status = [&model](suchm::can::HarmonyBmsModel::PacketId packet_id) {
        return model.decode(make_frame(packet_id, {}));
    };

    for (const auto packet_id : {
             suchm::can::HarmonyBmsModel::PacketId::Status1,
             suchm::can::HarmonyBmsModel::PacketId::Status2,
             suchm::can::HarmonyBmsModel::PacketId::Status2,
             suchm::can::HarmonyBmsModel::PacketId::Status3,
         }) {
        const auto result = decode_status(packet_id);
        ASSERT_TRUE(result);
        ASSERT_TRUE(result.value());
        EXPECT_FALSE(result.value()->status_updated);
    }

    auto result = model.decode(
        make_frame(suchm::can::HarmonyBmsModel::PacketId::Voltage, two_floats(48.0F, 50.0F)));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_FALSE(result.value()->status_updated);

    result = decode_status(suchm::can::HarmonyBmsModel::PacketId::Status4);
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_FALSE(result.value()->status_updated);

    result = decode_status(suchm::can::HarmonyBmsModel::PacketId::Status5);
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_TRUE(result.value()->status_updated);

    result = decode_status(suchm::can::HarmonyBmsModel::PacketId::Status5);
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_FALSE(result.value()->status_updated);

    result = model.decode(
        make_frame(suchm::can::HarmonyBmsModel::PacketId::Voltage, two_floats(48.0F, 50.0F)));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_FALSE(result.value()->status_updated);
}

}  // namespace sinsei_umiusi_control::test::hardware_model::can::harmony_bms
