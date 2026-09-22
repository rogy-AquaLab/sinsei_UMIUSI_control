#include <gtest/gtest.h>

#include <algorithm>
#include <array>
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

constexpr uint8_t BMS_ID = 10;

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

TEST(HarmonyBmsModelTest, IgnoresFramesFromOtherNodes) {
    auto model = suchm::can::HarmonyBmsModel(BMS_ID);
    const auto result = model.decode(make_frame(
        suchm::can::HarmonyBmsModel::PacketId::Voltage, two_floats(48.0F, 50.0F), 8, 11));

    ASSERT_TRUE(result);
    EXPECT_FALSE(result.value().has_value());
}

TEST(HarmonyBmsModelTest, DecodesVoltageCurrentAndSummary) {
    auto model = suchm::can::HarmonyBmsModel(BMS_ID);

    auto result = model.decode(
        make_frame(suchm::can::HarmonyBmsModel::PacketId::Voltage, two_floats(48.0F, 50.0F)));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_DOUBLE_EQ(result.value()->pack_voltage, 48.0);
    EXPECT_DOUBLE_EQ(result.value()->charger_voltage, 50.0);

    result = model.decode(
        make_frame(suchm::can::HarmonyBmsModel::PacketId::Current, two_floats(12.5F, -12.25F)));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_DOUBLE_EQ(result.value()->input_current, 12.5);
    EXPECT_DOUBLE_EQ(result.value()->measured_current, -12.25);

    result = model.decode(make_frame(
        suchm::can::HarmonyBmsModel::PacketId::Summary,
        {std::byte{0x0E}, std::byte{0x74},  // 3.700 V
         std::byte{0x10}, std::byte{0x04},  // 4.100 V
         std::byte{0x80}, std::byte{0xFF}, std::byte{42}, std::byte{0x17}}));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_DOUBLE_EQ(result.value()->cell_voltage_min, 3.7);
    EXPECT_DOUBLE_EQ(result.value()->cell_voltage_max, 4.1);
    EXPECT_NEAR(result.value()->state_of_charge, 128.0 / 255.0, 1e-12);
    EXPECT_DOUBLE_EQ(result.value()->state_of_health, 1.0);
    EXPECT_DOUBLE_EQ(result.value()->cell_temperature_max, 42.0);
    EXPECT_TRUE(result.value()->charging);
    EXPECT_TRUE(result.value()->balancing);
    EXPECT_TRUE(result.value()->charge_allowed);
    EXPECT_EQ(result.value()->data_version, 1);
}

TEST(HarmonyBmsModelTest, DecodesCountersTotalsAndHumidity) {
    auto model = suchm::can::HarmonyBmsModel(BMS_ID);

    auto result = model.decode(
        make_frame(suchm::can::HarmonyBmsModel::PacketId::Counters, two_floats(2.5F, 120.0F)));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_DOUBLE_EQ(result.value()->net_consumed_charge, 2.5);
    EXPECT_DOUBLE_EQ(result.value()->net_consumed_energy, 120.0);

    result = model.decode(
        make_frame(suchm::can::HarmonyBmsModel::PacketId::ChargeTotals, two_floats(9.0F, 420.0F)));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_DOUBLE_EQ(result.value()->total_charged_charge, 9.0);
    EXPECT_DOUBLE_EQ(result.value()->total_charged_energy, 420.0);

    result = model.decode(make_frame(
        suchm::can::HarmonyBmsModel::PacketId::DischargeTotals, two_floats(11.0F, 510.0F)));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_DOUBLE_EQ(result.value()->total_discharged_charge, 11.0);
    EXPECT_DOUBLE_EQ(result.value()->total_discharged_energy, 510.0);

    result = model.decode(make_frame(
        suchm::can::HarmonyBmsModel::PacketId::Humidity,
        {std::byte{0x09}, std::byte{0xC4},  // 25.00 degC
         std::byte{0x13}, std::byte{0x88},  // 50.00 %RH
         std::byte{0x0B}, std::byte{0xB8},  // 30.00 degC
         std::byte{0}, std::byte{0}}));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_DOUBLE_EQ(result.value()->humidity_sensor_temperature, 25.0);
    EXPECT_DOUBLE_EQ(result.value()->relative_humidity, 50.0);
    EXPECT_DOUBLE_EQ(result.value()->balance_ic_temperature, 30.0);
}

TEST(HarmonyBmsModelTest, ReassemblesCellsTemperaturesAndBalancing) {
    auto model = suchm::can::HarmonyBmsModel(BMS_ID);

    auto result = model.decode(make_frame(
        suchm::can::HarmonyBmsModel::PacketId::CellVoltage,
        {std::byte{0}, std::byte{4}, std::byte{0x0E}, std::byte{0x74}, std::byte{0x0E},
         std::byte{0xD8}, std::byte{0x0F}, std::byte{0x3C}}));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_EQ(result.value()->cell_count, 0);

    result = model.decode(make_frame(
        suchm::can::HarmonyBmsModel::PacketId::CellVoltage,
        {std::byte{3}, std::byte{4}, std::byte{0x0F}, std::byte{0xA0}}, 4));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_EQ(result.value()->cell_count, 4);
    EXPECT_DOUBLE_EQ(result.value()->cell_voltages[0], 3.7);
    EXPECT_DOUBLE_EQ(result.value()->cell_voltages[3], 4.0);

    result = model.decode(make_frame(
        suchm::can::HarmonyBmsModel::PacketId::Balancing,
        {std::byte{4}, std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0},
         std::byte{0}, std::byte{0x05}}));
    ASSERT_TRUE(result);
    EXPECT_TRUE(result.value()->cell_balancing[0]);
    EXPECT_FALSE(result.value()->cell_balancing[1]);
    EXPECT_TRUE(result.value()->cell_balancing[2]);

    result = model.decode(make_frame(
        suchm::can::HarmonyBmsModel::PacketId::Temperatures,
        {std::byte{0}, std::byte{3}, std::byte{0x09}, std::byte{0xC4}, std::byte{0x0A},
         std::byte{0x28}, std::byte{0x0A}, std::byte{0x8C}}));
    ASSERT_TRUE(result);
    EXPECT_EQ(result.value()->temperature_count, 3);
    EXPECT_DOUBLE_EQ(result.value()->temperatures[0], 25.0);
    EXPECT_DOUBLE_EQ(result.value()->temperatures[2], 27.0);
}

TEST(HarmonyBmsModelTest, ReassemblesStatusAndMapsHarmonyFaults) {
    auto model = suchm::can::HarmonyBmsModel(BMS_ID);
    const auto text = std::string("PSW_ON | FLT_PSW_OT");

    std::optional<suchm::can::HarmonyBmsModel::State> state;
    for (std::size_t chunk = 0; chunk < 3; ++chunk) {
        auto data = suchm::interface::CanFrame::Data{};
        const auto begin = chunk * 8;
        const auto length = std::min<std::size_t>(8, text.size() + 1 - begin);
        for (std::size_t i = 0; i < length && begin + i < text.size(); ++i) {
            data[i] = std::byte{static_cast<uint8_t>(text[begin + i])};
        }
        const auto packet_id = static_cast<suchm::can::HarmonyBmsModel::PacketId>(
            static_cast<uint8_t>(suchm::can::HarmonyBmsModel::PacketId::Status1) + chunk);
        const auto result = model.decode(make_frame(packet_id, data, static_cast<uint8_t>(length)));
        ASSERT_TRUE(result);
        ASSERT_TRUE(result.value());
        state = result.value();
    }

    ASSERT_TRUE(state);
    EXPECT_EQ(state->power_switch_state, suchm::can::HarmonyBmsModel::PowerSwitchState::Fault);
    EXPECT_EQ(
        state->fault_flags,
        static_cast<uint32_t>(suchm::can::HarmonyBmsModel::FaultSwitchOverTemperature));
    EXPECT_EQ(std::string(state->status.data()), text);
}

}  // namespace sinsei_umiusi_control::test::hardware_model::can::harmony_bms
