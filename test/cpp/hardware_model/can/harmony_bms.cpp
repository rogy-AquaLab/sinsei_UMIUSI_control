#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <variant>

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

// 全て同じ文字で埋めたステータス文字列のチャンク
auto make_status_frame(std::size_t chunk, char c) -> suchm::interface::CanFrame {
    auto data = suchm::interface::CanFrame::Data{};
    data.fill(std::byte{static_cast<uint8_t>(c)});
    return make_frame(
        static_cast<suchm::can::HarmonyBmsModel::PacketId>(
            static_cast<uint8_t>(suchm::can::HarmonyBmsModel::PacketId::Status1) + chunk),
        data);
}

using HarmonyBmsModel = suchm::can::HarmonyBmsModel;

}  // namespace

TEST(HarmonyBmsModelTest, HarmonyBmsModelExposesConfiguredIdTest) {
    const auto model = HarmonyBmsModel(BMS_ID);
    EXPECT_EQ(model.get_id(), BMS_ID);
}

TEST(HarmonyBmsModelTest, HarmonyBmsModelIgnoresFramesFromOtherNodesTest) {
    auto model = HarmonyBmsModel(BMS_ID);
    const auto result = model.decode(
        make_frame(HarmonyBmsModel::PacketId::Voltage, two_floats(48.0F, 50.0F), 8, 11));

    ASSERT_TRUE(result);
    EXPECT_FALSE(result.value().has_value());
}

TEST(HarmonyBmsModelTest, HarmonyBmsModelRejectsUnknownPacketFromConfiguredNodeTest) {
    auto model = HarmonyBmsModel(BMS_ID);
    const auto unknown_packet_id = static_cast<HarmonyBmsModel::PacketId>(0x7F);
    const auto result = model.decode(make_frame(unknown_packet_id, {}));

    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Received Harmony BMS frame with unknown packet ID: 127");
}

TEST(HarmonyBmsModelTest, HarmonyBmsModelRejectsInvalidLengthTest) {
    auto model = HarmonyBmsModel(BMS_ID);
    auto result = model.decode(make_frame(HarmonyBmsModel::PacketId::Voltage, {}, 4));

    ASSERT_FALSE(result);
    EXPECT_EQ(
        result.error(),
        "Received Harmony BMS packet 38 with invalid length (expected: 8, received: 4)");

    result = model.decode(make_frame(HarmonyBmsModel::PacketId::CellVoltage, {}, 3));
    ASSERT_FALSE(result);
    EXPECT_EQ(
        result.error(),
        "Received Harmony BMS packet 41 with invalid length (expected: 4, 6 or 8, received: 3)");

    result = model.decode(make_frame(HarmonyBmsModel::PacketId::Humidity, {}, 4));
    ASSERT_FALSE(result);
    EXPECT_EQ(
        result.error(),
        "Received Harmony BMS packet 44 with invalid length (expected: 6 or 8, received: 4)");

    result = model.decode(make_frame(HarmonyBmsModel::PacketId::Status1, {}, 0));
    ASSERT_FALSE(result);
    EXPECT_EQ(
        result.error(),
        "Received Harmony BMS packet 64 with invalid length (expected: 1 to 8, received: 0)");
}

TEST(HarmonyBmsModelTest, HarmonyBmsModelDecodesVoltageAndCurrentTest) {
    auto model = HarmonyBmsModel(BMS_ID);

    auto result =
        model.decode(make_frame(HarmonyBmsModel::PacketId::Voltage, two_floats(48.0F, 50.0F)));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    const auto * voltage = std::get_if<HarmonyBmsModel::PacketVoltage>(&result.value().value());
    ASSERT_NE(voltage, nullptr);
    EXPECT_DOUBLE_EQ(voltage->pack, 48.0);
    EXPECT_DOUBLE_EQ(voltage->charger, 50.0);

    result =
        model.decode(make_frame(HarmonyBmsModel::PacketId::Current, two_floats(12.5F, -12.25F)));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    const auto * current = std::get_if<HarmonyBmsModel::PacketCurrent>(&result.value().value());
    ASSERT_NE(current, nullptr);
    EXPECT_DOUBLE_EQ(current->input, 12.5);
    EXPECT_DOUBLE_EQ(current->measured, -12.25);
}

TEST(HarmonyBmsModelTest, HarmonyBmsModelDecodesCountersAndTotalsTest) {
    auto model = HarmonyBmsModel(BMS_ID);

    auto result =
        model.decode(make_frame(HarmonyBmsModel::PacketId::Counters, two_floats(2.5F, 120.0F)));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    const auto * counters = std::get_if<HarmonyBmsModel::PacketCounters>(&result.value().value());
    ASSERT_NE(counters, nullptr);
    EXPECT_DOUBLE_EQ(counters->amp_hour, 2.5);
    EXPECT_DOUBLE_EQ(counters->watt_hour, 120.0);

    result =
        model.decode(make_frame(HarmonyBmsModel::PacketId::ChargeTotals, two_floats(9.0F, 420.0F)));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    const auto * charge_totals =
        std::get_if<HarmonyBmsModel::PacketChargeTotals>(&result.value().value());
    ASSERT_NE(charge_totals, nullptr);
    EXPECT_DOUBLE_EQ(charge_totals->amp_hour, 9.0);
    EXPECT_DOUBLE_EQ(charge_totals->watt_hour, 420.0);

    result = model.decode(
        make_frame(HarmonyBmsModel::PacketId::DischargeTotals, two_floats(11.0F, 510.0F)));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    const auto * discharge_totals =
        std::get_if<HarmonyBmsModel::PacketDischargeTotals>(&result.value().value());
    ASSERT_NE(discharge_totals, nullptr);
    EXPECT_DOUBLE_EQ(discharge_totals->amp_hour, 11.0);
    EXPECT_DOUBLE_EQ(discharge_totals->watt_hour, 510.0);
}

TEST(HarmonyBmsModelTest, HarmonyBmsModelDecodesCellVoltageTest) {
    auto model = HarmonyBmsModel(BMS_ID);

    auto result = model.decode(make_frame(
        HarmonyBmsModel::PacketId::CellVoltage,
        {std::byte{3}, std::byte{12}, std::byte{0x0E}, std::byte{0x74},  // 3.700 V
         std::byte{0x0F}, std::byte{0xA0},                               // 4.000 V
         std::byte{0x10}, std::byte{0x04}}));                            // 4.100 V
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    const auto * cell = std::get_if<HarmonyBmsModel::PacketCellVoltage>(&result.value().value());
    ASSERT_NE(cell, nullptr);
    EXPECT_EQ(cell->offset, 3);
    EXPECT_EQ(cell->cell_count, 12);
    EXPECT_EQ(cell->value_count, 3);
    EXPECT_DOUBLE_EQ(cell->voltages[0], 3.7);
    EXPECT_DOUBLE_EQ(cell->voltages[1], 4.0);
    EXPECT_DOUBLE_EQ(cell->voltages[2], 4.1);

    // 最後のフレームは値が1つだけの場合がある
    result = model.decode(make_frame(
        HarmonyBmsModel::PacketId::CellVoltage,
        {std::byte{9}, std::byte{10}, std::byte{0x0E}, std::byte{0x74}}, 4));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    cell = std::get_if<HarmonyBmsModel::PacketCellVoltage>(&result.value().value());
    ASSERT_NE(cell, nullptr);
    EXPECT_EQ(cell->offset, 9);
    EXPECT_EQ(cell->value_count, 1);
    EXPECT_DOUBLE_EQ(cell->voltages[0], 3.7);
}

TEST(HarmonyBmsModelTest, HarmonyBmsModelDecodesBalancingTest) {
    auto model = HarmonyBmsModel(BMS_ID);

    const auto result = model.decode(make_frame(
        HarmonyBmsModel::PacketId::Balancing,
        {std::byte{4}, std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0},
         std::byte{0}, std::byte{0x15}}));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    const auto * balancing = std::get_if<HarmonyBmsModel::PacketBalancing>(&result.value().value());
    ASSERT_NE(balancing, nullptr);
    EXPECT_EQ(balancing->cell_count, 4);
    EXPECT_TRUE(balancing->balancing[0]);
    EXPECT_FALSE(balancing->balancing[1]);
    EXPECT_TRUE(balancing->balancing[2]);
    EXPECT_FALSE(balancing->balancing[3]);
    // BMSが報告したセル数 (4) を超えるビットは無視する
    EXPECT_FALSE(balancing->balancing[4]);
}

TEST(HarmonyBmsModelTest, HarmonyBmsModelDecodesTemperaturesTest) {
    auto model = HarmonyBmsModel(BMS_ID);

    const auto result = model.decode(make_frame(
        HarmonyBmsModel::PacketId::Temperatures,
        {std::byte{3}, std::byte{10}, std::byte{0x09}, std::byte{0xC4},  // 25.00 degC
         std::byte{0x0A}, std::byte{0x28},                               // 26.00 degC
         std::byte{0x0D}, std::byte{0x7A}}));                            // 34.50 degC
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    const auto * temperatures =
        std::get_if<HarmonyBmsModel::PacketTemperatures>(&result.value().value());
    ASSERT_NE(temperatures, nullptr);
    EXPECT_EQ(temperatures->offset, 3);
    EXPECT_EQ(temperatures->temperature_count, 10);
    EXPECT_EQ(temperatures->value_count, 3);
    EXPECT_DOUBLE_EQ(temperatures->temperatures[0], 25.0);
    EXPECT_DOUBLE_EQ(temperatures->temperatures[1], 26.0);
    EXPECT_DOUBLE_EQ(temperatures->temperatures[2], 34.5);
}

TEST(HarmonyBmsModelTest, HarmonyBmsModelDecodesHumidityTest) {
    auto model = HarmonyBmsModel(BMS_ID);

    auto result = model.decode(make_frame(
        HarmonyBmsModel::PacketId::Humidity, {std::byte{0x09}, std::byte{0xC4},     // 25.00 degC
                                              std::byte{0x13}, std::byte{0x88},     // 50.00 %RH
                                              std::byte{0x0B}, std::byte{0xB8},     // 30.00 degC
                                              std::byte{0x27}, std::byte{0x8D}}));  // 101250 Pa
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    const auto * humidity = std::get_if<HarmonyBmsModel::PacketHumidity>(&result.value().value());
    ASSERT_NE(humidity, nullptr);
    EXPECT_DOUBLE_EQ(humidity->temperature, 25.0);
    EXPECT_DOUBLE_EQ(humidity->humidity, 50.0);
    EXPECT_DOUBLE_EQ(humidity->balance_ic_temperature, 30.0);
    ASSERT_TRUE(humidity->pressure);
    EXPECT_NEAR(humidity->pressure.value(), 101250.0, 1e-6);

    // 6バイトのフレームは気圧を含まない
    result = model.decode(make_frame(
        HarmonyBmsModel::PacketId::Humidity,
        {std::byte{0x09}, std::byte{0xC4}, std::byte{0x13}, std::byte{0x88}, std::byte{0x0B},
         std::byte{0xB8}},
        6));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    humidity = std::get_if<HarmonyBmsModel::PacketHumidity>(&result.value().value());
    ASSERT_NE(humidity, nullptr);
    EXPECT_FALSE(humidity->pressure);
}

TEST(HarmonyBmsModelTest, HarmonyBmsModelDecodesSummaryTest) {
    auto model = HarmonyBmsModel(BMS_ID);

    const auto result = model.decode(make_frame(
        HarmonyBmsModel::PacketId::Summary,
        {std::byte{0x0E}, std::byte{0x74},  // 3.700 V
         std::byte{0x10}, std::byte{0x04},  // 4.100 V
         std::byte{0x80}, std::byte{0xFF}, std::byte{42}, std::byte{0x17}}));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    const auto * summary = std::get_if<HarmonyBmsModel::PacketSummary>(&result.value().value());
    ASSERT_NE(summary, nullptr);
    EXPECT_DOUBLE_EQ(summary->cell_voltage_min, 3.7);
    EXPECT_DOUBLE_EQ(summary->cell_voltage_max, 4.1);
    EXPECT_NEAR(summary->state_of_charge, 128.0 / 255.0, 1e-12);
    EXPECT_DOUBLE_EQ(summary->state_of_health, 1.0);
    EXPECT_TRUE(summary->charging);
    EXPECT_TRUE(summary->balancing);
    EXPECT_TRUE(summary->charge_allowed);
    EXPECT_DOUBLE_EQ(summary->cell_temperature_max, 42.0);
    EXPECT_EQ(summary->data_version, 1);
}

TEST(HarmonyBmsModelTest, HarmonyBmsModelAssemblesStatusAndMapsHarmonyFaultsTest) {
    auto model = HarmonyBmsModel(BMS_ID);
    const auto text = std::string("PSW_ON | FLT_PSW_OT");

    for (std::size_t chunk = 0; chunk < 5; ++chunk) {
        auto data = suchm::interface::CanFrame::Data{};
        const auto begin = chunk * 8;
        for (std::size_t i = 0; i < data.size() && begin + i < text.size(); ++i) {
            data[i] = std::byte{static_cast<uint8_t>(text[begin + i])};
        }
        const auto packet_id = static_cast<HarmonyBmsModel::PacketId>(
            static_cast<uint8_t>(HarmonyBmsModel::PacketId::Status1) + chunk);
        const auto result = model.decode(make_frame(packet_id, data));
        ASSERT_TRUE(result);
        ASSERT_TRUE(result.value());

        const auto * status = std::get_if<HarmonyBmsModel::PacketStatus>(&result.value().value());
        if (chunk < 4) {
            EXPECT_EQ(status, nullptr);
            continue;
        }
        ASSERT_NE(status, nullptr);
        EXPECT_EQ(
            status->power_switch_state, sinsei_umiusi_control::util::BmsPowerSwitchState::Fault);
        EXPECT_FALSE(status->faults.precharge);
        EXPECT_FALSE(status->faults.short_circuit);
        EXPECT_TRUE(status->faults.switch_over_temperature);
        EXPECT_FALSE(status->faults.charge_overcurrent);
        EXPECT_EQ(status->text, text);
    }
}

TEST(HarmonyBmsModelTest, HarmonyBmsModelDoesNotReturnStatusWhenAChunkIsMissingTest) {
    auto model = HarmonyBmsModel(BMS_ID);

    for (const auto chunk : {0, 1, 3, 4}) {
        const auto result = model.decode(make_status_frame(chunk, 'A'));
        ASSERT_TRUE(result);
        ASSERT_TRUE(result.value());
        EXPECT_TRUE(std::holds_alternative<std::monostate>(result.value().value()));
    }
}

TEST(HarmonyBmsModelTest, HarmonyBmsModelDoesNotMixStatusChunksAcrossCyclesTest) {
    auto model = HarmonyBmsModel(BMS_ID);

    // 1周期目: Status5を取りこぼす
    for (std::size_t chunk = 0; chunk < 4; ++chunk) {
        const auto result = model.decode(make_status_frame(chunk, 'A'));
        ASSERT_TRUE(result);
        EXPECT_TRUE(std::holds_alternative<std::monostate>(result.value().value()));
    }
    // 2周期目: Status1を取りこぼす
    for (std::size_t chunk = 1; chunk < 5; ++chunk) {
        const auto result = model.decode(make_status_frame(chunk, 'B'));
        ASSERT_TRUE(result);
        EXPECT_TRUE(std::holds_alternative<std::monostate>(result.value().value()));
    }
    // 3周期目: 全て受信
    for (std::size_t chunk = 0; chunk < 5; ++chunk) {
        const auto result = model.decode(make_status_frame(chunk, 'C'));
        ASSERT_TRUE(result);
        if (chunk < 4) {
            continue;
        }
        const auto * status = std::get_if<HarmonyBmsModel::PacketStatus>(&result.value().value());
        ASSERT_NE(status, nullptr);
        EXPECT_EQ(status->text, std::string(40, 'C'));
    }
}

TEST(HarmonyBmsModelTest, HarmonyBmsModelAssemblesStatusWithInterleavedFramesTest) {
    auto model = HarmonyBmsModel(BMS_ID);

    for (std::size_t chunk = 0; chunk < 3; ++chunk) {
        ASSERT_TRUE(model.decode(make_status_frame(chunk, 'A')));
    }
    // ステータス文字列以外のフレームが間に入っても組み立てを続ける
    ASSERT_TRUE(
        model.decode(make_frame(HarmonyBmsModel::PacketId::Voltage, two_floats(48.0F, 50.0F))));
    ASSERT_TRUE(model.decode(make_status_frame(3, 'A')));
    const auto result = model.decode(make_status_frame(4, 'A'));

    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    EXPECT_TRUE(std::holds_alternative<HarmonyBmsModel::PacketStatus>(result.value().value()));
}

}  // namespace sinsei_umiusi_control::test::hardware_model::can::harmony_bms
