#ifndef SINSEI_UMIUSI_CONTROL_test_TEST_HARDWARE_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_test_TEST_HARDWARE_MODEL_HPP

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <rcpputils/tl_expected/expected.hpp>

#include "sinsei_umiusi_control/cmd/headlights.hpp"
#include "sinsei_umiusi_control/cmd/indicator_led.hpp"
#include "sinsei_umiusi_control/hardware_model/headlights_model.hpp"
#include "sinsei_umiusi_control/hardware_model/imu_model.hpp"
#include "sinsei_umiusi_control/hardware_model/indicator_led_model.hpp"
#include "sinsei_umiusi_control/util/gpio_interface.hpp"

namespace suchm = sinsei_umiusi_control::hardware_model;
namespace succmd = sinsei_umiusi_control::cmd;
namespace sucst = sinsei_umiusi_control::state;
namespace sucutil = sinsei_umiusi_control::util;

using testing::AtLeast;
using testing::Return;

namespace sinsei_umiusi_control::test::hardware_model {

constexpr auto _ = ::testing::_;

namespace mock {

class Gpio : public sinsei_umiusi_control::util::GpioInterface {
  public:
    MOCK_METHOD1(
        set_mode_output, tl::expected<void, sucutil::GpioError>(std::vector<uint8_t> pins));
    MOCK_METHOD1(set_mode_input, tl::expected<void, sucutil::GpioError>(std::vector<uint8_t> pins));
    MOCK_METHOD2(write_digital, tl::expected<void, sucutil::GpioError>(uint8_t pin, bool enabled));
    MOCK_METHOD0(write_pwm, tl::expected<void, sucutil::GpioError>());  // TODO: 未実装
    MOCK_METHOD1(i2c_open, tl::expected<void, sucutil::GpioError>(int address));
    MOCK_METHOD0(i2c_close, tl::expected<void, sucutil::GpioError>());
    MOCK_METHOD1(i2c_write_byte, tl::expected<void, sucutil::GpioError>(uint8_t value));
    MOCK_METHOD0(i2c_read_byte, tl::expected<uint8_t, sucutil::GpioError>());
    MOCK_METHOD2(
        i2c_write_byte_data, tl::expected<void, sucutil::GpioError>(uint8_t reg, uint8_t value));
    MOCK_METHOD1(i2c_read_byte_data, tl::expected<uint8_t, sucutil::GpioError>(uint8_t reg));
};

}  // namespace mock

namespace headlights {

constexpr uint8_t HIGH_BEAM_PIN = 5;
constexpr uint8_t LOW_BEAM_PIN = 6;
constexpr uint8_t IR_PIN = 25;

class HeadlightsModelOnWriteTest
: public ::testing::TestWithParam<std::tuple<
      succmd::headlights::HighBeamEnabled, succmd::headlights::LowBeamEnabled,
      succmd::headlights::IrEnabled>> {};
INSTANTIATE_TEST_CASE_P(
    HeadlightsModelTest, HeadlightsModelOnWriteTest,
    ::testing::Combine(
        ::testing::Values(
            succmd::headlights::HighBeamEnabled{true}, succmd::headlights::HighBeamEnabled{false}),
        ::testing::Values(
            succmd::headlights::LowBeamEnabled{true}, succmd::headlights::LowBeamEnabled{false}),
        ::testing::Values(
            succmd::headlights::IrEnabled{true}, succmd::headlights::IrEnabled{false})));

TEST(HeadlightsModelOnInitTest, all) {
    auto gpio = std::make_unique<mock::Gpio>();

    EXPECT_CALL(*gpio, set_mode_output(_))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));

    auto headlights_model =
        suchm::HeadlightsModel(std::move(gpio), HIGH_BEAM_PIN, LOW_BEAM_PIN, IR_PIN);
    auto result = headlights_model.on_init();
    ASSERT_TRUE(result.has_value()) << std::string("Error: ") + result.error();
}

TEST_P(HeadlightsModelOnWriteTest, all) {
    auto [high_beam_enabled, low_beam_enabled, ir_enabled] = GetParam();

    auto gpio = std::make_unique<mock::Gpio>();

    int n_true = static_cast<int>(high_beam_enabled.value) +
                 static_cast<int>(low_beam_enabled.value) + static_cast<int>(ir_enabled.value);

    EXPECT_CALL(*gpio, write_digital(_, true)).Times(n_true);
    EXPECT_CALL(*gpio, write_digital(_, false)).Times(3 - n_true);

    auto headlights_model =
        suchm::HeadlightsModel(std::move(gpio), HIGH_BEAM_PIN, LOW_BEAM_PIN, IR_PIN);
    headlights_model.on_init();
    headlights_model.on_write(high_beam_enabled, low_beam_enabled, ir_enabled);
}

}  // namespace headlights

namespace imu {

// copied from sinsei_umiusi_control/hardware_model/imu_model.hpp
static constexpr uint8_t ADDRESS = 0x28;
static constexpr uint8_t ID = 0xA0;
static constexpr uint8_t CHIP_ID_ADDR = 0x00;
static constexpr uint8_t OPR_MODE_ADDR = 0x3D;
static constexpr uint8_t SYS_TRIGGER_ADDR = 0x3F;
static constexpr uint8_t PWR_MODE_ADDR = 0x3E;
static constexpr uint8_t PAGE_ID_ADDR = 0x07;
static constexpr uint8_t EULER_H_LSB_ADDR = 0x1A;
static constexpr uint8_t OPERATION_MODE_CONFIG = 0x00;
static constexpr uint8_t OPERATION_MODE_NDOF = 0X0C;
static constexpr uint8_t POWER_MODE_NORMAL = 0x00;
static constexpr uint8_t TEMP_ADDR = 0X34;

TEST(ImuModelBeginTest, all) {
    auto gpio = std::make_unique<mock::Gpio>();
    // TODO: 順序付けする
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(tl::expected<uint8_t, sucutil::GpioError>(ID)));
    EXPECT_CALL(*gpio, i2c_write_byte_data(OPR_MODE_ADDR, OPERATION_MODE_CONFIG))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, 0x20))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(PWR_MODE_ADDR, POWER_MODE_NORMAL))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(PAGE_ID_ADDR, 0x0))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, 0x0))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(OPR_MODE_ADDR, OPERATION_MODE_NDOF))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));

    auto imu_model = suchm::ImuModel(std::move(gpio));
    auto result = imu_model.begin();
    ASSERT_TRUE(result.has_value()) << std::string("Error: ") + result.error();
}

TEST(ImuModelOnReadTest, all) {
    auto gpio = std::make_unique<mock::Gpio>();
    for (int i = 0; i < 6; ++i) {
        // TODO: test with dummy data
        EXPECT_CALL(*gpio, i2c_read_byte_data(EULER_H_LSB_ADDR + i))
            .Times(1)
            .WillOnce(Return(tl::expected<uint8_t, sucutil::GpioError>(0)));
    }
    // TODO: test with dummy data
    EXPECT_CALL(*gpio, i2c_read_byte_data(TEMP_ADDR))
        .Times(1)
        .WillOnce(Return(tl::expected<uint8_t, sucutil::GpioError>(0)));

    auto imu_model = suchm::ImuModel(std::move(gpio));
    auto result = imu_model.on_read();
    ASSERT_TRUE(result.has_value()) << std::string("Error: ") + result.error();
}

}  // namespace imu

namespace indicator_led {

constexpr uint8_t LED_PIN = 24;

class IndicatorLedModelOnWriteTest : public ::testing::TestWithParam<cmd::indicator_led::Enabled> {
};
INSTANTIATE_TEST_CASE_P(
    IndicatorLedModelTest, IndicatorLedModelOnWriteTest,
    ::testing::Values(succmd::indicator_led::Enabled{true}, succmd::indicator_led::Enabled{false}));

TEST(IndicatorLedModelOnInitTest, all) {
    auto gpio = std::make_unique<mock::Gpio>();
    EXPECT_CALL(*gpio, set_mode_output(_))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));

    auto indicator_led_model = suchm::IndicatorLedModel(std::move(gpio), LED_PIN);
    auto result = indicator_led_model.on_init();
    ASSERT_TRUE(result.has_value()) << std::string("Error: ") + result.error();
}

TEST_P(IndicatorLedModelOnWriteTest, all) {
    auto enabled = GetParam();

    auto gpio = std::make_unique<mock::Gpio>();
    EXPECT_CALL(*gpio, write_digital(_, enabled.value)).Times(1);

    auto indicator_led_model = suchm::IndicatorLedModel(std::move(gpio), LED_PIN);
    indicator_led_model.on_init();
    indicator_led_model.on_write(enabled);
}

}  // namespace indicator_led

}  // namespace sinsei_umiusi_control::test::hardware_model

#endif  // SINSEI_UMIUSI_CONTROL_test_TEST_HARDWARE_MODEL_HPP
