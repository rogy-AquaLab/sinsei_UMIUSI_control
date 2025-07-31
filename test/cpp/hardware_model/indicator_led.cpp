#include "sinsei_umiusi_control/cmd/indicator_led.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rcpputils/tl_expected/expected.hpp>

#include "gmock/gmock.h"
#include "mock/gpio.hpp"
#include "sinsei_umiusi_control/hardware_model/indicator_led_model.hpp"

namespace succmd = sinsei_umiusi_control::cmd;
namespace suchm = sinsei_umiusi_control::hardware_model;

using testing::Return;

namespace sinsei_umiusi_control::test::hardware_model::indicator_led {

namespace {

constexpr auto _ = testing::_;

}

constexpr uint8_t LED_PIN = 24;

class IndicatorLedModelOnWriteTest : public ::testing::TestWithParam<cmd::indicator_led::Enabled> {
};
INSTANTIATE_TEST_SUITE_P(
    IndicatorLedModelTest, IndicatorLedModelOnWriteTest,
    ::testing::Values(succmd::indicator_led::Enabled{true}, succmd::indicator_led::Enabled{false}));

TEST(IndicatorLedModelOnInitTest, all) {
    auto gpio = std::make_unique<mock::Gpio>();
    EXPECT_CALL(*gpio, set_mode_output(_))
        .Times(1)
        .WillOnce(Return(tl::expected<void, suchm::interface::GpioError>()));

    auto indicator_led_model = suchm::IndicatorLedModel(std::move(gpio), LED_PIN);
    auto result = indicator_led_model.on_init();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

TEST_P(IndicatorLedModelOnWriteTest, all) {
    auto enabled = GetParam();

    auto gpio = std::make_unique<mock::Gpio>();
    EXPECT_CALL(*gpio, write_digital(_, std::move(enabled.value))).Times(1);

    auto indicator_led_model = suchm::IndicatorLedModel(std::move(gpio), LED_PIN);
    indicator_led_model.on_init();
    indicator_led_model.on_write(std::move(enabled));
}

}  // namespace sinsei_umiusi_control::test::hardware_model::indicator_led
