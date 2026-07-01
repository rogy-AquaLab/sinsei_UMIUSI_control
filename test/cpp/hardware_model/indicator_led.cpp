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
using testing::ByMove;
using testing::Field;

namespace sinsei_umiusi_control::test::hardware_model::indicator_led {

constexpr uint8_t LED_LINE_OFFSET = 24;

class IndicatorLedModelOnWriteTest : public ::testing::TestWithParam<cmd::indicator_led::Enabled> {
};
INSTANTIATE_TEST_SUITE_P(
    IndicatorLedModelTest, IndicatorLedModelOnWriteTest,
    ::testing::Values(succmd::indicator_led::Enabled{true}, succmd::indicator_led::Enabled{false}));

TEST(IndicatorLedModelOnInitTest, all) {
    auto gpio = std::make_unique<mock::GpioChip>();
    auto gpio_lines = std::make_unique<mock::GpioLineRequest>();
    EXPECT_CALL(
        *gpio,
        request_outputs(testing::AllOf(
            Field(
                &suchm::interface::GpioOutputRequest::offsets,
                testing::ElementsAre(LED_LINE_OFFSET)),
            Field(
                &suchm::interface::GpioOutputRequest::initial_values,
                testing::ElementsAre(suchm::interface::GpioValue::Inactive)),
            Field(
                &suchm::interface::GpioOutputRequest::consumer,
                testing::StrEq("sinsei_umiusi_control::IndicatorLedModel")))))
        .Times(1)
        .WillOnce(Return(ByMove(tl::expected<std::unique_ptr<suchm::interface::GpioLineRequest>, std::string>(
            std::move(gpio_lines)))));

    auto indicator_led_model = suchm::IndicatorLedModel(std::move(gpio), LED_LINE_OFFSET);
    auto result = indicator_led_model.on_init();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

TEST_P(IndicatorLedModelOnWriteTest, all) {
    auto enabled = GetParam();

    auto gpio = std::make_unique<mock::GpioChip>();
    auto gpio_lines = std::make_unique<mock::GpioLineRequest>();
    EXPECT_CALL(
        *gpio_lines, set_values(testing::ElementsAre(suchm::interface::to_gpio_value(enabled.value))))
        .Times(1)
        .WillOnce(Return(tl::expected<void, std::string>()));
    EXPECT_CALL(
        *gpio,
        request_outputs(testing::AllOf(
            Field(
                &suchm::interface::GpioOutputRequest::offsets,
                testing::ElementsAre(LED_LINE_OFFSET)),
            Field(
                &suchm::interface::GpioOutputRequest::initial_values,
                testing::ElementsAre(suchm::interface::GpioValue::Inactive)),
            Field(
                &suchm::interface::GpioOutputRequest::consumer,
                testing::StrEq("sinsei_umiusi_control::IndicatorLedModel")))))
        .Times(1)
        .WillOnce(Return(ByMove(tl::expected<std::unique_ptr<suchm::interface::GpioLineRequest>, std::string>(
            std::move(gpio_lines)))));

    auto indicator_led_model = suchm::IndicatorLedModel(std::move(gpio), LED_LINE_OFFSET);
    indicator_led_model.on_init();
    indicator_led_model.on_write(std::move(enabled));
}

}  // namespace sinsei_umiusi_control::test::hardware_model::indicator_led
