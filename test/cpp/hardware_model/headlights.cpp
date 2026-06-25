#include "sinsei_umiusi_control/cmd/headlights.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rcpputils/tl_expected/expected.hpp>

#include "mock/gpio.hpp"
#include "sinsei_umiusi_control/hardware_model/headlights_model.hpp"
#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"

namespace succmd = sinsei_umiusi_control::cmd;
namespace suchm = sinsei_umiusi_control::hardware_model;

using sinsei_umiusi_control::test::mock::Gpio;
using sinsei_umiusi_control::test::mock::GpioLineRequest;
using testing::ByMove;
using testing::Field;
using testing::Return;

namespace sinsei_umiusi_control::test::hardware_model::headlights {

constexpr uint8_t HIGH_BEAM_LINE_OFFSET = 5;
constexpr uint8_t LOW_BEAM_LINE_OFFSET = 6;
constexpr uint8_t IR_LINE_OFFSET = 25;

class HeadlightsModelOnWriteTest
: public ::testing::TestWithParam<std::tuple<bool, bool, bool, int>> {};
INSTANTIATE_TEST_SUITE_P(
    HeadlightsModelTest, HeadlightsModelOnWriteTest,
    ::testing::Combine(
        ::testing::Bool(), ::testing::Bool(), ::testing::Bool(), ::testing::Values(0, 1, 2, 3)));

TEST(HeadlightsModelOnInitTest, all) {
    auto gpio = std::make_unique<mock::Gpio>();
    auto gpio_lines = std::make_unique<mock::GpioLineRequest>();

    EXPECT_CALL(
        *gpio,
        request_outputs(testing::AllOf(
            Field(
                &suchm::interface::GpioOutputRequest::offsets,
                testing::ElementsAre(HIGH_BEAM_LINE_OFFSET, LOW_BEAM_LINE_OFFSET, IR_LINE_OFFSET)),
            Field(
                &suchm::interface::GpioOutputRequest::initial_values,
                testing::ElementsAre(
                    suchm::interface::GpioValue::Inactive, suchm::interface::GpioValue::Inactive,
                    suchm::interface::GpioValue::Inactive)),
            Field(
                &suchm::interface::GpioOutputRequest::consumer,
                testing::StrEq("sinsei_umiusi_control::HeadlightsModel")))))
        .Times(1)
        .WillOnce(Return(ByMove(tl::expected<std::unique_ptr<suchm::interface::GpioLineRequest>, std::string>(
            std::move(gpio_lines)))));

    auto headlights_model =
        suchm::HeadlightsModel(
            std::move(gpio), HIGH_BEAM_LINE_OFFSET, LOW_BEAM_LINE_OFFSET, IR_LINE_OFFSET);
    auto result = headlights_model.on_init();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

TEST_P(HeadlightsModelOnWriteTest, all) {
    auto [high_beam_enabled, low_beam_enabled, ir_enabled, error] = GetParam();

    auto gpio = std::make_unique<mock::Gpio>();
    auto gpio_lines = std::make_unique<mock::GpioLineRequest>();
    auto requested_values = std::vector<suchm::interface::GpioValue>{
        suchm::interface::to_gpio_value(high_beam_enabled),
        suchm::interface::to_gpio_value(low_beam_enabled),
        suchm::interface::to_gpio_value(ir_enabled),
    };
    auto write_result = tl::expected<void, std::string>{};
    if (error != 0) {
        write_result = tl::make_unexpected(std::string("write failed"));
    }
    EXPECT_CALL(*gpio_lines, set_values(requested_values))
        .Times(1)
        .WillOnce(Return(std::move(write_result)));
    EXPECT_CALL(
        *gpio,
        request_outputs(testing::AllOf(
            Field(
                &suchm::interface::GpioOutputRequest::offsets,
                testing::ElementsAre(HIGH_BEAM_LINE_OFFSET, LOW_BEAM_LINE_OFFSET, IR_LINE_OFFSET)),
            Field(
                &suchm::interface::GpioOutputRequest::initial_values,
                testing::ElementsAre(
                    suchm::interface::GpioValue::Inactive, suchm::interface::GpioValue::Inactive,
                    suchm::interface::GpioValue::Inactive)),
            Field(
                &suchm::interface::GpioOutputRequest::consumer,
                testing::StrEq("sinsei_umiusi_control::HeadlightsModel")))))
        .Times(1)
        .WillOnce(Return(ByMove(tl::expected<std::unique_ptr<suchm::interface::GpioLineRequest>, std::string>(
            std::move(gpio_lines)))));

    auto headlights_model =
        suchm::HeadlightsModel(
            std::move(gpio), HIGH_BEAM_LINE_OFFSET, LOW_BEAM_LINE_OFFSET, IR_LINE_OFFSET);
    auto hb = succmd::headlights::HighBeamEnabled{high_beam_enabled};
    auto lb = succmd::headlights::LowBeamEnabled{low_beam_enabled};
    auto ir = succmd::headlights::IrEnabled{ir_enabled};
    headlights_model.on_init();
    const auto result = headlights_model.on_write(std::move(hb), std::move(lb), std::move(ir));
    if (error == 0) {
        ASSERT_TRUE(result) << std::string("Error: ") + result.error();
    } else {
        ASSERT_FALSE(result);
    }
}

}  // namespace sinsei_umiusi_control::test::hardware_model::headlights
