#include "sinsei_umiusi_control/cmd/headlights.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rcpputils/tl_expected/expected.hpp>

#include "mock/gpio.hpp"
#include "sinsei_umiusi_control/hardware_model/headlights_model.hpp"
#include "sinsei_umiusi_control/util/gpio_interface.hpp"

namespace succmd = sinsei_umiusi_control::cmd;
namespace sucutil = sinsei_umiusi_control::util;
namespace suchm = sinsei_umiusi_control::hardware_model;

using sinsei_umiusi_control::test::mock::Gpio;
using testing::Return;

namespace sinsei_umiusi_control::test::hardware_model::headlights {

namespace {

constexpr auto _ = testing::_;

}

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
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
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

}  // namespace sinsei_umiusi_control::test::hardware_model::headlights
