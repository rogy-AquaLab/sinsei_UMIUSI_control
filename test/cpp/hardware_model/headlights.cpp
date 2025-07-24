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
: public ::testing::TestWithParam<std::tuple<bool, bool, bool, int>> {};
INSTANTIATE_TEST_CASE_P(
    HeadlightsModelTest, HeadlightsModelOnWriteTest,
    ::testing::Combine(
        ::testing::Bool(), ::testing::Bool(), ::testing::Bool(), ::testing::Values(0, 1, 2, 3)));

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
    auto [high_beam_enabled, low_beam_enabled, ir_enabled, error] = GetParam();

    auto gpio = std::make_unique<mock::Gpio>();

    const int n_true = static_cast<int>(high_beam_enabled) + static_cast<int>(low_beam_enabled) +
                       static_cast<int>(ir_enabled);
    const int n_false = 3 - n_true;

    switch (error) {
        case 1:  // High beam error
            EXPECT_CALL(*gpio, write_digital(HIGH_BEAM_PIN, high_beam_enabled))
                .Times(1)
                .WillOnce(Return(tl::make_unexpected(util::GpioError::UnknownError)));
            EXPECT_CALL(*gpio, write_digital(LOW_BEAM_PIN, low_beam_enabled)).Times(1);
            EXPECT_CALL(*gpio, write_digital(IR_PIN, ir_enabled)).Times(1);
            break;
        case 2:  // Low beam error
            EXPECT_CALL(*gpio, write_digital(HIGH_BEAM_PIN, high_beam_enabled)).Times(1);
            EXPECT_CALL(*gpio, write_digital(LOW_BEAM_PIN, low_beam_enabled))
                .Times(1)
                .WillOnce(Return(tl::make_unexpected(util::GpioError::UnknownError)));
            EXPECT_CALL(*gpio, write_digital(IR_PIN, ir_enabled)).Times(1);
            break;
        case 3:  // IR error
            EXPECT_CALL(*gpio, write_digital(HIGH_BEAM_PIN, high_beam_enabled)).Times(1);
            EXPECT_CALL(*gpio, write_digital(IR_PIN, ir_enabled))
                .Times(1)
                .WillOnce(Return(tl::make_unexpected(util::GpioError::UnknownError)));
            EXPECT_CALL(*gpio, write_digital(LOW_BEAM_PIN, low_beam_enabled)).Times(1);
            break;
        case 0:  // No error
        default:
            EXPECT_CALL(*gpio, write_digital(_, true)).Times(n_true);
            EXPECT_CALL(*gpio, write_digital(_, false)).Times(n_false);
            break;
    }

    auto headlights_model =
        suchm::HeadlightsModel(std::move(gpio), HIGH_BEAM_PIN, LOW_BEAM_PIN, IR_PIN);
    auto hb = succmd::headlights::HighBeamEnabled{high_beam_enabled};
    auto lb = succmd::headlights::LowBeamEnabled{low_beam_enabled};
    auto ir = succmd::headlights::IrEnabled{ir_enabled};
    const auto result = headlights_model.on_write(hb, lb, ir);
    if (error == 0) {
        ASSERT_TRUE(result) << std::string("Error: ") + result.error();
    } else {
        ASSERT_FALSE(result);
    }
}

}  // namespace sinsei_umiusi_control::test::hardware_model::headlights
