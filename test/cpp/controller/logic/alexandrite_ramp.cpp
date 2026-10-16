#include <gtest/gtest.h>

#include "sinsei_umiusi_control/controller/logic/alexandrite/ramp.hpp"

namespace logic = sinsei_umiusi_control::controller::logic::alexandrite;

TEST(AlexandriteRampTest, LimitsIncrease) { EXPECT_DOUBLE_EQ(logic::ramp(0.0, 0.5, 0.02), 0.02); }

TEST(AlexandriteRampTest, LimitsDecrease) { EXPECT_DOUBLE_EQ(logic::ramp(0.3, -0.5, 0.02), 0.28); }

TEST(AlexandriteRampTest, ReachesTargetWithinStep) {
    EXPECT_DOUBLE_EQ(logic::ramp(0.49, 0.5, 0.02), 0.5);
}

TEST(AlexandriteRampTest, ReachesTargetAfterEnoughSteps) {
    // 1.0 /s、50 Hz（0.02 s周期）なら、0から0.5まで25周期で届く
    auto duty = 0.0;
    for (int i = 0; i < 24; ++i) {
        duty = logic::ramp(duty, 0.5, 1.0 * 0.02);
    }
    EXPECT_LT(duty, 0.5);
    duty = logic::ramp(duty, 0.5, 1.0 * 0.02);
    EXPECT_NEAR(duty, 0.5, 1e-12);
}

TEST(AlexandriteRampTest, NegativeStepKeepsCurrent) {
    EXPECT_DOUBLE_EQ(logic::ramp(0.1, 0.5, -1.0), 0.1);
}
