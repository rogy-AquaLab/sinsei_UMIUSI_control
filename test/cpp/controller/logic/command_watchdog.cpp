#include "sinsei_umiusi_control/controller/logic/command_watchdog.hpp"

#include <gtest/gtest.h>

#include <chrono>

namespace sinsei_umiusi_control::test::controller::logic {

using sinsei_umiusi_control::controller::logic::CommandWatchdog;
using namespace std::chrono_literals;

const auto T0 = CommandWatchdog::Clock::time_point{} + 100s;

TEST(CommandWatchdogTest, DisabledIsAlwaysFresh) {
    auto watchdog = CommandWatchdog{0s};

    EXPECT_TRUE(watchdog.is_fresh(T0));
    watchdog.feed(T0);
    EXPECT_TRUE(watchdog.is_fresh(T0 + 1h));
}

TEST(CommandWatchdogTest, StaleUntilFirstCommand) {
    auto watchdog = CommandWatchdog{500ms};

    EXPECT_FALSE(watchdog.is_fresh(T0));
}

TEST(CommandWatchdogTest, FreshWithinTimeoutThenStale) {
    auto watchdog = CommandWatchdog{500ms};

    watchdog.feed(T0);
    EXPECT_TRUE(watchdog.is_fresh(T0));
    EXPECT_TRUE(watchdog.is_fresh(T0 + 500ms));
    EXPECT_FALSE(watchdog.is_fresh(T0 + 501ms));
}

TEST(CommandWatchdogTest, FeedingAgainRecovers) {
    auto watchdog = CommandWatchdog{500ms};

    watchdog.feed(T0);
    EXPECT_FALSE(watchdog.is_fresh(T0 + 2s));
    watchdog.feed(T0 + 2s);
    EXPECT_TRUE(watchdog.is_fresh(T0 + 2s + 100ms));
}

TEST(CommandWatchdogTest, TimeoutCanBeChanged) {
    auto watchdog = CommandWatchdog{};

    watchdog.feed(T0);
    EXPECT_TRUE(watchdog.is_fresh(T0 + 10s));
    watchdog.set_timeout(1s);
    EXPECT_FALSE(watchdog.is_fresh(T0 + 10s));
}

}  // namespace sinsei_umiusi_control::test::controller::logic
