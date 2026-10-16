#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>

#include "sinsei_umiusi_control/controller/logic/alexandrite/mixer.hpp"

namespace mixer = sinsei_umiusi_control::controller::logic::alexandrite;

namespace {

constexpr auto TOLERANCE = 1e-9;
// 飽和しないよう十分大きくする
constexpr auto UNLIMITED_DUTY = 1e9;

const auto DEGREES_TO_RADIANS = std::acos(-1.0) / 180.0;
// Alexandriteの実測値（mm）
const auto GEOMETRY = mixer::Geometry{200.0, 175.0, 300.0, 20.0 * DEGREES_TO_RADIANS};
// UMIUSIと同じ45度配置
const auto UMIUSI_LIKE_GEOMETRY = mixer::Geometry{0.6, 0.8, 1.3, 45.0 * DEGREES_TO_RADIANS};

// スラスタの出力から、機体に働く力・モーメントを計算する（ミキサーの逆）
// ミキサーと同じく、前側2基までの距離を1とした長さでモーメントを計算する
auto to_wrench(
    const std::array<mixer::ThrusterOutput, mixer::THRUSTER_COUNT> & outputs,
    const mixer::Geometry & raw_geometry) -> mixer::Wrench {
    const auto scale = std::hypot(raw_geometry.front_x, raw_geometry.front_y);
    const auto geometry = mixer::Geometry{
        raw_geometry.front_x / scale, raw_geometry.front_y / scale, raw_geometry.rear_x / scale,
        raw_geometry.front_angle};
    const auto cos_phi = std::cos(geometry.front_angle);
    const auto sin_phi = std::sin(geometry.front_angle);
    // 位置と、サーボ角0・正Dutyでの水平推力の方向（thruster1〜3 = 左前、後ろ、右前）
    const auto positions = std::array<std::array<double, 2>, mixer::THRUSTER_COUNT>{{
        {geometry.front_x, geometry.front_y},
        {-geometry.rear_x, 0.0},
        {geometry.front_x, -geometry.front_y},
    }};
    const auto directions = std::array<std::array<double, 2>, mixer::THRUSTER_COUNT>{{
        {-cos_phi, sin_phi},
        {0.0, -1.0},
        {cos_phi, sin_phi},
    }};

    auto wrench = mixer::Wrench{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < mixer::THRUSTER_COUNT; ++i) {
        const auto horizontal = outputs[i].duty * std::cos(outputs[i].angle);
        const auto vertical = outputs[i].duty * std::sin(outputs[i].angle);
        const auto fx = horizontal * directions[i][0];
        const auto fy = horizontal * directions[i][1];
        const auto [x, y] = positions[i];
        wrench.fx += fx;
        wrench.fy += fy;
        wrench.fz += vertical;
        wrench.mx += y * vertical;
        wrench.my += -x * vertical;
        wrench.mz += x * fy - y * fx;
    }
    return wrench;
}

auto expect_wrench_near(const mixer::Wrench & actual, const mixer::Wrench & expected) {
    EXPECT_NEAR(actual.fx, expected.fx, TOLERANCE);
    EXPECT_NEAR(actual.fy, expected.fy, TOLERANCE);
    EXPECT_NEAR(actual.fz, expected.fz, TOLERANCE);
    EXPECT_NEAR(actual.mx, expected.mx, TOLERANCE);
    EXPECT_NEAR(actual.my, expected.my, TOLERANCE);
    EXPECT_NEAR(actual.mz, expected.mz, TOLERANCE);
}

}  // namespace

TEST(AlexandriteMixerTest, ReproducesTargetWrench) {
    const auto targets = std::array<mixer::Wrench, 4>{{
        {0.3, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, -0.2, 0.0, 0.0, 0.0, 0.0},
        {0.1, 0.2, -0.3, 0.05, -0.1, 0.15},
        {-0.2, 0.1, 0.4, -0.2, 0.3, -0.1},
    }};
    for (const auto & geometry : {GEOMETRY, UMIUSI_LIKE_GEOMETRY}) {
        for (const auto & target : targets) {
            const auto outputs = mixer::mix(target, geometry, UNLIMITED_DUTY);
            expect_wrench_near(to_wrench(outputs, geometry), target);
        }
    }
}

TEST(AlexandriteMixerTest, PureYawDrivesEveryThrusterPositive) {
    // どのスラスタも、正のDutyで反時計回りのヨーを生む
    const auto outputs = mixer::mix({0.0, 0.0, 0.0, 0.0, 0.0, 0.2}, GEOMETRY, UNLIMITED_DUTY);
    for (const auto & output : outputs) {
        EXPECT_GT(output.duty, 0.0);
        EXPECT_NEAR(output.angle, 0.0, TOLERANCE);
    }
}

TEST(AlexandriteMixerTest, LateralMotionDoesNotCreateYaw) {
    const auto target = mixer::Wrench{0.0, 0.3, 0.0, 0.0, 0.0, 0.0};
    const auto outputs = mixer::mix(target, GEOMETRY, UNLIMITED_DUTY);
    const auto wrench = to_wrench(outputs, GEOMETRY);
    EXPECT_NEAR(wrench.mz, 0.0, TOLERANCE);
    EXPECT_NEAR(wrench.fy, 0.3, TOLERANCE);
}

TEST(AlexandriteMixerTest, ForwardMotionUsesOnlyFrontThrusters) {
    const auto outputs = mixer::mix({0.3, 0.0, 0.0, 0.0, 0.0, 0.0}, GEOMETRY, UNLIMITED_DUTY);
    EXPECT_NEAR(outputs[1].duty, 0.0, TOLERANCE);
    // 前側2基は逆向きに回し、横方向の力とヨーを打ち消す
    EXPECT_NEAR(outputs[0].duty, -outputs[2].duty, TOLERANCE);
}

TEST(AlexandriteMixerTest, VerticalOnlyTiltsServosStraightUp) {
    const auto outputs = mixer::mix({0.0, 0.0, 0.3, 0.0, 0.0, 0.0}, GEOMETRY, UNLIMITED_DUTY);
    for (const auto & output : outputs) {
        EXPECT_GT(output.duty, 0.0);
        EXPECT_NEAR(output.angle, std::acos(0.0), TOLERANCE);
    }
}

TEST(AlexandriteMixerTest, DownwardOnlyTiltsServosStraightDown) {
    const auto outputs = mixer::mix({0.0, 0.0, -0.3, 0.0, 0.0, 0.0}, GEOMETRY, UNLIMITED_DUTY);
    for (const auto & output : outputs) {
        EXPECT_GT(output.duty, 0.0);
        EXPECT_NEAR(output.angle, -std::acos(0.0), TOLERANCE);
    }
}

TEST(AlexandriteMixerTest, ZeroTargetStopsEveryThruster) {
    const auto outputs = mixer::mix({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, GEOMETRY, UNLIMITED_DUTY);
    for (const auto & output : outputs) {
        EXPECT_EQ(output.duty, 0.0);
        EXPECT_EQ(output.angle, 0.0);
    }
}

TEST(AlexandriteMixerTest, SaturationKeepsWrenchDirection) {
    constexpr auto MAX_DUTY = 0.5;
    const auto target = mixer::Wrench{2.0, -1.0, 3.0, 0.5, -1.5, 2.5};
    const auto outputs = mixer::mix(target, GEOMETRY, MAX_DUTY);

    auto max_abs = 0.0;
    for (const auto & output : outputs) {
        max_abs = std::max(max_abs, std::abs(output.duty));
    }
    EXPECT_NEAR(max_abs, MAX_DUTY, TOLERANCE);

    // 3基を同じ比率で縮めるため、力・モーメントは目標の定数倍になる
    const auto wrench = to_wrench(outputs, GEOMETRY);
    const auto ratio = wrench.fx / target.fx;
    EXPECT_GT(ratio, 0.0);
    EXPECT_LT(ratio, 1.0);
    expect_wrench_near(
        wrench, {target.fx * ratio, target.fy * ratio, target.fz * ratio, target.mx * ratio,
                 target.my * ratio, target.mz * ratio});
}

TEST(AlexandriteMixerTest, GeometryUsesOnlyRatios) {
    const auto target = mixer::Wrench{0.1, 0.2, -0.3, 0.05, -0.1, 0.15};
    // mmとmで同じ出力になる
    const auto scaled = mixer::Geometry{
        GEOMETRY.front_x * 0.001, GEOMETRY.front_y * 0.001, GEOMETRY.rear_x * 0.001,
        GEOMETRY.front_angle};
    const auto outputs = mixer::mix(target, GEOMETRY, UNLIMITED_DUTY);
    const auto scaled_outputs = mixer::mix(target, scaled, UNLIMITED_DUTY);
    for (std::size_t i = 0; i < mixer::THRUSTER_COUNT; ++i) {
        EXPECT_NEAR(outputs[i].duty, scaled_outputs[i].duty, TOLERANCE);
        EXPECT_NEAR(outputs[i].angle, scaled_outputs[i].angle, TOLERANCE);
    }
}
