// RL logic の単体テスト。
//
// バンドル (deploy.pt) を要るテストは環境変数が指すときだけ走る。配備物は生成物で
// repo に入っていないため (umiusi_sim の .gitignore が models/ ごと除外)、CI では skip される。
//
//     SUC_RL_BUNDLE=<umiusi_sim>/models/av_cal1_best_rep103/deploy.pt
//     SUC_RL_BUNDLE_MODES=<umiusi_sim>/models/av_mode13/deploy.pt
//     colcon test --packages-select sinsei_umiusi_control
//
// `SUC_RL_BUNDLE` は direct 出力 (17/14 次元)、`SUC_RL_BUNDLE_MODES` はレンチモード
// (18 次元・6 次元レート) を指す。両方の経路を通す。

#include "sinsei_umiusi_control/controller/logic/attitude/rl.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>

namespace attitude = sinsei_umiusi_control::controller::logic::attitude;
using sinsei_umiusi_control::controller::AttitudeController;

namespace {

auto env_path(const char * name) -> std::string {
    const auto * p = std::getenv(name);
    return p == nullptr ? std::string{} : std::string{p};
}

}  // namespace

TEST(SubQuat, IdentityIsZero) {
    const auto e = attitude::sub_quat({1.0, 0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0});
    EXPECT_NEAR(e[0], 0.0, 1e-12);
    EXPECT_NEAR(e[1], 0.0, 1e-12);
    EXPECT_NEAR(e[2], 0.0, 1e-12);
}

TEST(SubQuat, QuarterTurnAboutZ) {
    // qa = +90 deg about z, qb = identity -> 回転ベクトルは (0, 0, pi/2)
    const auto c = std::cos(M_PI / 4.0);
    const auto s = std::sin(M_PI / 4.0);
    const auto e = attitude::sub_quat({c, 0.0, 0.0, s}, {1.0, 0.0, 0.0, 0.0});
    EXPECT_NEAR(e[0], 0.0, 1e-12);
    EXPECT_NEAR(e[1], 0.0, 1e-12);
    EXPECT_NEAR(e[2], M_PI / 2.0, 1e-12);
}

TEST(TargetQuat, ZeroRotvecIsIdentity) {
    const auto q = attitude::target_quat_from({0.0, 0.0, 0.0});
    EXPECT_NEAR(q[0], 1.0, 1e-12);
    EXPECT_NEAR(q[1], 0.0, 1e-12);
    EXPECT_NEAR(q[2], 0.0, 1e-12);
    EXPECT_NEAR(q[3], 0.0, 1e-12);
}

TEST(TargetQuat, RoundTripsThroughSubQuat) {
    // 目標を回転ベクトルで与えて姿勢が単位のとき、ori_err はその回転ベクトルそのもの
    const std::array<double, 3> v{0.1, -0.2, 0.3};
    const auto q = attitude::target_quat_from({v[0], v[1], v[2]});
    const auto e = attitude::sub_quat(q, {1.0, 0.0, 0.0, 0.0});
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(e[i], v[i], 1e-12);
    }
}

TEST(ToUnitQuat, ZeroFallsBackToIdentity) {
    const auto q = attitude::to_unit_quat({0.0, 0.0, 0.0, 0.0});
    EXPECT_NEAR(q[0], 1.0, 1e-12);
}

TEST(ToUnitQuat, Normalizes) {
    const auto q = attitude::to_unit_quat({/*x=*/2.0, /*y=*/0.0, /*z=*/0.0, /*w=*/0.0});
    EXPECT_NEAR(q[0], 0.0, 1e-12);  // w
    EXPECT_NEAR(q[1], 1.0, 1e-12);  // x
}

TEST(Slew, LimitsTheStep) {
    EXPECT_NEAR(attitude::slew(0.0, 1.0, 4.0, 0.02), 0.08, 1e-12);
    EXPECT_NEAR(attitude::slew(0.0, -1.0, 4.0, 0.02), -0.08, 1e-12);
    // 目標が届く範囲なら行き過ぎない
    EXPECT_NEAR(attitude::slew(0.0, 0.01, 4.0, 0.02), 0.01, 1e-12);
    // 0 以下は無効 = 制限しない
    EXPECT_NEAR(attitude::slew(0.0, 1.0, 0.0, 0.02), 1.0, 1e-12);
}

// バンドルを指す環境変数が要るテストの共通土台。
class BundleTest : public ::testing::Test {
  protected:
    void load(const char * env) {
        env_ = env;
        path_ = env_path(env);
        if (path_.empty()) {
            GTEST_SKIP() << env << " が未設定なので skip (deploy.pt は生成物)";
        }
        if (!std::filesystem::exists(path_)) {
            GTEST_SKIP() << env << " が指す " << path_ << " がありません";
        }
    }

    auto options() const -> attitude::Rl::Options {
        auto opt = attitude::Rl::Options{};
        opt.model_path = path_;
        const auto golden = std::filesystem::path(path_).parent_path() / "golden.pt";
        opt.golden_path = std::filesystem::exists(golden) ? golden.string() : std::string{};
        opt.max_duty = 0.25;
        opt.servo_range_deg = 90.0;
        opt.servo_slew_deg_per_s = 250.0;
        opt.thrust_slew_per_s = 4.0;
        opt.hold_yaw = true;
        opt.control_hz = 50.0;
        return opt;
    }

    static auto level_input() -> AttitudeController::Input {
        auto input = AttitudeController::Input{};
        input.state.imu_quaternion = {/*x=*/0.0, /*y=*/0.0, /*z=*/0.0, /*w=*/1.0};
        return input;
    }

    const char * env_{nullptr};
    std::string path_;
};

// `action_mode: "direct"` のバンドル (17 / 14 次元)。
class RlBundle : public BundleTest {
  protected:
    void SetUp() override { this->load("SUC_RL_BUNDLE"); }
};

// `action_mode: "modes"` のバンドル (18 次元・6 次元レート)。
class RlModes : public BundleTest {
  protected:
    void SetUp() override { this->load("SUC_RL_BUNDLE_MODES"); }
};

// 配備前検証。重み・正規化統計・観測レイアウトが sim と食い違っていればここで落ちる。
TEST_F(RlBundle, LoadsAndPassesGolden) {
    const auto opt = this->options();
    ASSERT_FALSE(opt.golden_path.empty()) << "golden.pt が無いので検証できない";
    const auto runner = attitude::PolicyRunner{opt.model_path};
    const auto g = attitude::verify_golden(runner, opt.golden_path);
    EXPECT_GT(g.count, 0u);
    EXPECT_LE(g.worst, 1e-4);
}

// 出力が指令のレート制限と duty 上限を守る。1 tick 目は 0 から max_rate * dt しか動けない。
TEST_F(RlBundle, FirstStepRespectsSlewAndDutyLimit) {
    const auto opt = this->options();
    auto rl = attitude::Rl{opt};

    auto input = AttitudeController::Input{};
    input.state.imu_quaternion = {/*x=*/0.0, /*y=*/0.0, /*z=*/0.0, /*w=*/1.0};
    input.cmd.target_orientation = {0.2, -0.1, 0.0};  // 回転ベクトル [rad]

    constexpr auto DT = 0.02;  // 50 Hz
    const auto out = rl.update(0.0, DT, input);
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_LE(std::abs(out.cmd.servo_angles[i].value), opt.servo_slew_deg_per_s * DT + 1e-9);
        EXPECT_LE(std::abs(out.cmd.esc_thrusts[i].value), opt.thrust_slew_per_s * DT + 1e-9);
        EXPECT_LE(std::abs(out.cmd.esc_thrusts[i].value), opt.max_duty + 1e-9);
    }
}

// duty 上限は何 tick 回しても効き続ける (レート制限が追いついた後も)。
TEST_F(RlBundle, HoldsDutyLimitOverTime) {
    const auto opt = this->options();
    auto rl = attitude::Rl{opt};

    auto input = AttitudeController::Input{};
    input.state.imu_quaternion = {0.0, 0.0, 0.0, 1.0};
    input.cmd.target_orientation = {0.5, 0.5, 0.0};

    auto out = AttitudeController::Output{};
    for (int step = 0; step < 200; ++step) {
        out = rl.update(step * 0.02, 0.02, input);
    }
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_TRUE(std::isfinite(out.cmd.esc_thrusts[i].value));
        EXPECT_LE(std::abs(out.cmd.esc_thrusts[i].value), opt.max_duty + 1e-9);
        EXPECT_LE(std::abs(out.cmd.servo_angles[i].value), 90.0 + 1e-9);
    }
}

// init() は出力と内部状態 (prev_action / レート制限) を 0 に戻す。
TEST_F(RlBundle, InitResetsState) {
    const auto opt = this->options();
    auto rl = attitude::Rl{opt};

    auto input = AttitudeController::Input{};
    input.state.imu_quaternion = {0.0, 0.0, 0.0, 1.0};
    input.cmd.target_orientation = {0.5, 0.5, 0.0};
    for (int step = 0; step < 50; ++step) {
        rl.update(step * 0.02, 0.02, input);
    }
    const auto zeroed = rl.init(0.0, input, AttitudeController::Output{});
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_EQ(zeroed.cmd.esc_thrusts[i].value, 0.0);
        EXPECT_EQ(zeroed.cmd.servo_angles[i].value, 0.0);
    }
    // 状態が戻っているので、次の 1 tick はまた 0 からのレート制限に従う
    const auto out = rl.update(0.0, 0.02, input);
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_LE(std::abs(out.cmd.esc_thrusts[i].value), opt.thrust_slew_per_s * 0.02 + 1e-9);
    }
}

// direct のバンドルも obs_fields を運ぶので、既定設定なら警告が出ない。
TEST_F(RlBundle, LoadsCleanlyWithDefaultParams) {
    const auto rl = attitude::Rl{this->options()};
    EXPECT_EQ(rl.report().find("[warn]"), std::string::npos) << rl.report();
    EXPECT_NE(rl.report().find("golden 検証 PASS"), std::string::npos) << rl.report();
}

TEST_F(RlBundle, ReportsControlMode) {
    const auto rl = attitude::Rl{this->options()};
    EXPECT_EQ(rl.control_mode(), sinsei_umiusi_control::controller::logic::ControlMode::Rl);
}

// ---------------------------------------------------------------------------
// action_mode: "modes" (レンチモード)
// ---------------------------------------------------------------------------

TEST_F(RlModes, BundleCarriesTheWholeContract) {
    const auto runner = attitude::PolicyRunner{this->path_};
    EXPECT_EQ(runner.action_mode(), "modes");
    EXPECT_EQ(runner.act_dim(), attitude::PolicyRunner::MODE_DIM);
    EXPECT_EQ(runner.obs_dim(), attitude::PolicyRunner::OBS_DIM_CAP);
    EXPECT_TRUE(runner.needs_max_duty());
    EXPECT_TRUE(runner.needs_velocity());
    // 18 次元は obs_fields が必須。無ければ PolicyRunner が落ちるので、
    // ここまで来た時点で照合は通っている
    EXPECT_TRUE(runner.warnings().empty()) << runner.warnings().front();
}

// ネットの生出力に加えて、積分・ミキサ・折返しの 3 段も Python 実装と突き合わせる。
// 生出力だけでは 3 段の取り違えが素通りする。
TEST_F(RlModes, LoadsAndPassesGolden) {
    const auto opt = this->options();
    ASSERT_FALSE(opt.golden_path.empty()) << "golden.pt が無いので検証できない";
    const auto runner = attitude::PolicyRunner{opt.model_path};
    auto ma = attitude::ModeAction{runner};
    const auto g = attitude::verify_golden(runner, opt.golden_path, &ma);
    EXPECT_GT(g.count, 0u);
    EXPECT_LE(g.worst, 1e-4);
    EXPECT_TRUE(g.has_mixed) << "golden.pt に mixed が無い (export_deploy_bundle.py が古い)";
    EXPECT_LE(g.mixed_worst, 1e-4);
}

// ミキサ 3 段を既知の入力で固定する。zero rate なら積分器は 0 のままで、
// デッドバンドに入って出力も 0。
TEST_F(RlModes, ZeroRateStaysInTheDeadband) {
    const auto runner = attitude::PolicyRunner{this->path_};
    auto ma = attitude::ModeAction{runner};
    const auto out = ma.step(std::vector<double>(6, 0.0), 0.25, 0.02);
    for (const auto v : out) {
        EXPECT_EQ(v, 0.0);
    }
}

// 鉛直モード (fz) を 1 秒ぶん積分すると、全ユニットが真上を向いて f_max を出す。
// 期待値は action_contract の係数から手で解いたもの:
//   m.fz = clamp(1 * 2.0 /s * 1 s) = 1
//   f_max = thrust_per_cmd * max_duty^exp = 30 * 0.25^2 = 1.875 N
//   h = 0, v = f_max  ->  phi = +90 deg = +1.0 (正規化)
//   esc = (f_max / thrust_per_cmd)^(1/exp) = 0.25
TEST_F(RlModes, VerticalModeMixesToFullUp) {
    const auto runner = attitude::PolicyRunner{this->path_};
    auto ma = attitude::ModeAction{runner};
    ASSERT_DOUBLE_EQ(ma.mode_slew_per_s(), 2.0);
    ASSERT_DOUBLE_EQ(ma.thrust_per_cmd(), 30.0);
    ASSERT_DOUBLE_EQ(ma.thrust_curve_exp(), 2.0);
    ASSERT_DOUBLE_EQ(ma.servo_range_deg(), 90.0);

    // mode_names の並びは [fx, fy, fz, tx, ty, tz]
    auto rate = std::vector<double>(6, 0.0);
    rate[2] = 1.0;  // fz
    const auto out = ma.step(rate, 0.25, 1.0);
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_NEAR(out[i], 1.0, 1e-12) << "servo[" << i << "]";
        EXPECT_NEAR(out[4 + i], 0.25, 1e-12) << "esc[" << i << "]";
    }
}

// 積分器は tick をまたいで残り、reset() で戻る。
TEST_F(RlModes, IntegratorPersistsAndResets) {
    const auto runner = attitude::PolicyRunner{this->path_};
    auto ma = attitude::ModeAction{runner};
    auto rate = std::vector<double>(6, 0.0);
    rate[2] = 1.0;  // fz

    const auto one = ma.step(rate, 0.25, 0.1);  // m.fz = 0.2
    const auto two = ma.step(rate, 0.25, 0.1);  // m.fz = 0.4 (積分が残っている)
    EXPECT_GT(two[4], one[4]);

    ma.reset();
    const auto after = ma.step(rate, 0.25, 0.1);  // 0 から積み直し
    EXPECT_NEAR(after[4], one[4], 1e-12);
}

TEST_F(RlModes, RespectsSlewAndDutyLimitThroughTheLogic) {
    const auto opt = this->options();
    auto rl = attitude::Rl{opt};
    auto input = this->level_input();
    input.cmd.target_orientation = {0.3, -0.2, 0.0};
    input.cmd.target_velocity = {0.3, 0.0, 0.0};

    constexpr auto DT = 0.02;
    const auto first = rl.update(0.0, DT, input);
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_LE(std::abs(first.cmd.servo_angles[i].value), opt.servo_slew_deg_per_s * DT + 1e-9);
        EXPECT_LE(std::abs(first.cmd.esc_thrusts[i].value), opt.thrust_slew_per_s * DT + 1e-9);
    }
    auto out = AttitudeController::Output{};
    for (int step = 0; step < 500; ++step) {
        out = rl.update(step * DT, DT, input);
    }
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_TRUE(std::isfinite(out.cmd.esc_thrusts[i].value));
        EXPECT_LE(std::abs(out.cmd.esc_thrusts[i].value), opt.max_duty + 1e-9);
        EXPECT_LE(std::abs(out.cmd.servo_angles[i].value), 90.0 + 1e-9);
    }
}

// 契約とパラメータの servo_range_deg が食い違ったら起動させない
// (ミキサの正規化と出力側の逆正規化が食い違い、角度が別物になる)。
TEST_F(RlModes, RefusesServoRangeMismatch) {
    auto opt = this->options();
    opt.servo_range_deg = 45.0;
    EXPECT_THROW({ attitude::Rl{opt}; }, std::runtime_error);
}

// 推奨設定 (params/controllers.yaml の既定値) で読み込んだら、起動ログに警告が出ない。
// 「普通に起動したらエラーも警告も無い」ことを固定する。
TEST_F(RlModes, LoadsCleanlyWithDefaultParams) {
    const auto rl = attitude::Rl{this->options()};
    EXPECT_EQ(rl.report().find("[warn]"), std::string::npos) << rl.report();
    EXPECT_NE(rl.report().find("golden 検証 PASS"), std::string::npos) << rl.report();
    EXPECT_NE(rl.report().find("mixed"), std::string::npos) << rl.report();
}

// duty 上限が学習分布の外なら、黙って丸めずに警告を残す。
TEST_F(RlModes, WarnsWhenMaxDutyIsOutOfDistribution) {
    auto opt = this->options();
    opt.max_duty = 0.6;
    const auto rl = attitude::Rl{opt};
    EXPECT_NE(rl.report().find("[warn]"), std::string::npos) << rl.report();
}

// frame 契約 (2026-08-21 の再発防止ゲート): 同梱バンドルはすべて rep103。
TEST_F(RlBundle, EnforcesRep103Frame) {
    const auto runner = attitude::PolicyRunner{this->path_};
    EXPECT_EQ(runner.obs_frame(), "rep103");
}

TEST_F(RlModes, EnforcesRep103Frame) {
    const auto runner = attitude::PolicyRunner{this->path_};
    EXPECT_EQ(runner.obs_frame(), "rep103");
}
