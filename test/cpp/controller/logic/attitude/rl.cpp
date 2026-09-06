// RL logic の単体テスト。
//
// バンドルは models/ に同梱しているので環境変数なしで走る (CMake が SUC_MODELS_DIR で
// ソースツリーの models/ を指す)。同梱物が壊れたら CI が落ちるのが狙い。
//
// 別のバンドルで試すときだけ環境変数で上書きする:
//     SUC_RL_BUNDLE=<dir>/deploy.pt          direct 出力 (17/14 次元)
//     SUC_RL_BUNDLE_MODES=<dir>/deploy.pt    レンチモード (18 次元・6 次元レート)

#include "sinsei_umiusi_control/controller/logic/attitude/rl.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>

namespace attitude = sinsei_umiusi_control::controller::logic::attitude;
using sinsei_umiusi_control::controller::AttitudeController;

namespace {

// 同梱バンドルの deploy.pt。環境変数があればそちらを優先する。
auto bundle_path(const char * env, const char * shipped) -> std::string {
    const auto * p = std::getenv(env);
    if (p != nullptr && *p != '\0') {
        return std::string{p};
    }
    return (std::filesystem::path(SUC_MODELS_DIR) / shipped / "deploy.pt").string();
}

// 同梱しているバンドルの一覧 (models/README.md の表と 1:1)。
constexpr const char * SHIPPED[] = {
    "av_mode13", "av_cal1_best_rep103", "att_cal1_best_rep103", "av_cal5_3d_rep103",
    "av_sim2real2_rep103"};

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
    void load(const char * env, const char * shipped) {
        path_ = bundle_path(env, shipped);
        ASSERT_TRUE(std::filesystem::exists(path_)) << path_ << " がありません";
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

    std::string path_;
};

// `action_mode: "direct"` のバンドル (17 / 14 次元)。
class RlBundle : public BundleTest {
  protected:
    void SetUp() override { this->load("SUC_RL_BUNDLE", "av_cal1_best_rep103"); }
};

// `action_mode: "modes"` のバンドル (18 次元・6 次元レート)。
class RlModes : public BundleTest {
  protected:
    void SetUp() override { this->load("SUC_RL_BUNDLE_MODES", "av_mode13"); }
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

// ---------------------------------------------------------------------------
// 同梱バンドル (models/) — README.md の表に載っているもの全部
// ---------------------------------------------------------------------------

// 同梱物が壊れていない = 5 本すべてが読めて、frame 契約と観測レイアウトが合っていて、
// golden を通ること。umiusi_sim 側でバンドルを作り直してコピーし忘れると、ここが落ちる。
TEST(ShippedBundles, AllLoadAndPassGolden) {
    for (const auto * name : SHIPPED) {
        const auto dir = std::filesystem::path(SUC_MODELS_DIR) / name;
        SCOPED_TRACE(name);
        ASSERT_TRUE(std::filesystem::exists(dir / "deploy.pt")) << dir;
        ASSERT_TRUE(std::filesystem::exists(dir / "golden.pt")) << dir;

        const auto runner = attitude::PolicyRunner{(dir / "deploy.pt").string()};
        EXPECT_EQ(runner.obs_frame(), "rep103");
        // obs_fields が入っていれば警告は出ない。出るなら export が古い
        EXPECT_TRUE(runner.warnings().empty())
            << (runner.warnings().empty() ? "" : runner.warnings().front());

        auto ma = std::unique_ptr<attitude::ModeAction>{};
        if (runner.action_mode() == "modes") {
            ma = std::make_unique<attitude::ModeAction>(runner);
        }
        const auto g = attitude::verify_golden(runner, (dir / "golden.pt").string(), ma.get());
        EXPECT_GT(g.count, 0u);
        EXPECT_LE(g.worst, 1e-4);
        // レンチモードのバンドルは 3 段まで検証できていること
        EXPECT_EQ(g.has_mixed, runner.action_mode() == "modes");
    }
}

// 既定の rl.model_name が同梱されていること (これが無いと control_mode:=rl が起動しない)。
TEST(ShippedBundles, DefaultModelNameIsShipped) {
    const auto def = std::filesystem::path(SUC_MODELS_DIR) / "av_mode13" / "deploy.pt";
    EXPECT_TRUE(std::filesystem::exists(def))
        << "params/controllers.yaml の rl.model_name の既定と models/ が食い違っている";
}

// ---------------------------------------------------------------------------
// 鉛直指令インターロック
// ---------------------------------------------------------------------------

namespace {

auto shipped_options(const char * name) -> attitude::Rl::Options {
    auto opt = attitude::Rl::Options{};
    const auto dir = std::filesystem::path(SUC_MODELS_DIR) / name;
    opt.model_path = (dir / "deploy.pt").string();
    opt.golden_path = (dir / "golden.pt").string();
    opt.max_duty = 0.25;
    opt.servo_range_deg = 90.0;
    opt.servo_slew_deg_per_s = 250.0;
    opt.thrust_slew_per_s = 4.0;
    opt.hold_yaw = true;
    opt.control_hz = 50.0;
    return opt;
}

auto level_in() -> AttitudeController::Input {
    auto input = AttitudeController::Input{};
    input.state.imu_quaternion = {/*x=*/0.0, /*y=*/0.0, /*z=*/0.0, /*w=*/1.0};
    return input;
}

}  // namespace

// 水平専用の方策 (vertical_ok なし) では、鉛直の速度指令が出力を変えない。
// UI のゲームパッドは L2/R2 で velocity.z を送ってくるので実際に踏む経路。
TEST(VerticalInterlock, HorizontalOnlyPolicyIgnoresVerticalCommand) {
    const auto opt = shipped_options("av_mode13");  // vertical_ok なし
    auto input = level_in();
    input.cmd.target_velocity = {0.2, 0.0, 0.0};

    auto without = attitude::Rl{opt};
    auto with_z = attitude::Rl{opt};
    auto in_z = input;
    in_z.cmd.target_velocity.z = 0.3;   // UI の L2/R2 が送る値

    for (int step = 0; step < 20; ++step) {
        const auto a = without.update(step * 0.02, 0.02, input);
        const auto b = with_z.update(step * 0.02, 0.02, in_z);
        for (size_t i = 0; i < 4; ++i) {
            EXPECT_DOUBLE_EQ(a.cmd.esc_thrusts[i].value, b.cmd.esc_thrusts[i].value)
                << "step " << step << " esc " << i;
            EXPECT_DOUBLE_EQ(a.cmd.servo_angles[i].value, b.cmd.servo_angles[i].value)
                << "step " << step << " servo " << i;
        }
    }
    EXPECT_TRUE(with_z.vertical_clamped()) << "丸めたことが呼び出し側に伝わっていない";
    EXPECT_FALSE(without.vertical_clamped());
}

// 3-D 方策 (vertical_ok あり) には鉛直の速度指令がそのまま届く。
TEST(VerticalInterlock, VerticalCapablePolicyReceivesTheCommand) {
    const auto opt = shipped_options("av_cal5_3d_rep103");  // vertical_ok: true
    auto input = level_in();
    input.cmd.target_velocity = {0.2, 0.0, 0.0};

    auto without = attitude::Rl{opt};
    auto with_z = attitude::Rl{opt};
    auto in_z = input;
    in_z.cmd.target_velocity.z = 0.3;

    auto differed = false;
    for (int step = 0; step < 20; ++step) {
        const auto a = without.update(step * 0.02, 0.02, input);
        const auto b = with_z.update(step * 0.02, 0.02, in_z);
        for (size_t i = 0; i < 4; ++i) {
            differed = differed || a.cmd.esc_thrusts[i].value != b.cmd.esc_thrusts[i].value ||
                       a.cmd.servo_angles[i].value != b.cmd.servo_angles[i].value;
        }
    }
    EXPECT_TRUE(differed) << "鉛直指令が観測に届いていない";
    EXPECT_FALSE(with_z.vertical_clamped());
}

// --- 配備前検証の 5 段目: ホバリング economy ---------------------------------
// 他の 4 段は「契約が一致しているか」だけを見ていて、出来上がった方策の振る舞いは見ていない。
// 指令ゼロでも duty が上限に張り付く方策が golden PASS のまま実機に載った実績がある。

TEST(HoverEconomy, DisabledByDefaultSoOldBundlesStillLoad) {
    // 上限を設定しなければ、実測値が無いバンドルでも素通りする (既存挙動を壊さない)
    EXPECT_FALSE(attitude::hover_economy_error(std::nullopt, std::nullopt, 0.0, 0.0));
}

TEST(HoverEconomy, RejectsBundleThatWasNeverMeasured) {
    // ゲートを有効にしたのに実測値が無い = 一度も測っていない方策。警告で通してはいけない
    EXPECT_TRUE(attitude::hover_economy_error(std::nullopt, std::nullopt, 0.5, 0.3));
    EXPECT_TRUE(attitude::hover_economy_error(0.2, std::nullopt, 0.5, 0.3));
    EXPECT_TRUE(attitude::hover_economy_error(std::nullopt, 0.1, 0.5, 0.3));
}

TEST(HoverEconomy, PassesWhenBothAreUnderTheLimit) {
    EXPECT_FALSE(attitude::hover_economy_error(0.20, 0.10, 0.5, 0.3));
}

TEST(HoverEconomy, RejectsWastefulPolicy) {
    // 実測されたホバリング duty (cap の 90%) が通らないこと
    EXPECT_TRUE(attitude::hover_economy_error(0.90, 0.10, 0.5, 0.3));
}

TEST(HoverEconomy, RejectsPolicyThatTradedAttitudeForDuty) {
    // duty だけを見ると通してしまう方策。fy クランプ実験で duty -24% / ori +39% が実際に起きた
    EXPECT_FALSE(attitude::hover_economy_error(0.17, 0.495, 0.5, /*ori_limit=*/0.0));
    EXPECT_TRUE(attitude::hover_economy_error(0.17, 0.495, 0.5, /*ori_limit=*/0.3));
}

TEST(HoverEconomy, EachLimitCanBeUsedAlone) {
    EXPECT_TRUE(attitude::hover_economy_error(0.90, 0.10, /*duty=*/0.5, /*ori=*/0.0));
    EXPECT_FALSE(attitude::hover_economy_error(0.90, 0.10, /*duty=*/0.0, /*ori=*/0.3));
}
