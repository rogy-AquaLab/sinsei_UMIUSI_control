#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_RL_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_RL_HPP

// 学習済み方策で姿勢を制御する logic。`control_mode:=rl` で選ぶ。
//
// 読むのは umiusi_sim の `tools/export_deploy_bundle.py` が作る `deploy.pt` 1 ファイル。
// 方策を作るのも配備物を作るのも sim なので、この経路に autonomy は入らない。
// ネットと正規化パラメータと `action_contract` が入った TorchScript で、C++ 側に
// JSON パーサも npz リーダも要らない。
//
// 観測の並びはバンドルが決める (`obs_field_names` / `obs_field_widths` で照合する):
//   18 次元 [ori_err(3), gyro(3), v_cmd(3), prev_action(8), max_duty(1)]
//   17 次元 [ori_err(3), gyro(3), v_cmd(3), prev_action(8)]
//   14 次元 [ori_err(3), gyro(3),           prev_action(8)]
//
// 出力の形は `action_mode` が決める:
//   "direct" — 8 次元 [servo x4, esc x4] をそのまま使う
//   "modes"  — 6 次元のレンチモードレート。積分 -> ミキサ -> 折返しの 3 段で
//              8 次元に直す (`ModeAction`)。sim が回したのと同じ 3 段を同じ順序で
//              再現しないと、学習したのと別のプラントになる
//
// 守ること:
//   * 観測の順序と正規化はバンドルと厳密に合わせる。ずれても golden 以外では気付けない
//     — その golden も並びの取り違えは検出できない (組み立て済みの観測を再生するだけ)
//     ので、`obs_fields` の照合が唯一のゲートになる
//   * `prev_action` は「自分が直前に出した指令」。init() でゼロに戻す
//   * IMU の異常サンプルはここでは弾いていない。Python の参照実装 (`ImuSanity`) は
//     弾いており、観測に直接入るので 1 発で指令が跳ねる (autonomy known_issues A-1)。
//     `prev_action` を通って次の観測にも戻るので、跳ねは数 tick 残る。
//     `to_unit_quat` はゼロ quat しか守らない — フィルタは呼び出し側の責務
//   * スレッド数は 1 に固定する。他のノードと CPU を奪い合うと多スレッドは逆効果
//     (autonomy の docs/performance_tuning.md、検出器で実測)
//
// 出力は duty [-1, 1] とサーボ角 [deg]。方策は duty で学習しているのでそのまま流す。
// 推力 [N] で出す案 (`thrust_per_cmd` / `thrust_curve_exp` で順写像) は保留 — 下流の
// `ThrusterController` は `duty = duty_per_thrust * esc_thrust` の線形で、
// `duty_per_thrust` も `thrust_per_cmd` もベンチ未較正のまま食い違っている。
//
// 指令のレート制限はここで掛ける。sim のプラントが持っていたもの。ただし esc は
// 下流の `ThrusterController` も `max_duty_step_per_sec` で制限するので、実効レートは
// 両者の厳しい方:
//   * `params/controllers.yaml` の既定は 1.0/s で、方策が学習・golden 検証された
//     `rl.thrust_slew_per_s: 4.0` より厳しい。rl で走らせるなら上げないと、
//     duty 0.25 に届くまで 62.5 ms ではなく 250 ms かかる (sim と別のプラントになる)
//   * サーボにはどちらの経路にも制限が無いので、`rl.servo_slew_deg_per_s` が唯一の制限

#include <ATen/Parallel.h>
#include <torch/script.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "sinsei_umiusi_control/controller/attitude_controller.hpp"

namespace sinsei_umiusi_control::controller::logic::attitude {

// スラスタの並び。`attitude_controller.cpp` の `THRUSTER_SUFFIX` と、
// バンドルの `mode_positions` の両方に一致していなければならない。
inline constexpr std::array<std::string_view, 4> POSITIONS{"lf", "lb", "rb", "rf"};

// 方策の推論と、バンドル (`deploy.pt`) の読み込み・検証。
// 正規化 -> 推論 -> クリップ を Python の `policy_infer.PolicyRunner` と同じ順序で行う。
// 正規化を float64 で計算してから float32 に落とすところまで揃えること —
// float32 で計算すると golden がわずかにずれる。
class PolicyRunner {
  public:
    static constexpr int64_t OBS_DIM_CAP = 18;       // + duty 上限
    static constexpr int64_t OBS_DIM_WITH_VEL = 17;  // attitude_velocity
    static constexpr int64_t OBS_DIM_NO_VEL = 14;    // attitude (速度指令を持たない)
    static constexpr int64_t ACT_DIM = 8;            // [servo x4, esc x4]
    static constexpr int64_t MODE_DIM = 6;           // レンチモードのレート

    explicit PolicyRunner(const std::string & deploy_path) : path_(deploy_path) {
        // 他ノードと CPU を奪い合うので 1 スレッドに固定する
        at::set_num_threads(1);
        try {
            module_ = torch::jit::load(deploy_path);
        } catch (const std::exception & e) {
            throw std::runtime_error(
                "バンドルを読み込めません (" + deploy_path + "): " + e.what() +
                " — tools/export_deploy_bundle.py で作った deploy.pt を指定してください");
        }
        module_.eval();

        mean_ = to_vector(this->tensor_attr("obs_mean"));
        var_ = to_vector(this->tensor_attr("obs_var"));
        clip_ = this->attr("obs_clip").toDouble();
        eps_ = this->attr("obs_eps").toDouble();
        obs_dim_ = this->attr("obs_dim").toInt();
        act_dim_ = this->attr("act_dim").toInt();
        action_mode_ = this->attr("action_mode").toStringRef();
        obs_frame_ = this->attr("obs_frame").toStringRef();

        // frame 契約: この logic は IMU を無変換 (REP-103) で観測に入れるので、
        // rep103 変換済みのバンドルだけを許す。2026-08-21 のプール試験では
        // pitch/yaw が入れ替わった観測が入り、姿勢制御が全く効かなかった
        if (obs_frame_ != "rep103") {
            throw std::runtime_error(
                "obs_frame=\"" + obs_frame_ + "\" のバンドルです (" + deploy_path +
                ")。この logic は IMU を無変換 (REP-103) で観測に入れるので、rep103 変換済み"
                "バンドルだけを使えます (umiusi_sim の tools/convert_policy_frame.py)");
        }
        if (action_mode_ != "direct" && action_mode_ != "modes") {
            throw std::runtime_error(
                "未対応の action_mode=\"" + action_mode_ + "\" です (" + deploy_path +
                ")。\"direct\" か \"modes\" のみ対応します");
        }
        const auto want_act = action_mode_ == "modes" ? MODE_DIM : ACT_DIM;
        if (act_dim_ != want_act) {
            throw std::runtime_error(
                "action_mode=\"" + action_mode_ + "\" なのに act_dim=" + std::to_string(act_dim_) +
                " です (" + deploy_path + ")。" + std::to_string(want_act) +
                " 次元を出す約束です");
        }
        if (obs_dim_ != OBS_DIM_CAP && obs_dim_ != OBS_DIM_WITH_VEL &&
            obs_dim_ != OBS_DIM_NO_VEL) {
            throw std::runtime_error(
                "対応していない観測次元 " + std::to_string(obs_dim_) + " です (" + deploy_path +
                ")。" + std::to_string(OBS_DIM_CAP) + " / " + std::to_string(OBS_DIM_WITH_VEL) +
                " / " + std::to_string(OBS_DIM_NO_VEL) + " のみ対応します");
        }
        if (static_cast<int64_t>(mean_.size()) != obs_dim_ || mean_.size() != var_.size()) {
            throw std::runtime_error(
                "obs_norm の次元がポリシーの入力次元と一致しません (" + deploy_path + ")");
        }
        this->check_obs_fields();

        x_ = torch::empty({1, obs_dim_}, torch::kFloat32);
        inputs_.emplace_back(x_);
    }

    auto obs_dim() const -> int64_t { return obs_dim_; }
    auto act_dim() const -> int64_t { return act_dim_; }
    auto action_mode() const -> const std::string & { return action_mode_; }
    auto obs_frame() const -> const std::string & { return obs_frame_; }
    auto path() const -> const std::string & { return path_; }
    auto needs_velocity() const -> bool { return obs_dim_ != OBS_DIM_NO_VEL; }
    auto needs_max_duty() const -> bool { return obs_dim_ == OBS_DIM_CAP; }
    // 読み込み時に出た「落とすほどではないが黙って通したくない」こと。
    auto warnings() const -> const std::vector<std::string> & { return warnings_; }

    // 制御周期で回る側はこちらを使い、out を使い回すこと。入力テンソルと引数リストは
    // 構築時に確保して使い回している — update() は controller_manager の更新スレッド上で
    // 走るので、毎 tick のヒープ確保を持ち込まない
    void act(const std::vector<double> & obs, std::vector<double> & out) const {
        if (static_cast<int64_t>(obs.size()) != obs_dim_) {
            throw std::runtime_error(
                "観測の次元が " + std::to_string(obs.size()) + " です (" +
                std::to_string(obs_dim_) + " が必要)");
        }
        auto * p = x_.data_ptr<float>();
        for (int64_t i = 0; i < obs_dim_; ++i) {
            const double n = (obs[i] - mean_[i]) / std::sqrt(var_[i] + eps_);
            p[i] = static_cast<float>(std::clamp(n, -clip_, clip_));
        }
        torch::NoGradGuard no_grad;
        const auto y = module_.forward(inputs_).toTensor().squeeze(0).contiguous();
        const auto * q = y.data_ptr<float>();
        out.resize(static_cast<size_t>(y.numel()));
        for (size_t i = 0; i < out.size(); ++i) {
            out[i] = std::clamp(static_cast<double>(q[i]), -1.0, 1.0);
        }
    }

    // 制御周期の外 (検証・テスト) 用。
    auto act(const std::vector<double> & obs) const -> std::vector<double> {
        auto out = std::vector<double>{};
        this->act(obs, out);
        return out;
    }

    // バンドルの属性を読む。無ければ「deploy.pt が古い」と分かるメッセージで落とす。
    auto attr(const std::string & name) const -> c10::IValue {
        try {
            return module_.attr(name);
        } catch (const std::exception & e) {
            throw std::runtime_error(
                "バンドルに属性 " + name + " がありません (" + path_ + "): " + e.what() +
                " — deploy.pt が古いので export_deploy_bundle.py を流し直してください");
        }
    }

    auto tensor_attr(const std::string & name) const -> at::Tensor {
        return this->attr(name).toTensor().to(torch::kFloat64).contiguous();
    }

    auto double_attr(const std::string & name) const -> double {
        return this->attr(name).toDouble();
    }

    auto string_list_attr(const std::string & name) const -> std::vector<std::string> {
        const auto list = this->attr(name).toListRef();
        auto out = std::vector<std::string>{};
        out.reserve(list.size());
        for (const auto & v : list) {
            out.push_back(v.toStringRef());
        }
        return out;
    }

  private:
    // このクラスが組み立てる観測の並び。`Rl::update` と 1:1 で対応させること。
    static auto expected_fields(int64_t obs_dim)
        -> std::vector<std::pair<std::string, int64_t>> {
        auto fields = std::vector<std::pair<std::string, int64_t>>{{"ori_err", 3}, {"gyro", 3}};
        if (obs_dim != OBS_DIM_NO_VEL) {
            fields.emplace_back("v_cmd", 3);
        }
        fields.emplace_back("prev_action", ACT_DIM);
        if (obs_dim == OBS_DIM_CAP) {
            fields.emplace_back("max_duty", 1);
        }
        return fields;
    }

    static auto describe(const std::vector<std::pair<std::string, int64_t>> & f) -> std::string {
        auto s = std::string{"["};
        for (size_t i = 0; i < f.size(); ++i) {
            s += (i ? ", " : "") + f[i].first + ":" + std::to_string(f[i].second);
        }
        return s + "]";
    }

    // 観測の組み立て順をバンドルと照合する。golden では通ってしまう領域 —
    // golden は組み立て済みの観測を再生するだけで、組み立て順は見ていない。
    //
    // 18 次元では必須。並びを取り違えて一番困るのが末尾に max_duty を足したこの次元で、
    // そこだけ穴を開けるのは本末転倒なので、照合できないなら起動させない。
    void check_obs_fields() {
        const auto expected = expected_fields(obs_dim_);
        const auto names = this->string_list_attr("obs_field_names");
        const auto widths = this->attr("obs_field_widths").toIntVector();
        if (names.empty()) {
            if (obs_dim_ == OBS_DIM_CAP) {
                throw std::runtime_error(
                    "バンドルに obs_fields がありません (" + path_ + ")。" +
                    std::to_string(OBS_DIM_CAP) +
                    " 次元では必須です (末尾 max_duty の位置を照合できないと、golden が "
                    "PASS しても方策が別の入力を読みます)。この logic の並び: " +
                    describe(expected));
            }
            warnings_.push_back(
                "バンドルに obs_fields がありません (" + path_ +
                ")。観測の並びを照合できないので、golden が PASS しても組み立て順の"
                "取り違えは検出できません。この logic の並び: " +
                describe(expected));
            return;
        }
        if (names.size() != widths.size()) {
            throw std::runtime_error("obs_fields の名前と幅の数が違います (" + path_ + ")");
        }
        auto got = std::vector<std::pair<std::string, int64_t>>{};
        auto total = int64_t{0};
        for (size_t i = 0; i < names.size(); ++i) {
            got.emplace_back(names[i], widths[i]);
            total += widths[i];
        }
        if (total != obs_dim_) {
            throw std::runtime_error(
                "obs_fields の幅の合計 " + std::to_string(total) + " がポリシーの入力次元 " +
                std::to_string(obs_dim_) + " と一致しません (" + path_ + ")");
        }
        if (got != expected) {
            throw std::runtime_error(
                "観測レイアウトが sim と食い違っています (" + path_ + ")。バンドル: " +
                describe(got) + " / この logic: " + describe(expected));
        }
    }

    static auto to_vector(const at::Tensor & t) -> std::vector<double> {
        const auto * p = t.data_ptr<double>();
        return std::vector<double>(p, p + t.numel());
    }

    std::string path_;
    std::vector<double> mean_;
    std::vector<double> var_;
    double clip_{0.0};
    double eps_{0.0};
    int64_t obs_dim_{0};
    int64_t act_dim_{0};
    std::string action_mode_;
    std::string obs_frame_;
    std::vector<std::string> warnings_;
    at::Tensor x_;                       // 推論の入力バッファ (使い回す)
    std::vector<c10::IValue> inputs_;    // forward() の引数リスト (同上、x_ を指す)
    mutable torch::jit::script::Module module_;
};

// レンチモード action (`action_mode: "modes"`) を [servo x4, esc x4] に直す。
// 6 次元のモードレートを 積分 -> ミキサ -> 折返し の 3 段で 8 次元にする。
// 係数はすべてバンドルの `action_contract` が正。ここでハードコードしない
// (sim が値を変えたら読み込みで落とす)。
//
// 状態は積分器 `m_` と前回サーボ角 `prev_servo_` の 2 つ。どちらも tick をまたいで残り、
// `reset()` (= disarm / モード切替) で 0 に戻る。
class ModeAction {
  public:
    ModeAction(const PolicyRunner & runner) {
        mode_slew_per_s_ = runner.double_attr("mode_slew_per_s");
        deadband_frac_ = runner.double_attr("deadband_frac");
        thrust_per_cmd_ = runner.double_attr("thrust_per_cmd");
        thrust_curve_exp_ = runner.double_attr("thrust_curve_exp");
        servo_range_deg_ = runner.double_attr("servo_range_deg");
        control_rate_hz_ = runner.double_attr("control_rate_hz");
        // deadband_frac も必須にする。契約に無いキーは書き出し側が 0.0 で埋めるので、
        // ここで許すと「欠けている」を「デッドバンド無効」として黙って受け入れることになり、
        // 原点近傍でサーボが atan2 の数値ノイズを追ってチャタリングする
        if (mode_slew_per_s_ <= 0.0 || thrust_per_cmd_ <= 0.0 || thrust_curve_exp_ <= 0.0 ||
            servo_range_deg_ <= 0.0 || deadband_frac_ <= 0.0) {
            throw std::runtime_error(
                "action_contract の係数が欠けています (" + runner.path() +
                ")。レンチモードのバンドルは mode_slew_per_s / deadband_frac / "
                "thrust_per_cmd / thrust_curve_exp / servo_range_deg を持っている必要が"
                "あります");
        }

        const auto names = runner.string_list_attr("mode_names");
        const auto cols = runner.string_list_attr("mode_sign_columns");
        const auto positions = runner.string_list_attr("mode_positions");
        if (static_cast<int64_t>(names.size()) != PolicyRunner::MODE_DIM) {
            throw std::runtime_error(
                "mode_names は " + std::to_string(PolicyRunner::MODE_DIM) + " 個必要です (" +
                runner.path() + ")");
        }
        {   // 列の並びは mode_names の順序を仮定せず名前で引く
            auto a = names;
            auto b = cols;
            std::sort(a.begin(), a.end());
            std::sort(b.begin(), b.end());
            if (a != b) {
                throw std::runtime_error(
                    "mode_sign_columns が mode_names と一致しません (" + runner.path() + ")");
            }
        }
        const auto signs = runner.tensor_attr("mode_signs");
        if (signs.dim() != 2 || signs.size(0) != static_cast<int64_t>(POSITIONS.size()) ||
            signs.size(1) != PolicyRunner::MODE_DIM) {
            throw std::runtime_error(
                "mode_signs の形が不正です (" + runner.path() + ")。(" +
                std::to_string(POSITIONS.size()) + ", " +
                std::to_string(PolicyRunner::MODE_DIM) + ") が必要です");
        }
        if (positions.size() != POSITIONS.size()) {
            throw std::runtime_error(
                "mode_positions のユニット数が " + std::to_string(positions.size()) +
                " です (" + runner.path() + ")。" + std::to_string(POSITIONS.size()) +
                " 個必要です");
        }

        // 符号表の行を、この logic のスラスタ順 (POSITIONS) に並べ替える。
        // バンドルの並びを暗黙に信用すると、ユニットが入れ替わってもテストが通ってしまう
        const auto * s = signs.data_ptr<double>();
        for (size_t i = 0; i < POSITIONS.size(); ++i) {
            const auto it = std::find(positions.begin(), positions.end(), POSITIONS[i]);
            if (it == positions.end()) {
                throw std::runtime_error(
                    "mode_signs にユニット " + std::string(POSITIONS[i]) + " がありません (" +
                    runner.path() + ")");
            }
            const auto row = static_cast<size_t>(std::distance(positions.begin(), it));
            for (size_t j = 0; j < 3; ++j) {
                sh_[i][j] = s[row * PolicyRunner::MODE_DIM + j];
                sv_[i][j] = s[row * PolicyRunner::MODE_DIM + 3 + j];
            }
        }
        for (size_t j = 0; j < 3; ++j) {
            h_idx_[j] = index_of(names, cols[j]);
            v_idx_[j] = index_of(names, cols[3 + j]);
        }
        this->reset();
    }

    // disarm / モード切替のたびに呼ぶ。積分器と前回サーボ角を初期状態に戻す。
    void reset() {
        m_.fill(0.0);
        prev_servo_.fill(0.0);
    }

    auto servo_range_deg() const -> double { return servo_range_deg_; }
    auto control_rate_hz() const -> double { return control_rate_hz_; }
    auto mode_slew_per_s() const -> double { return mode_slew_per_s_; }
    auto thrust_per_cmd() const -> double { return thrust_per_cmd_; }
    auto thrust_curve_exp() const -> double { return thrust_curve_exp_; }

    // モードレート -> [servo x4, esc x4] (各 [-1, 1])。
    // `max_duty` は方策が観測しているのと同じ値を渡すこと (モード 1.0 の意味を揃える)。
    auto step(const std::vector<double> & raw, double max_duty, double dt)
        -> std::array<double, static_cast<size_t>(PolicyRunner::ACT_DIM)> {
        if (static_cast<int64_t>(raw.size()) != PolicyRunner::MODE_DIM) {
            throw std::runtime_error(
                "レンチモードの action は " + std::to_string(PolicyRunner::MODE_DIM) +
                " 次元が必要です (" + std::to_string(raw.size()) + " 次元でした)");
        }
        // 1. 積分 (レート制限は方策の内側)
        for (size_t k = 0; k < static_cast<size_t>(PolicyRunner::MODE_DIM); ++k) {
            const auto a = std::clamp(raw[k], -1.0, 1.0);
            m_[k] = std::clamp(m_[k] + a * mode_slew_per_s_ * dt, -1.0, 1.0);
        }
        // 2. ミキサ
        const double f_max = thrust_per_cmd_ * std::pow(max_duty, thrust_curve_exp_);
        constexpr auto PI = M_PI;
        const double servo_range_rad = servo_range_deg_ * PI / 180.0;

        auto out = std::array<double, static_cast<size_t>(PolicyRunner::ACT_DIM)>{};
        for (size_t i = 0; i < POSITIONS.size(); ++i) {
            double h = 0.0;
            double v = 0.0;
            for (size_t j = 0; j < 3; ++j) {
                h += sh_[i][j] * m_[h_idx_[j]];
                v += sv_[i][j] * m_[v_idx_[j]];
            }
            h *= f_max;  // 接線方向の力 [N]
            v *= f_max;  // 鉛直方向の力 [N]
            // 3. 折返し: 到達できない半平面は折り返して esc を反転する
            double phi = std::atan2(v, h);
            double esc_sign = 1.0;
            if (std::abs(phi) > PI / 2.0) {
                phi -= (phi < 0.0 ? -1.0 : 1.0) * PI;
                esc_sign = -1.0;
            }
            const double mag = std::hypot(h, v);
            double u = esc_sign * std::pow(
                                      std::min(mag, f_max) / thrust_per_cmd_,
                                      1.0 / thrust_curve_exp_);
            double servo = phi / servo_range_rad;
            if (mag < deadband_frac_ * f_max) {  // 原点近傍: 前回のサーボ角を保持
                servo = prev_servo_[i];
                u = 0.0;
            }
            servo = std::clamp(servo, -1.0, 1.0);
            prev_servo_[i] = servo;
            out[i] = servo;
            out[POSITIONS.size() + i] = std::clamp(u, -1.0, 1.0);
        }
        return out;
    }

  private:
    static auto index_of(const std::vector<std::string> & v, const std::string & s) -> size_t {
        return static_cast<size_t>(std::distance(v.begin(), std::find(v.begin(), v.end(), s)));
    }

    double mode_slew_per_s_{0.0};
    double deadband_frac_{0.0};
    double thrust_per_cmd_{0.0};
    double thrust_curve_exp_{0.0};
    double servo_range_deg_{0.0};
    double control_rate_hz_{0.0};
    std::array<std::array<double, 3>, 4> sh_{};  // 接線方向の符号表 (POSITIONS の順)
    std::array<std::array<double, 3>, 4> sv_{};  // 鉛直方向の符号表 (同上)
    std::array<size_t, 3> h_idx_{};              // 接線 3 列がモードベクトルのどこを指すか
    std::array<size_t, 3> v_idx_{};              // 鉛直 3 列 (同上)
    std::array<double, static_cast<size_t>(PolicyRunner::MODE_DIM)> m_{};  // 積分器
    std::array<double, 4> prev_servo_{};  // 正規化 (±1 = ±servo_range_deg)
};

// 配備前検証の結果 (`verify_golden` の戻り値)。
struct GoldenResult {
    size_t count;
    double worst;
    bool has_mixed{false};  // レンチモードの 3 段まで検証できたか
    double mixed_worst{0.0};
};

// sim で記録した golden vectors を実機の推論経路で再生する。
// 重み・正規化統計のどちらかが sim と食い違っていれば落ちる。
//
// golden.pt は `obs` [N, obs_dim] と `act` [N, act_dim] を持つ TorchScript。
// 突き合わせるのはネットの生出力なので、それだけでは `modes` の 3 段
// (積分・ミキサ・折返し) を取り違えても PASS してしまう。そのため golden.pt は
// `mixed` [N, 8] — 学習に使った env そのもの (sim の `ModeMixer`) に同じ act を通した
// 結果 — も運ぶ。
// `mode_action` を渡すとそこまで照合する (渡さなければ生出力までで止める)。
//
// 判定閾値 1e-4 は Python の配備前検証と同じ。
inline auto verify_golden(
    const PolicyRunner & runner, const std::string & golden_path,
    ModeAction * mode_action = nullptr, double tol = 1e-4) -> GoldenResult {
    torch::jit::script::Module g;
    try {
        g = torch::jit::load(golden_path);
    } catch (const std::exception & e) {
        throw std::runtime_error("golden を読み込めません (" + golden_path + "): " + e.what());
    }
    at::Tensor obs;
    at::Tensor act;
    try {
        obs = g.attr("obs").toTensor().to(torch::kFloat64).contiguous();
        act = g.attr("act").toTensor().to(torch::kFloat64).contiguous();
    } catch (const std::exception & e) {
        throw std::runtime_error(
            "golden に obs / act がありません (" + golden_path + "): " + e.what());
    }
    if (obs.dim() != 2 || act.dim() != 2 || obs.size(0) != act.size(0)) {
        throw std::runtime_error("golden の形が不正です (" + golden_path + ")");
    }
    if (obs.size(1) != runner.obs_dim() || act.size(1) != runner.act_dim()) {
        throw std::runtime_error(
            "golden の次元がポリシーと一致しません (" + golden_path + "): obs " +
            std::to_string(obs.size(1)) + " act " + std::to_string(act.size(1)) + " / policy obs " +
            std::to_string(runner.obs_dim()) + " act " + std::to_string(runner.act_dim()));
    }

    const auto n = static_cast<size_t>(obs.size(0));
    const auto obs_dim = static_cast<size_t>(obs.size(1));
    const auto act_dim = static_cast<size_t>(act.size(1));
    const auto * op = obs.data_ptr<double>();
    const auto * ap = act.data_ptr<double>();
    auto worst = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const auto o = std::vector<double>(op + i * obs_dim, op + (i + 1) * obs_dim);
        const auto got = runner.act(o);
        for (size_t k = 0; k < act_dim; ++k) {
            worst = std::max(worst, std::abs(got[k] - ap[i * act_dim + k]));
        }
    }
    if (worst > tol) {
        throw std::runtime_error(
            "golden 検証 FAIL: max|action-golden|=" + std::to_string(worst) + " (" + golden_path +
            ")。重みか正規化統計が sim と食い違っています");
    }

    auto result = GoldenResult{n, worst, false, 0.0};
    if (mode_action == nullptr || !g.attr("has_mixed").toBool()) {
        return result;
    }
    // レンチモードの 3 段。積分器は行をまたいで持ち越すので、生成側と同じく
    // 1 本の連続した列として再生する (再生の前後で状態を 0 に戻す)
    const auto mixed = g.attr("mixed").toTensor().to(torch::kFloat64).contiguous();
    const auto max_duty = g.attr("mixed_max_duty").toDouble();
    const auto dt = g.attr("mixed_dt").toDouble();
    if (mixed.dim() != 2 || static_cast<size_t>(mixed.size(0)) != n ||
        mixed.size(1) != PolicyRunner::ACT_DIM) {
        throw std::runtime_error("golden の mixed の形が不正です (" + golden_path + ")");
    }
    const auto * mp = mixed.data_ptr<double>();
    mode_action->reset();
    auto mixed_worst = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const auto a = std::vector<double>(ap + i * act_dim, ap + (i + 1) * act_dim);
        const auto got = mode_action->step(a, max_duty, dt);
        for (size_t k = 0; k < static_cast<size_t>(PolicyRunner::ACT_DIM); ++k) {
            mixed_worst =
                std::max(mixed_worst, std::abs(got[k] - mp[i * PolicyRunner::ACT_DIM + k]));
        }
    }
    mode_action->reset();
    if (mixed_worst > tol) {
        throw std::runtime_error(
            "golden 検証 FAIL (mixed): max|mixed-golden|=" + std::to_string(mixed_worst) + " (" +
            golden_path +
            ")。積分・ミキサ・折返しのどれかが sim と食い違っています");
    }
    result.has_mixed = true;
    result.mixed_worst = mixed_worst;
    return result;
}

// qb を qa へ持っていく回転ベクトル。MuJoCo の `mju_subQuat` 相当。
// MuJoCo の numpy 再実装と乱数 2000 組で照合し、最大誤差 4.4e-16 (倍精度の丸め 1-2 ulp)。
// bit 一致ではないので、golden の判定閾値をこれより厳しくしないこと。
inline auto sub_quat(
    const std::array<double, 4> & qa, const std::array<double, 4> & qb) -> std::array<double, 3> {
    // qb の共役 (単位クォータニオン前提)
    const std::array<double, 4> qn{qb[0], -qb[1], -qb[2], -qb[3]};
    const std::array<double, 4> qd{
        qn[0] * qa[0] - qn[1] * qa[1] - qn[2] * qa[2] - qn[3] * qa[3],
        qn[0] * qa[1] + qn[1] * qa[0] + qn[2] * qa[3] - qn[3] * qa[2],
        qn[0] * qa[2] - qn[1] * qa[3] + qn[2] * qa[0] + qn[3] * qa[1],
        qn[0] * qa[3] + qn[1] * qa[2] - qn[2] * qa[1] + qn[3] * qa[0]};
    const double sin_a_2 = std::sqrt(qd[1] * qd[1] + qd[2] * qd[2] + qd[3] * qd[3]);
    if (sin_a_2 < 1e-12) {
        return {0.0, 0.0, 0.0};
    }
    double speed = 2.0 * std::atan2(sin_a_2, qd[0]);
    if (speed > M_PI) {  // pi を超える回転は逆向きが最短
        speed -= 2.0 * M_PI;
    }
    const double k = speed / sin_a_2;
    return {qd[1] * k, qd[2] * k, qd[3] * k};
}

// IMU の姿勢を (w, x, y, z) に並べ替えて正規化する。
inline auto to_unit_quat(const state::imu::Quaternion & q) -> std::array<double, 4> {
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1e-9) {
        return {1.0, 0.0, 0.0, 0.0};  // 未受信 / 異常値: 単位姿勢として扱う
    }
    return {q.w / n, q.x / n, q.y / n, q.z / n};
}

// 目標姿勢の解釈。`ff` は `target_orientation` をトルク配分の入力として使うが、
// `rl` は 回転ベクトル [rad] (REP-103、ワールド基準) として読む。
// 大きさが回転角、向きが回転軸。ゼロは水平・機首方位維持。
inline auto target_quat_from(const cmd::attitude::Orientation & o) -> std::array<double, 4> {
    const double theta = std::sqrt(o.x * o.x + o.y * o.y + o.z * o.z);
    if (theta < 1e-12) {
        return {1.0, 0.0, 0.0, 0.0};
    }
    const double s = std::sin(theta / 2.0) / theta;
    return {std::cos(theta / 2.0), o.x * s, o.y * s, o.z * s};
}

// current を target へ、1 ステップあたり最大 `max_rate * dt` だけ近づける。
// `max_rate` が 0 以下なら制限しない。
inline auto slew(double current, double target, double max_rate, double dt) -> double {
    if (max_rate <= 0.0) {
        return target;
    }
    const double step = max_rate * dt;
    return current + std::clamp(target - current, -step, step);
}

class Rl : public AttitudeController::Logic {
  public:
    // 18 次元ポリシーの学習時 duty 上限の分布。観測に入れる値だけここへ丸める
    // (duty のクリップ自体はオペレータの max_duty のまま)。
    static constexpr double MAX_DUTY_OBS_MIN = 0.2;
    static constexpr double MAX_DUTY_OBS_MAX = 0.4;

    struct Options {
        std::string model_path;   // deploy.pt (必須)
        std::string golden_path;  // "" なら配備前検証をスキップ
        double max_duty;
        double servo_range_deg;
        double servo_slew_deg_per_s;
        double thrust_slew_per_s;
        bool hold_yaw;
        double control_hz;  // 0 なら学習時レートとの照合をしない
    };

    // バンドルを読み、配備前検証を通してから使える状態にする。
    // 失敗は例外。呼び出し側 (`on_configure`) で拾って ERROR にすること —
    // 検証を通っていない方策でスラスタを回さない。
    explicit Rl(const Options & opt) : opt_(opt), runner_(opt.model_path) {
        report_ = "policy loaded from " + opt.model_path + " (obs " +
                  std::to_string(runner_.obs_dim()) + "-D, act " +
                  std::to_string(runner_.act_dim()) + "-D, action_mode=" + runner_.action_mode() +
                  ", rep103)";
        for (const auto & w : runner_.warnings()) {
            report_ += "\n  [warn] " + w;
        }

        if (runner_.action_mode() == "modes") {
            mode_action_ = std::make_unique<ModeAction>(runner_);
            // サーボ範囲がずれるとミキサの正規化と出力側の逆正規化が食い違い、
            // 角度が別物になる。黙って丸めない
            if (std::abs(mode_action_->servo_range_deg() - opt_.servo_range_deg) > 1e-6) {
                throw std::runtime_error(
                    "servo_range_deg がパラメータ (" + std::to_string(opt_.servo_range_deg) +
                    ") と契約 (" + std::to_string(mode_action_->servo_range_deg()) +
                    ") で食い違っています (" + opt.model_path + ")");
            }
            report_ += "\n  action_mode=modes: モードレートを積分してミキサに通します (slew " +
                       std::to_string(mode_action_->mode_slew_per_s()) + "/s, f_max = " +
                       std::to_string(mode_action_->thrust_per_cmd()) + " * max_duty^" +
                       std::to_string(mode_action_->thrust_curve_exp()) + ")";
            // モードの積分は実測 dt で行うので発散はしないが、応答は学習時と変わる
            if (opt_.control_hz > 0.0 && mode_action_->control_rate_hz() > 0.0 &&
                std::abs(mode_action_->control_rate_hz() - opt_.control_hz) > 1e-6) {
                report_ += "\n  [warn] update_rate " + std::to_string(opt_.control_hz) +
                           " Hz は学習時の " + std::to_string(mode_action_->control_rate_hz()) +
                           " Hz と違います";
            }
        }
        if (runner_.needs_max_duty()) {
            report_ += "\n  duty 上限を観測に持つポリシーです (観測末尾 max_duty=" +
                       std::to_string(this->obs_max_duty()) + ")";
            if (opt_.max_duty < MAX_DUTY_OBS_MIN || opt_.max_duty > MAX_DUTY_OBS_MAX) {
                // 観測に入る値はクランプするので壊れはしないが、オペレータの意図と
                // 実挙動がずれる (0.5 に上げても方策は 0.4 のつもりで指令を作る)
                report_ += "\n  [warn] max_duty=" + std::to_string(opt_.max_duty) +
                           " は学習分布 [" + std::to_string(MAX_DUTY_OBS_MIN) + ", " +
                           std::to_string(MAX_DUTY_OBS_MAX) + "] の外です。観測に入れる値は " +
                           std::to_string(this->obs_max_duty()) +
                           " にクランプします (duty のクリップ自体は設定値のまま)";
            }
        }

        if (!opt.golden_path.empty()) {
            const auto g = verify_golden(runner_, opt.golden_path, mode_action_.get());
            report_ += "\n  golden 検証 PASS: " + std::to_string(g.count) + " vectors, max err " +
                       std::to_string(g.worst);
            if (g.has_mixed) {
                report_ += " / mixed (積分・ミキサ・折返し) max err " +
                           std::to_string(g.mixed_worst);
            } else if (mode_action_) {
                report_ +=
                    "\n  [warn] golden に mixed がないので 3 段の変換は未検証です "
                    "(export_deploy_bundle.py を流し直してください)";
            }
        } else {
            report_ += "\n  [warn] golden が無いので配備前検証をスキップしました";
        }
        this->reset();
    }

    // 読み込みと検証の結果。`on_configure` がそのままログに出す。
    auto report() const -> const std::string & { return report_; }

    auto control_mode() const -> logic::ControlMode override { return logic::ControlMode::Rl; }

    // モードに入った瞬間は出力をゼロにし、方策の内部状態も初期化する。
    // `prev_action` は「自分が直前に出した指令」なので、指令を出していない間の値を
    // 残すと最初の観測が実際とずれる。レンチモードの積分器も同じ理由で戻す。
    auto init(
        double /*time*/, const AttitudeController::Input & /*input*/,
        const AttitudeController::Output & /*output*/) -> AttitudeController::Output override {
        this->reset();
        return AttitudeController::Output{};
    }

    auto update(double /*time*/, double duration, const AttitudeController::Input & input)
        -> AttitudeController::Output override {
        const auto q_imu = to_unit_quat(input.state.imu_quaternion);
        const auto q_target = target_quat_from(input.cmd.target_orientation);
        auto ori_err = sub_quat(q_target, q_imu);
        if (!opt_.hold_yaw) {
            // yaw 成分を落とす = その軸まわりの姿勢誤差を 0 として扱う。回転ベクトルの
            // 成分を落とすだけなので特異点が無い (RPY に直すと pitch±90 で破綻する)
            ori_err[YAW_IDX] = 0.0;
        }

        auto obs = std::vector<double>{};
        obs.reserve(static_cast<size_t>(runner_.obs_dim()));
        obs.insert(obs.end(), ori_err.begin(), ori_err.end());
        // IMU の gyro は軸変換せずそのまま入れる。ずれていたら IMU ドライバ側 (AXIS_MAP)
        // を直す (autonomy known_issues A-13)
        obs.push_back(input.state.imu_angular_velocity.x);
        obs.push_back(input.state.imu_angular_velocity.y);
        obs.push_back(input.state.imu_angular_velocity.z);
        if (runner_.needs_velocity()) {  // attitude タスクだけが速度指令を持たない
            obs.push_back(input.cmd.target_velocity.x);
            obs.push_back(input.cmd.target_velocity.y);
            obs.push_back(input.cmd.target_velocity.z);
        }
        obs.insert(obs.end(), prev_action_.begin(), prev_action_.end());
        if (runner_.needs_max_duty()) {
            obs.push_back(this->obs_max_duty());
        }

        const auto raw = runner_.act(obs);
        // レンチモードなら 6 次元のレートを 8 次元に直す。max_duty は方策が観測しているのと
        // 同じ値を渡すこと (モード 1.0 の意味を揃える)
        const auto action = mode_action_
                                ? mode_action_->step(raw, this->obs_max_duty(), duration)
                                : to_action(raw);
        // 観測に返すのは (レンチモードならミックス後の) 8 次元。レート制限は掛けない前の値
        prev_action_ = action;

        auto output = AttitudeController::Output{};
        for (size_t i = 0; i < THRUSTER_NUM; ++i) {
            const double servo_target = action[i] * opt_.servo_range_deg;
            const double duty_target =
                std::clamp(action[THRUSTER_NUM + i], -opt_.max_duty, opt_.max_duty);
            servo_cmd_[i] = slew(servo_cmd_[i], servo_target, opt_.servo_slew_deg_per_s, duration);
            duty_cmd_[i] = slew(duty_cmd_[i], duty_target, opt_.thrust_slew_per_s, duration);
            // 範囲外は CAN 送信が失敗するので ±90 deg に収める (autonomy known_issues B-13)
            output.cmd.servo_angles[i].value = std::clamp(servo_cmd_[i], -90.0, 90.0);
            output.cmd.esc_thrusts[i].value = duty_cmd_[i];
        }
        return output;
    }

  private:
    using Action = std::array<double, static_cast<size_t>(PolicyRunner::ACT_DIM)>;

    // REP-103 (x前/y左/z上) では yaw は z 軸まわり
    static constexpr size_t YAW_IDX = 2;
    static constexpr size_t THRUSTER_NUM = 4;

    static auto to_action(const std::vector<double> & raw) -> Action {
        auto a = Action{};
        std::copy(raw.begin(), raw.end(), a.begin());
        return a;
    }

    // 方策が観測する duty 上限。ミキサにも同じ値を渡す。
    auto obs_max_duty() const -> double {
        return std::clamp(opt_.max_duty, MAX_DUTY_OBS_MIN, MAX_DUTY_OBS_MAX);
    }

    void reset() {
        prev_action_.fill(0.0);
        servo_cmd_.fill(0.0);
        duty_cmd_.fill(0.0);
        if (mode_action_) {
            mode_action_->reset();
        }
    }

    Options opt_;
    PolicyRunner runner_;
    std::unique_ptr<ModeAction> mode_action_;  // action_mode="modes" のときだけ
    std::string report_;
    Action prev_action_{};
    std::array<double, THRUSTER_NUM> servo_cmd_{};  // レート制限後の「いま出している指令」[deg]
    std::array<double, THRUSTER_NUM> duty_cmd_{};   // 同上 [-1, 1]
};

}  // namespace sinsei_umiusi_control::controller::logic::attitude

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_RL_HPP
