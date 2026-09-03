#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_RL_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_RL_HPP

/// 学習済み方策で姿勢を制御する logic。`control_mode:=rl` で選ぶ。
///
/// 観測の並びは方策のバンドルが決める (`export/meta.json` の `obs_fields`)。ここで
/// 組み立てるのは 17 次元:
///   [ori_err(3), gyro(3), v_cmd(3), prev_action(8)]
///
/// 守ること:
///   * 観測の順序と正規化はバンドルと厳密に合わせる。ずれても golden 以外では気付けない
///   * `prev_action` は「自分が直前に出した指令」。init() でゼロに戻す
///   * スレッド数は 1 に固定する。他のノードと CPU を奪い合うと多スレッドは逆効果
///     (autonomy の docs/performance_tuning.md、検出器で実測)
///
/// 出力は推力 [N] とサーボ角 [deg]。方策は duty で学習しているので、
/// `F = |u|^thrust_curve_exp * thrust_per_cmd` で推力に直してから返す。係数はバンドルの
/// `action_contract` が正。**ここでハードコードしない** (sim 側が変えたら読み込みで落とす)。

#include <ATen/Parallel.h>
#include <torch/script.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "sinsei_umiusi_control/controller/attitude_controller.hpp"

namespace sinsei_umiusi_control::controller::logic::attitude {

/// 方策の推論。正規化 -> 推論 -> クリップ を Python の `policy_infer.PolicyRunner` と
/// 同じ順序で行う。正規化を float64 で計算してから float32 に落とすところまで揃えること —
/// float32 で計算すると golden がわずかにずれる。
class PolicyRunner {
  public:
    PolicyRunner(
        const std::string & ts_path, std::vector<double> mean, std::vector<double> var,
        double clip, double eps)
    : mean_(std::move(mean)), var_(std::move(var)), clip_(clip), eps_(eps) {
        if (mean_.size() != var_.size()) {
            throw std::runtime_error("obs_norm の mean と var の次元が違います");
        }
        at::set_num_threads(1);
        module_ = torch::jit::load(ts_path);
        module_.eval();
        obs_dim_ = static_cast<int64_t>(mean_.size());
    }

    auto obs_dim() const -> int64_t { return obs_dim_; }

    auto act(const std::vector<double> & obs) const -> std::vector<double> {
        if (static_cast<int64_t>(obs.size()) != obs_dim_) {
            throw std::runtime_error(
                "観測の次元が " + std::to_string(obs.size()) + " です (" +
                std::to_string(obs_dim_) + " が必要)");
        }
        auto x = torch::empty({1, obs_dim_}, torch::kFloat32);
        auto * p = x.data_ptr<float>();
        for (int64_t i = 0; i < obs_dim_; ++i) {
            const double n = (obs[i] - mean_[i]) / std::sqrt(var_[i] + eps_);
            p[i] = static_cast<float>(std::clamp(n, -clip_, clip_));
        }
        torch::NoGradGuard no_grad;
        const auto y = module_.forward({x}).toTensor().squeeze(0).contiguous();
        const auto * q = y.data_ptr<float>();
        std::vector<double> out(static_cast<size_t>(y.numel()));
        for (size_t i = 0; i < out.size(); ++i) {
            out[i] = std::clamp(static_cast<double>(q[i]), -1.0, 1.0);
        }
        return out;
    }

  private:
    std::vector<double> mean_;
    std::vector<double> var_;
    double clip_;
    double eps_;
    int64_t obs_dim_{0};
    mutable torch::jit::script::Module module_;
};

/// qb を qa へ持っていく回転ベクトル。MuJoCo の `mju_subQuat` 相当。
/// autonomy の numpy 実装と乱数 2000 組で照合し、最大誤差 4.4e-16 (倍精度の丸め 1-2 ulp)。
/// bit 一致ではないので、golden の判定閾値をこれより厳しくしないこと。
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

}  // namespace sinsei_umiusi_control::controller::logic::attitude

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_RL_HPP
