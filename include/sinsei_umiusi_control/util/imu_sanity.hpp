#ifndef SINSEI_UMIUSI_CONTROL_UTIL_IMU_SANITY_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_IMU_SANITY_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>

// IMU の化けサンプルを判定する。ROS 非依存。
// 化けの種類・実測値・既定を enforce=false にした理由は autonomy docs/known_issues.md A-1。
// autonomy の umiusi_common/imu_sanity.py と同じ規約で動かすこと (両スタックの run を
// 突き合わせるため)。パラメータ名・既定値・stale の意味を勝手にずらさない。
namespace sinsei_umiusi_control::util {

// 2 つの単位クォータニオン間の回転角 [rad]。符号の曖昧さ (q と -q は同じ姿勢) を吸収する。
inline auto quat_angle_between(
    const std::array<double, 4> & qa, const std::array<double, 4> & qb) -> double {
    auto dot = 0.0;
    for (size_t i = 0; i < 4; ++i) {
        dot += qa[i] * qb[i];
    }
    return 2.0 * std::acos(std::min(1.0, std::abs(dot)));
}

// 検査を通った IMU 値。quat は (w, x, y, z) の正規化済みクォータニオン。
struct ImuSample {
    std::array<double, 4> quat;
    std::array<double, 3> gyro;
};

class ImuSanity {
  public:
    struct Options {
        // ROV が実際に出しうる値より十分大きく、フルスケール (35.74) より十分小さく取る
        double max_gyro = 10.0;       // [rad/s]
        double max_step_deg = 30.0;   // 1 サンプル間の姿勢跳躍の上限。50 Hz なら 1500 deg/s 相当
        double quat_tol = 0.01;       // ノルムの許容誤差
        int stale_after = 5;          // 連続棄却がこれを超えたら跳躍チェックを解除する
        bool enforce = false;         // false なら検出だけして値は通す (既定)
    };

    // 判定に引っかかった理由の種別。ログの文言ではなくこれで分岐すること
    // (文言を変えたら挙動が変わる、を避ける)。
    enum class Reason {
        None,
        NotFinite,      // NaN / Inf
        Unnormalizable, // ノルムが 0 近傍。正規化が 0 除算になる
        BadNorm,        // ノルムが 1 から外れている
        GyroOverLimit,
        AttitudeStep,
    };

    struct Result {
        std::optional<ImuSample> sample;  // 採用値、または直前の有効値。まだ無ければ nullopt
        Reason reason = Reason::None;
        // 実際に棄却して直前の有効値を返したか。false なら (検出したかどうかに関わらず)
        // 呼び出し側は生値をそのまま使ってよい。enforce=false で生データを録り続けるために
        // 要る — ここで正規化した値を返すと、bag から |q| の化けが見えなくなる
        bool held = false;
        std::string detail;  // ログ用。実測値を含む
    };

    // Options はネストクラスなので、既定引数に `Options{}` は書けない
    // (NSDMI が囲むクラスの終わりまで完成しない)。既定構築は defaulted ctor で行う
    ImuSanity() = default;
    explicit ImuSanity(const Options & opt) : opt_(opt) {}

    // enforce に関係なく必ず捨てる理由。閾値を緩めても救えない、値そのものが数値として
    // 使えないものだけ。
    static auto unusable(Reason r) -> bool {
        return r == Reason::NotFinite || r == Reason::Unnormalizable;
    }

    auto stale() const -> bool { return consecutive_ > opt_.stale_after; }
    auto accepted() const -> size_t { return accepted_; }
    auto rejected() const -> size_t { return rejected_; }
    auto flagged() const -> size_t { return flagged_; }
    auto resyncs() const -> size_t { return resyncs_; }
    auto reject_ratio() const -> double {
        const auto total = accepted_ + rejected_;
        return total ? static_cast<double>(rejected_) / static_cast<double>(total) : 0.0;
    }
    auto flag_ratio() const -> double {
        const auto total = accepted_ + rejected_;
        return total ? static_cast<double>(flagged_) / static_cast<double>(total) : 0.0;
    }

    auto update(const std::array<double, 4> & quat_wxyz, const std::array<double, 3> & gyro_xyz)
        -> Result {
        auto res = Result{};
        res.reason = this->check(quat_wxyz, gyro_xyz, res.detail);

        if (res.reason != Reason::None) {
            ++flagged_;
            if (opt_.enforce || unusable(res.reason)) {
                ++rejected_;
                ++consecutive_;
                res.held = true;
                res.sample = last_;
                return res;
            }
        }

        if (this->stale()) {
            ++resyncs_;
        }
        const auto n = norm(quat_wxyz);
        last_ = ImuSample{
            {quat_wxyz[0] / n, quat_wxyz[1] / n, quat_wxyz[2] / n, quat_wxyz[3] / n}, gyro_xyz};
        ++accepted_;
        consecutive_ = 0;
        // enforce=false のときは reason を付けたまま値を通す。呼び出し側が
        // 「検出したが通した」ことをログに出せるようにするため
        res.sample = last_;
        return res;
    }

    // ログ 1 行。enforce の有無で「破棄した」のか「通した」のかが変わる。
    auto describe(const Result & r) const -> std::string {
        if (opt_.enforce || unusable(r.reason)) {
            return "IMU サンプルを破棄: " + r.detail + " (棄却率 " + ratio_str(reject_ratio()) +
                   ")";
        }
        return "IMU の異常サンプルを検出 (破棄していません): " + r.detail + " (検出率 " +
               ratio_str(flag_ratio()) + ")";
    }

  private:
    // 正規化が定義できないノルムの下限。閾値ではなく数値上の限界なので設定にしない。
    static constexpr double MIN_NORM = 1e-6;
    // BNO055 の角速度フルスケール [rad/s]。これ付近の値は読み出し化けとみなす。
    static constexpr double GYRO_FULL_SCALE = 35.74;

    static auto norm(const std::array<double, 4> & q) -> double {
        return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    }

    static auto ratio_str(double r) -> std::string {
        return std::to_string(r * 100.0).substr(0, 5) + "%";
    }

    auto check(
        const std::array<double, 4> & q, const std::array<double, 3> & g,
        std::string & detail) const -> Reason {
        for (const auto v : {q[0], q[1], q[2], q[3], g[0], g[1], g[2]}) {
            if (!std::isfinite(v)) {
                detail = "NaN/Inf が含まれる";
                return Reason::NotFinite;
            }
        }

        const auto n = norm(q);
        if (n < MIN_NORM) {
            detail = "正規化できない (|q|=" + std::to_string(n) + ")";
            return Reason::Unnormalizable;
        }
        if (std::abs(n - 1.0) > opt_.quat_tol) {
            detail = "クォータニオンのノルムが不正 (|q|=" + std::to_string(n) + ")";
            return Reason::BadNorm;
        }

        const auto gmax = std::max({std::abs(g[0]), std::abs(g[1]), std::abs(g[2])});
        if (gmax > opt_.max_gyro) {
            detail = "角速度が上限超過 (" + std::to_string(gmax) + " rad/s)";
            if (std::abs(gmax - GYRO_FULL_SCALE) < 1.0) {
                detail += " — int16 フルスケール相当の化け";
            }
            return Reason::GyroOverLimit;
        }

        // stale の間は跳躍チェックを止める。姿勢基準そのものが飛ぶと、飛ぶ前の値と
        // 比べ続ける限り永久に復帰できない (known_issues A-1 で実際に 144 秒棄却し続けた)
        if (last_ && !this->stale()) {
            const auto unit = std::array<double, 4>{q[0] / n, q[1] / n, q[2] / n, q[3] / n};
            const auto step = quat_angle_between(last_->quat, unit);
            if (step > opt_.max_step_deg * M_PI / 180.0) {
                detail = "姿勢が急変 (" + std::to_string(step * 180.0 / M_PI) + " deg/sample)";
                return Reason::AttitudeStep;
            }
        }
        return Reason::None;
    }

    Options opt_;
    std::optional<ImuSample> last_;
    size_t accepted_{0};
    size_t rejected_{0};
    size_t flagged_{0};
    size_t resyncs_{0};
    int consecutive_{0};
};

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_IMU_SANITY_HPP
