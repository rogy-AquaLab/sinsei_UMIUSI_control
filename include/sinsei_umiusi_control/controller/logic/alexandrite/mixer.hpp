#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ALEXANDRITE_MIXER_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ALEXANDRITE_MIXER_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

// Alexandriteの3基のアジマススラスタへ、目標の力・モーメントを割り振る
//
// 機体座標は x = 前、y = 左、z = 上。スラスタはUMIUSIと同じく左前から反時計回りに数える。
//
//   thruster1: 左前 ( front_x,  front_y)  水平推力の正方向 (-1, 1) / √2
//   thruster2: 後ろ (-rear_x,   0      )  水平推力の正方向 ( 0, -1)
//   thruster3: 右前 ( front_x, -front_y)  水平推力の正方向 ( 1, 1) / √2
//
// 前側2基はUMIUSIと同じ45度配置。どのスラスタも、サーボ角0・正のDutyでz軸まわり反時計回り
// （+yaw）のモーメントを生む向きを正とする。サーボ角θのとき、推力の向きは
// cosθ・(水平推力の正方向) + sinθ・(+z) になる。

namespace sinsei_umiusi_control::controller::logic::alexandrite {

constexpr std::size_t THRUSTER_COUNT = 3;

// スラスタの取り付け位置。長さの比だけを使うため、単位は何でもよい
struct Geometry {
    // 重心から前側2基までの前後方向の距離
    double front_x;
    // 重心から前側2基までの左右方向の距離
    double front_y;
    // 重心から後ろの1基までの前後方向の距離
    double rear_x;
};

// 目標の力（fx, fy, fz）とモーメント（mx = roll, my = pitch, mz = yaw）
struct Wrench {
    double fx;
    double fy;
    double fz;
    double mx;
    double my;
    double mz;
};

struct ThrusterOutput {
    // -max_duty〜max_duty
    double duty;
    // サーボ角 [rad]（-pi/2〜pi/2）
    double angle;
};

namespace detail {

inline auto to_output(const double horizontal, const double vertical) -> ThrusterOutput {
    if (horizontal == 0.0) {
        // 真上か真下へ向ける。-0.0で割ると向きが反転するため、垂直成分の符号で決める
        return {
            std::abs(vertical), vertical == 0.0 ? 0.0 : std::copysign(std::acos(0.0), vertical)};
    }
    // 水平成分の符号をDutyの符号とし、サーボ角は-pi/2〜pi/2に収める
    const auto sign = horizontal < 0.0 ? -1.0 : 1.0;
    return {sign * std::hypot(horizontal, vertical), std::atan(vertical / horizontal)};
}

}  // namespace detail

// どれかのDutyがmax_dutyを超える場合は、力・モーメントの向きを保つよう全基を同じ比率で縮める
inline auto mix(const Wrench & target, const Geometry & geometry, const double max_duty)
    -> std::array<ThrusterOutput, THRUSTER_COUNT> {
    // 長さの比だけを使うため、前側2基までの距離が1になるよう正規化する
    const auto scale = std::hypot(geometry.front_x, geometry.front_y);
    const auto a = geometry.front_x / scale;
    const auto b = geometry.front_y / scale;
    const auto c = geometry.rear_x / scale;
    const auto sqrt2 = std::sqrt(2.0);

    // 水平成分: fx, fy, mz から f1h, f2h, f3h を解く
    //   fx = (f3h - f1h) / √2
    //   fy = (f1h + f3h) / √2 - f2h
    //   mz = (a + b) (f1h + f3h) / √2 + c f2h
    const auto front_sum = sqrt2 * (target.mz + c * target.fy) / (a + b + c);
    const auto front_diff = sqrt2 * target.fx;
    const auto f1h = (front_sum - front_diff) / 2.0;
    const auto f3h = (front_sum + front_diff) / 2.0;
    const auto f2h = (target.mz - (a + b) * target.fy) / (a + b + c);

    // 垂直成分: fz, mx, my から f1v, f2v, f3v を解く
    //   fz = f1v + f2v + f3v
    //   mx = b (f1v - f3v)
    //   my = c f2v - a (f1v + f3v)
    const auto f2v = (target.my + a * target.fz) / (a + c);
    const auto front_vertical_sum = (c * target.fz - target.my) / (a + c);
    const auto front_vertical_diff = target.mx / b;
    const auto f1v = (front_vertical_sum + front_vertical_diff) / 2.0;
    const auto f3v = (front_vertical_sum - front_vertical_diff) / 2.0;

    auto outputs = std::array<ThrusterOutput, THRUSTER_COUNT>{
        detail::to_output(f1h, f1v),
        detail::to_output(f2h, f2v),
        detail::to_output(f3h, f3v),
    };

    auto max_abs = 0.0;
    for (const auto & output : outputs) {
        max_abs = std::max(max_abs, std::abs(output.duty));
    }
    if (max_abs > max_duty) {
        const auto ratio = max_duty / max_abs;
        for (auto & output : outputs) {
            output.duty *= ratio;
        }
    }
    return outputs;
}

}  // namespace sinsei_umiusi_control::controller::logic::alexandrite

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ALEXANDRITE_MIXER_HPP
