# `AttitudeController`の制御

## ゲイン設定

姿勢制御ゲインは`attitude_controller.ros__parameters`で設定する。

| モード | パラメータ | 既定値 | 用途 |
| --- | --- | ---: | --- |
| FF | `feed_forward.k_attitude` | 2.0 | roll/pitch目標姿勢から操作量への変換ゲイン |
| FF | `feed_forward.k_yaw_rate` | 0.2 | yawレート目標から操作量への変換ゲイン |
| FB | `feedback.kp_roll`, `feedback.kp_pitch` | 1.0 | roll/pitch姿勢誤差の比例ゲイン |
| FB | `feedback.kd_roll`, `feedback.kd_pitch` | 0.35 | roll/pitch角速度の減衰ゲイン |
| FB | `feedback.kp_yaw_rate` | 1.0 | yawレート誤差の比例ゲイン |
| FB | `feedback.ki_roll`, `feedback.ki_pitch` | 0.0 | roll/pitch姿勢誤差の積分ゲイン |
| FB | `feedback.i_max` | 0.2 | roll/pitch積分誤差の絶対値上限 |

値はconfigure時に読み込むため、変更後は`attitude_controller`を再configureする。

## ミキサー設定

| パラメータ | 既定値 | 用途 |
| --- | ---: | --- |
| `mixer.servo_direction_deadband_deg` | 5.0 | この角度以内の推力方向の変化ではサーボを動かさない |
| `mixer.servo_reversal_deadband_deg` | 10.0 | ±90 deg の境界付近で反対側の端へ回さない幅 |
| `mixer.servo_retarget_thrust_enter` / `exit` | 0.10 / 0.06 | サーボが推力方向へ追従し始める / やめる正規化推力 |
| `mixer.esc_thrust_limit` | 1.0（yaml 0.5） | ESC 推力の上限。`max_duty / duty_per_thrust` に合わせる。超えるときは姿勢モーメントを優先して並進を一様に縮める |

## `logic::attitude::FeedForward`

### TL;DR

一番シンプルなフィードフォワード制御。将来的にフィードバック制御で置き換えたい。

入力(`target_orientation`, `target_velocity`)から線形変換により出力(各スラスタへの命令: `angle`, `duty_cycle`)を得る。

### Detail

以下のように文字を定義する。

- Input
    - `target_orientation`: $\Phi^\text{ref}_x$, $\Phi^\text{ref}_y$, $\Phi^\text{ref}_z$
    - `target_velocity`: $V^\text{ref}_x$, $V^\text{ref}_y$, $V^\text{ref}_z$
- Output
    - `thruster_lf/*`(スラスタ左前): $\phi_1, f_1$
    - `thruster_lb/*`(スラスタ左後): $\phi_2, f_2$
    - `thruster_rb/*`(スラスタ右後): $\phi_3, f_3$
    - `thruster_rf/*`(スラスタ右前): $\phi_4, f_4$

```math
U := \begin{bmatrix}
        \Phi^\text{ref}_x \\
        \Phi^\text{ref}_y \\
        \Phi^\text{ref}_z \\
        V^\text{ref}_x    \\
        V^\text{ref}_y    \\
        V^\text{ref}_z
     \end{bmatrix}, \quad
Y := \begin{bmatrix}
        f_{1\text{h}} \\
        f_{1\text{v}} \\
        f_{2\text{h}} \\
        f_{2\text{v}} \\
        f_{3\text{h}} \\
        f_{3\text{v}} \\
        f_{4\text{h}} \\
        f_{4\text{v}}
     \end{bmatrix}, \quad
\begin{cases}
    \phi_i = \text{atan}(f_{i\text{v}} / f_{i\text{h}}) \\
    f_i = \begin{cases}
                \frac{\sqrt{f_{i\text{h}}^2 + f_{i\text{v}}^2}}{\sqrt{2}} &\text{if} \quad f_{i\text{h}} \geq 0 \\
                -\frac{\sqrt{f_{i\text{h}}^2 + f_{i\text{v}}^2}}{\sqrt{2}} &\text{if} \quad f_{i\text{h}} < 0
          \end{cases}
\end{cases}
```

このとき以下の行列 $A$ によって $U$ から $Y = AU$ を得る。

```math
A = \begin{bmatrix}
        0  & 0  & 1 & -\sqrt{2} & \sqrt{2}  & 0 \\
        1  & -1 & 0 & 0         & 0         & 1 \\
        0  & 0  & 1 & -\sqrt{2} & -\sqrt{2} & 0 \\
        1  & 1  & 0 & 0         & 0         & 1 \\
        0  & 0  & 1 & \sqrt{2}  & -\sqrt{2} & 0 \\
        -1 & 1  & 0 & 0         & 0         & 1 \\
        0  & 0  & 1 & \sqrt{2}  & \sqrt{2}  & 0 \\
        -1 & -1 & 0 & 0         & 0         & 0
    \end{bmatrix}
```

まとめると、

```math
\begin{cases}
    \phi_i = \text{atan}(f_{i\text{v}} / f_{i\text{h}}) \\
    f_i = \begin{cases}
                \frac{\sqrt{f_{i\text{h}}^2 + f_{i\text{v}}^2}}{\sqrt{2}} &\text{if} \quad f_{i\text{h}} \geq 0 \\
                -\frac{\sqrt{f_{i\text{h}}^2 + f_{i\text{v}}^2}}{\sqrt{2}} &\text{if} \quad f_{i\text{h}} < 0
          \end{cases}
\end{cases}
```

```math
\begin{bmatrix}
    f_{1\text{h}} \\
    f_{1\text{v}} \\
    f_{2\text{h}} \\
    f_{2\text{v}} \\
    f_{3\text{h}} \\
    f_{3\text{v}} \\
    f_{4\text{h}} \\
    f_{4\text{v}}
\end{bmatrix}
=
\begin{bmatrix}
    0  & 0  & 1 & -\sqrt{2} & \sqrt{2}  & 0 \\
    1  & -1 & 0 & 0         & 0         & 1 \\
    0  & 0  & 1 & -\sqrt{2} & -\sqrt{2} & 0 \\
    1  & 1  & 0 & 0         & 0         & 1 \\
    0  & 0  & 1 & \sqrt{2}  & -\sqrt{2} & 0 \\
    -1 & 1  & 0 & 0         & 0         & 1 \\
    0  & 0  & 1 & \sqrt{2}  & \sqrt{2}  & 0 \\
    -1 & -1 & 0 & 0         & 0         & 0
\end{bmatrix}
\begin{bmatrix}
    \Phi^\text{ref}_x \\
    \Phi^\text{ref}_y \\
    \Phi^\text{ref}_z \\
    V^\text{ref}_x    \\
    V^\text{ref}_y    \\
    V^\text{ref}_z
\end{bmatrix}
```

---

## `logic::attitude::FeedBack` の yaw

yaw は既定でレート制御（`AttitudeTarget.yaw_rate` [rad/s]）。目標 quaternion の yaw は無視する。
`AttitudeTarget.hold_yaw` が true の間は、その外側に方位保持が乗る。

| `hold_yaw` | 挙動 |
|---|---|
| `false` | レート制御。状態を持たない |
| `false` → `true` | その時点の実測方位をラッチする |
| `true` | ラッチ方位を保つ。`yaw_rate` はラッチ方位を回す（先行は `yaw_hold_max_lead` まで） |
| `true` → `false` | ラッチを捨てる |

```
commanded_rate = yaw_rate + kp_yaw_hold * wrap(latched - heading)   (hold_yaw のとき)
yaw モーメント  = kp_yaw_rate * (commanded_rate - omega_z)
```

- 方位誤差が `yaw_hold_relatch_error`（90 deg）を超えたら、IMU の方位が飛んだとみなして現在方位へ
  ラッチし直す。BNO055 の方位は磁気基準で、yaw だけが跳ぶ（`sinsei_UMIUSI_autonomy` の
  `docs/known_issues.md` A-1）
- 先行の上限は slew だけに掛かる。外乱で開いた誤差は削らない
- 積分項とラッチは `FeedBack::init()` で捨てる
- `kp_yaw_hold` / `yaw_hold_max_lead` / `yaw_hold_relatch_error` は実機未検証。パラメータ化はしていない
