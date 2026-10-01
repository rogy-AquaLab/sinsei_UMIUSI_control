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
