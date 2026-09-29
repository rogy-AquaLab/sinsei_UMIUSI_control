# `AttitudeController`の制御

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

## `logic::attitude::FeedBack` の yaw — レート制御と方位保持

roll/pitch は body-up の向きだけを合わせる reduced-attitude 制御で、**目標 quaternion の yaw は
意図的に無視する**。yaw は既定で**レート制御**（`AttitudeTarget.yaw_rate` [rad/s]）。

`AttitudeTarget.hold_yaw`（bool、既定 `false`）を立てると、そのレート制御の**外側**に方位保持
ループが乗る。

| `hold_yaw` | 挙動 |
|---|---|
| `false` | 従来どおりのレート制御。**状態を持たない** |
| `false` → `true` のエッジ | **そのときの実測方位をラッチ**して保持対象にする |
| `true` の間 | ラッチした方位を保つ。`yaw_rate` は**ラッチ値を slew** する（`latched += yaw_rate * dt`）ので、小さな修正のたびにトグルしなくてよい |
| `true` → `false` | ラッチを捨てる |

出力は 2 段の縦続で、内側は今までと同じレートループ:

```
hold_yaw なら  commanded_rate = yaw_rate + kp_yaw_hold * wrap(latched - heading)
そうでなければ commanded_rate = yaw_rate
yaw モーメント = kp_yaw_rate * (commanded_rate - omega_z)
```

### なぜ「ヨーレートがほぼ 0 なら保持」にしないのか

閾値と継続時間というノブが 2 つ増えるうえ、**閾値の境目でチャタるとラッチし直すたびに方位が
少しずつずれる**。「流されても気付けない」という、保持を入れたい理由そのものが壊れる。
`hold_yaw` が明示的な bool なら bag にも残り、`ros2 topic echo` でも見える。

### 誤差クランプ（`yaw_hold_relatch_error`、既定 90 deg）

方位誤差がこれを超えたら、**IMU の方位が飛んだ**とみなして追いかけずに現在方位へラッチし直す。

BNO055 は NDOF モード（磁気基準）で動いており、**yaw だけが磁気外乱で飛ぶ**。実機 bag では
**20.1 ms で yaw だけ −169.03 deg 跳び、roll/pitch はほぼ動かず、`|q|` は 1.00000**、
同時刻の `omega_z` は −0.03 rad/s だった（回っていない）。**追いかけると、この跳躍がそのまま
「180 度回れ」という指令に化ける。** 通常の追従誤差は実測で最大 29.2 deg なので、90 deg なら
誤爆しない。

> ⚠ `kp_yaw_hold` と `yaw_hold_relatch_error` は**実機未検証**。プールで振ってから確定すること。
> ⚠ ラッチは disarm で捨てる必要がある（`FeedBack::init()` で `reset()` している）。
