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

$$
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
                \frac{\sqrt{f_{i\text{h}}^2 + f_{i\text{v}}^2}}{\text{max}(\sqrt{f_{j\text{h}}^2 + f_{j\text{v}}^2})} &\text{if} \quad f_{i\text{h}} \geq 0 \\
                -\frac{\sqrt{f_{i\text{h}}^2 + f_{i\text{v}}^2}}{\text{max}(\sqrt{f_{j\text{h}}^2 + f_{j\text{v}}^2})} &\text{if} \quad f_{i\text{h}} < 0
          \end{cases}
\end{cases}
$$

このとき以下の行列$A$によって$U$から$Y = AU$を得る。

$$
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
$$

まとめると、

$$
\begin{cases}
    \phi_i = \text{atan}(f_{i\text{v}} / f_{i\text{h}}) \\
    f_i = \begin{cases}
                \frac{\sqrt{f_{i\text{h}}^2 + f_{i\text{v}}^2}}{\text{max}(\sqrt{f_{j\text{h}}^2 + f_{j\text{v}}^2})} &\text{if} \quad f_{i\text{h}} \geq 0 \\
                -\frac{\sqrt{f_{i\text{h}}^2 + f_{i\text{v}}^2}}{\text{max}(\sqrt{f_{j\text{h}}^2 + f_{j\text{v}}^2})} &\text{if} \quad f_{i\text{h}} < 0
          \end{cases}
\end{cases}
$$
$$
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
$$
