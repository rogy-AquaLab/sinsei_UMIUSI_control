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

## `logic::attitude::Rl`

### TL;DR

学習済み方策 (RL) で姿勢を保つ。`control_mode: "rl"` で選ぶ。libtorch があるときだけビルドされる。

`umiusi_sim` で学習した方策を、同じ repo の `tools/export_deploy_bundle.py` が固めた
`deploy.pt` (TorchScript 1 ファイル) として読む。起動時に golden vectors を再生して、
重み・正規化統計・観測レイアウトが sim と一致することを確かめてから使う。

**配備の経路に autonomy は入らない。** 方策を作るのも配備物を作るのも sim で、
配備するバンドルはこのパッケージが `models/` に同梱する (`models/README.md`)。
既定では `rl.model_name` (既定 `av_mode13`) を
`share/sinsei_umiusi_control/models/<name>/deploy.pt` に解決するので、
`control_mode:=rl` だけで立ち上がる。同梱していないバンドルを使うときだけ
`rl.model_path` に `deploy.pt` のフルパスを指定する。

### 入出力

`FeedForward` と同じインタフェースを使うが、**`target_orientation` の意味が違う**。

| | `FeedForward` | `Rl` |
| --- | --- | --- |
| `target_orientation` | 姿勢ベクトル (線形変換の入力) | **回転ベクトル [rad]** (REP-103・ワールド基準。大きさが回転角、向きが回転軸。ゼロ = 水平・機首方位維持) |
| `target_velocity` | 速度ベクトル (線形変換の入力) | 速度指令 [m/s] (観測にそのまま入る。14 次元の方策は使わない) |
| `esc/duty_cycle` | 正規化した推力 | duty [-1, 1] |
| `servo/angle` | 角度 [deg] | 角度 [deg] |

観測は方策のバンドルが決める。並びは `deploy.pt` の `obs_fields` と照合され、
食い違えば起動しない。

- 18 次元 `[ori_err(3), gyro(3), v_cmd(3), prev_action(8), max_duty(1)]` — duty 上限を観測に持つ
- 17 次元 `[ori_err(3), gyro(3), v_cmd(3), prev_action(8)]` — `attitude_velocity` タスク
- 14 次元 `[ori_err(3), gyro(3), prev_action(8)]` — `attitude` タスク

`ori_err` は現在姿勢から目標姿勢への回転ベクトル (MuJoCo の `mju_subQuat` 相当)。
IMU の quat / gyro は軸変換せずそのまま入れる。ずれていたら IMU ドライバ側 (`AXIS_MAP`) を直す。

観測に入れる `max_duty` は学習分布 `[0.2, 0.4]` へクランプする (duty のクリップ自体は
`rl.max_duty` の設定値のまま)。外れていれば起動ログに警告が出る。

### `action_mode`

| | 出力 | 変換 |
| --- | --- | --- |
| `direct` | 8 次元 `[servo x4, esc x4]` | そのまま使う |
| `modes` | 6 次元のレンチモード**レート** | 積分 -> ミキサ -> 折返しの 3 段で 8 次元に直す |

`modes` の 3 段は sim が回したのと同じ順序・同じ係数で再現しないと、学習したのと別の
プラントになる。係数は `deploy.pt` の `action_contract` が正で、コードにハードコードしない。

### 配備前検証

起動時に `golden.pt` を再生する。突き合わせるのは 2 段階:

1. **ネットの生出力** — 重みと正規化統計が sim と一致するか
2. **`mixed`** (`modes` のときだけ) — 積分・ミキサ・折返しを通した 8 次元が、学習に使った
   env そのもの (`umiusi_rl.envs.mode_mixer.ModeMixer`) の出力と一致するか。
   1 だけでは 3 段の取り違えが素通りする

どちらかがずれていれば `on_configure` が ERROR になり、スラスタは回らない。
観測の**並び**はどちらの golden でも検出できない (組み立て済みの観測を再生するだけ) ので、
そこは `obs_fields` の照合が守る。

### ライフサイクルと制御周期

バンドルの読み込みと golden 再生は `on_configure` に置いてある。実測 86-109 ms
(av_mode13、golden 73 本の再生込み) で、50 Hz の制御周期 20 ms の 4-5 周期ぶん —
`update()` でやるとその間スラスタへ指令が出ない。`update()` 側は入力テンソルと
引数リストを使い回して毎 tick のヒープ確保をなくしてある。

内部状態 (`prev_action` / モード積分器 / レート制限) は `on_activate` で毎回 0 に戻る。
deactivate は disarm の経路なので、前回の値を抱えたまま再開すると最初の tick で
disarm 前の指令が出る (モード積分器は ±1 に飽和したままのことがある)。

### 制約

- **`obs_frame: rep103` のバンドルしか受け付けない**。IMU を無変換で観測に入れるため。
  2026-08-21 のプール試験で pitch/yaw が入れ替わった観測が入り、姿勢制御が全く効かなかった
  ことへの再発防止ゲート。sim 側の変換は `umiusi_sim/tools/convert_policy_frame.py`。
- **`rl` への実行時切替は不可**。バンドルの読み込みと golden 検証が制御周期より長いので、
  `on_configure` でしか入れない (`control_mode` を変えて再 configure する)。
- 18 次元のバンドルは `obs_fields` が**必須**。無ければ起動を拒否する
  (末尾 `max_duty` の位置を照合できないと、golden が PASS しても別の入力を読む)。
- `modes` では `rl.servo_range_deg` と契約の値が一致していなければ起動しない
  (ミキサの正規化と出力側の逆正規化が食い違い、角度が別物になる)。
- 出力は duty。推力 [N] で出すには `duty_per_thrust` のベンチ較正が要る
  (`ThrusterController` は線形、方策側は 2 乗カーブで食い違っている)。
- **鉛直の速度指令は `vertical_ok` を持つ方策にしか渡さない。** 水平専用の方策に z 成分を
  入れると姿勢が崩壊する。同梱バンドルで `vertical_ok` を持つのは `av_cal5_3d_rep103`
  (降下専用) だけなので、既定の `av_mode13` では UI の L2/R2 (`target_velocity.z`) は
  効かず、5 秒ごとに警告が出る。
- **IMU の異常サンプルを弾いていない。** Python の参照実装 (`ImuSanity`) は弾いており、
  観測に直接入るので 1 発で指令が跳ねる (autonomy known_issues A-1)。`prev_action` を
  通って次の観測にも戻るので跳ねは数 tick 残る。フィルタは未移植。
- `servo_sign` (ch 別のサーボ回転センス補正) は未移植。実機の結線が反転していたら要追加。

### パラメータ

`params/controllers.yaml` の `attitude_controller.rl.*` を参照。

指令のレート制限は sim のプラントが持っていたもの。**どちらが効くかは経路で違う**:

| | この logic | 下流 `ThrusterController` | 実効 |
| --- | --- | --- | --- |
| esc | `rl.thrust_slew_per_s` (既定 4.0/s) | `max_duty_step_per_sec` (既定 **1.0/s**) | 厳しい方 = 1.0/s |
| servo | `rl.servo_slew_deg_per_s` (既定 250 deg/s) | 制限なし | 250 deg/s |

esc は下流が 1.0/s なので、`rl.thrust_slew_per_s` の 4.0 は効かない。**それでよい** —
av_mode13 は ESC ランプを `[1.0, 10.0]` の domain randomization で学習しており、1.0/s は
その範囲の中。sim 側の sweep 実測では 1.0/s が範囲内で最良の点だった
(ori 0.154 / null 5.2% / 巡航 104%)。**理由なく上げないこと。**

なお `/cmd/direct` に出す経路 (autonomy の rl_attitude_node) は `ThrusterController` を
迂回する (known_issues B-12) ので、そちらの実効値は 4.0/s になる。**同じ方策でも
スタックによって実効ランプが違う。**

servo にはどちらの経路にも下流の制限が無いので、`rl.servo_slew_deg_per_s` が唯一の制限。

### ビルド

```sh
colcon build --packages-select sinsei_umiusi_control \
  --cmake-args -DTorch_DIR=<venv>/lib/pythonX.Y/site-packages/torch/share/cmake/Torch
```

libtorch が見つからなければ RL logic はビルドから外れ、`control_mode: "rl"` は
`on_configure` で明示的に拒否される。
