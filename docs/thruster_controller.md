# `ThrusterController`の制御

## TL;DR

- 1基分のスラスタを制御するコントローラ。`thruster_controller_(lf|lb|rb|rf)`ごとに1つずつ実体化され、`params/controllers.yaml`の設定でハードウェアIDやドライバ種別を切り替える。
- `cmd/direct/thruster_controller/output_*`トピックからの直接指令が届く場合はそれをそのままESC/サーボに流し、パブリッシャが居ないときのみ内部ロジック(`logic::thruster::LinearAcceleration`)で推力指令を生成する。
- `(esc|servo)_disabled`パラメータの処理については[resolve_thruster_mode](#resolve_thruster_mode)の項目を参照。

## 入出力構造

`ThrusterController::Input`/`Output`は以下の4種類のインタフェースを束ねる。

| 種別 | 名称 | 説明 |
| ---- | ---- | ---- |
| Command In | `cmd/thruster/esc::Runnable`, `cmd/thruster/esc::Thrust`, `cmd/thruster/servo::Runnable`, `cmd/thruster/servo::Angle` | GateControllerやAttitudeControllerから受け取る指示。`esc::Thrust`は後段でDutyに変換される。 |
| State In (CAN) | `state/thruster/esc::Rpm`, `Voltage`, `WaterLeaked` | ドライバ種別が`can`のときのみ購読。 |
| State In (Direct) | `state/thruster/esc::Health`, `state/thruster/servo::Health` | ドライバ種別が`direct`のときのみ購読。 |
| Command Out | `esc/allowed`, `esc/duty_cycle`, `servo/allowed`, `servo/angle` | 実機へ書き込まれるコマンド。`*_allowed`は`ThrusterMode::Runnable`判定結果から導出。 |
| State Out | `esc/mode`, `esc/duty_cycle`, `servo/mode`, `servo/angle` (+ driver-specific health/RPM) | GateControllerやヘルスチェックノードが購読する状態量。IDを隠すため`thruster/esc/rpm`のように番号を取り除いてエクスポートされる。 |

## 受信トピックと優先順位

- 名前空間: `cmd/direct/thruster_controller/`
- 個別トピック: `output_lf`, `output_lb`, `output_rb`, `output_rf` (型: `ThrusterOutput`)
- 一括トピック: `output_all` (型: `ThrusterOutputAll`)

各コントローラは自分の名前 (`thruster_controller_lf` など) からインデックスを切り出し、該当するトピックを購読する。`output_*`にパブリッシャが1つでも存在すれば `output_all` は無視され、より細粒度な指令を優先する。

## パラメータ一覧

| パラメータ | 型 / 例 | 用途 |
| ---------- | ------- | ---- |
| `thruster_driver_type` | string (`can` or `direct`) | スラスタ駆動に使用するハードウェアを切り替え。 |
| `id` | int (1-4) | URDF上の`thruster{id}`に対応。Command/State Interface名生成に使用。 |
| `esc_disabled`, `servo_disabled` | bool | trueの場合は常に`ThrusterMode::Disabled`で動作し、`*_allowed`をfalseに落とす。 |
| `is_forward` | bool | 推力向きを示すフラグ。`false`にするとロジック内で推力指令を反転し、逆向きに取り付けたスラスタでも正の推力指令で前進できる。 |
| `duty_per_thrust` | double | `LinearAcceleration`が推力[N]からDuty(無次元)へ換算する係数。 |
| `max_duty` | double (0.0-1.0) | Dutyの絶対値上限。 |
| `max_duty_step_per_sec` | double | 1秒あたりのDuty変化上限。周期`dt`あたりのステップ制限は `max_duty_step_per_sec * dt`。 |

これらは `params/controllers.yaml` でスラスタごとに設定可能。

## `logic::thruster::LinearAcceleration`

内部ロジックは現状フィードフォワード (`ControlMode::FeedForward`) のみが実装されている。

### `ControlMode::FeedForward`

Dutyの台形加速を行う。

```math
\begin{aligned}
\Delta d_{\max} &= \text{max\_duty\_step\_per\_sec} \times \Delta t \\
\underline{d} &= \max(-\text{max\_duty}, d_{k-1} - \Delta d_{\max}) \\
\overline{d} &= \min(\text{max\_duty}, d_{k-1} + \Delta d_{\max}) \\
 d_k &= \text{clamp}(\text{duty\_per\_thrust} \times F,\ \underline{d},\ \overline{d})
\end{aligned}
```

ここで $F$ は`input.cmd.esc_thrust.value`に格納された推力指令、 $d_k$ は次のDuty。

## `resolve_thruster_mode`

`util::resolve_thruster_mode(disabled, runnable)` はThrusterController本体とLogic双方で使用されており、2つの引数をもとに`ThrusterMode`を算出するヘルパー関数。`disabled=true` のときは`ThrusterMode::Disabled` を返し、それ以外は `runnable` フラグの真偽によって `Runnable` か `Standby` を返す。
