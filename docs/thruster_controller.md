# `ThrusterController`の制御

## TL;DR

- 1基分のスラスタを制御するコントローラ。`thruster_controller_(lf|lb|rb|rf)`ごとに1つずつ実体化され、`params/controllers.yaml`の設定でハードウェアIDなどを切り替える。
- `cmd/direct/thruster_controller/output_*`トピックからの直接指令が届く場合はそれをESC/サーボに流し、パブリッシャが居ないときのみ内部ロジック(`logic::thruster::LinearAcceleration`)で推力指令を生成する。
- **ハードウェア境界の歯止め(`ThrusterLimits`)は指令の出所によらず必ず通る。** 詳細は[アクチュエータ制限](#ハードウェア境界の歯止め-thrusterlimits)の項目。
- `(esc|servo)_disabled`パラメータの処理については[resolve_thruster_mode](#resolve_thruster_mode)の項目を参照。

## 入出力構造

`ThrusterController::Input`/`Output`は以下の4種類のインタフェースを束ねる。

| 種別 | 名称 | 説明 |
| ---- | ---- | ---- |
| Command In | `cmd/thruster/esc::Runnable`, `cmd/thruster/esc::Thrust`, `cmd/thruster/servo::Runnable`, `cmd/thruster/servo::Angle` | GateControllerやAttitudeControllerから受け取る指示。`esc::Thrust`は後段でDutyに変換される。 |
| State In | `state/thruster/esc::Rpm`, `Voltage`, `WaterLeaked` | CANハードウェアから受け取る状態。 |
| Command Out | `esc/allowed`, `esc/duty_cycle`, `servo/allowed`, `servo/angle` | 実機へ書き込まれるコマンド。`*_allowed`は`ThrusterMode::Runnable`判定結果から導出。 |
| State Out | `esc/mode`, `esc/duty_cycle`, `servo/mode`, `servo/angle`, `thruster/esc/rpm`, `thruster/esc/voltage`, `thruster/esc/water_leaked` | GateControllerやヘルスチェックノードが購読する状態量。IDを隠すため`thruster1/esc/rpm`のような名前は `thruster/esc/rpm` に変換してエクスポートされる。 |

## 受信トピックと優先順位

- 名前空間: `cmd/direct/thruster_controller/`
- 個別トピック: `output_lf`, `output_lb`, `output_rb`, `output_rf` (型: `ThrusterOutput`)
- 一括トピック: `output_all` (型: `ThrusterOutputAll`)

各コントローラは自分の名前 (`thruster_controller_lf` など) からインデックスを切り出し、該当するトピックを購読する。`output_*`にパブリッシャが1つでも存在すれば `output_all` は無視され、より細粒度な指令を優先する。

## パラメータ一覧

| パラメータ | 型 / 例 | 用途 |
| ---------- | ------- | ---- |
| `id` | int (1-4) | URDF上の`thruster{id}`に対応。Command/State Interface名生成に使用。 |
| `esc_disabled`, `servo_disabled` | bool | trueの場合は常に`ThrusterMode::Disabled`で動作し、`*_allowed`をfalseに落とす。 |
| `is_forward` | bool | 推力向きを示すフラグ。`false`にするとロジック内で推力指令を反転し、逆向きに取り付けたスラスタでも正の推力指令で前進できる。**Logic側**(core経路のみ)。 |
| `servo_sign` | double (±1.0) | サーボの回転センス。逆向きに取り付けた基で`-1.0`にすると角度指令を反転する。**歯止め側**(指令の出所によらず適用)。 |
| `duty_per_thrust` | double | `LinearAcceleration`が推力[N]からDuty(無次元)へ換算する係数。 |
| `max_duty` | double (0.0-1.0) | Dutyの絶対値上限。**歯止め側**(指令の出所によらず適用)。`0.0`にするとそのスラスタは一切出力しない(configure時に警告)。 |
| `max_duty_step_per_sec` | double | 1秒あたりのDuty変化上限。周期`dt`あたりのステップ制限は `max_duty_step_per_sec * dt`。**Logic側**(core経路のみ)。 |

これらは `params/controllers.yaml` でスラスタごとに設定可能。

## ハードウェア境界の歯止め (`ThrusterLimits`)

`include/sinsei_umiusi_control/controller/thruster_limits.hpp`。

`update_and_write_commands`は、Logicを通った出力にも直接指令にも**必ず**この歯止めを通してから`output.cmd`(=実機へ書き込む値)を作る。

| 歯止め | 内容 |
| ---- | ---- |
| Dutyの絶対値上限 | `±max_duty` にクランプ |
| サーボの回転センス | `servo_sign` (±1.0) を角度に掛ける |
| サーボ角の範囲 | `±90 deg` にクランプ。VESCは`-90.0 ~ 90.0`を`0.0 ~ 1.0`に写すため、範囲外は**CANフレームの送信そのものが失敗**していた |

クランプが実際に効いたときは警告を出す(3秒スロットル)。**黙って値を削らない**ためで、較正(`sinsei_UMIUSI_autonomy`の`tools/thruster_cmd.py`)は生の指令を出す前提なので、上限に当たったことに気付けないと推力カーブの上端を誤って測る。ベンチ較正で`max_duty`を超える指令を出したいときは、そのパラメータ自体を上げること。

### 設計上の約束: ステートレスであること

**ここに置くのは状態を持たない範囲の強制だけ**で、意図的にそう決めている。

* **スルーレート制限はここに置かない。** あれはハードの保護ではなく「方策が学習したプラント(シミュレータ)を実機で再現する」ための平滑化なので、持ち主は指令を出す側(`sinsei_UMIUSI_autonomy`の`thruster_limits.py`)。状態を持つ制限を素通し経路に挟むと、(a) 生の指令をラッチしないと制限後の値に制限が重ね掛けされる (b) 実効レートがpublisherのレートに依存する (c) 較正ツールが生の指令を出せなくなる、という三重の問題が出る。**core経路のスルーレートは従来どおり`LinearAcceleration`が持つ。**
* `is_forward` / `duty_per_thrust` もここには無い。あれは「推力[N] → Duty」の**換算**であって範囲の強制ではない。直接指令はすでにDutyで届くので掛け直すと二重換算になる(`sinsei_UMIUSI_autonomy`側のアロケーションが、プラントの二次カーブを逆写像でプリワープしたDutyを出している)。

ステートレスなので**`output.state`は書き換えない**。`state`が指令のエコーであるという既存の契約を保つのと、`output`が永続メンバなので書き戻すと次の周期に歯止めが重ね掛けされるため(`servo_sign = -1`のとき角度の符号が周期ごとに反転する)。

**この変更でcore経路の挙動は一切変わらない。** `LinearAcceleration`は無変更で、そこを通った値はすでに`max_duty`内に収まっているためクランプは恒等、`servo_sign`の既定は`1.0`で恒等、サーボ角もFFは`atan`由来で範囲内。

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
