# Alexandrite暫定制御

Alexandriteは、サーボ付きBLDCアジマススラスタ3基と、サーボなしDCクローラー2基を
それぞれVESC経由で制御する。

## 起動

VESCのCAN IDを実機に合わせて指定すること。省略時は順に124〜128を使用する。

```sh
ros2 launch sinsei_umiusi_control alexandrite.yaml \
  thruster1_vesc_id:=124 \
  thruster2_vesc_id:=125 \
  thruster3_vesc_id:=126 \
  crawler_left_vesc_id:=127 \
  crawler_right_vesc_id:=128
```

## 指令インターフェース

スラスタはUMIUSIと同じトピック・メッセージを使用する。

- `cmd/target` (`sinsei_umiusi_msgs/msg/Target`)
- `cmd/thruster_runnable_all` (`sinsei_umiusi_msgs/msg/ThrusterRunnableAll`)

前側の`thruster1`、`thruster2`は、UMIUSIの前側2基と同じ45度配置として、既存の
FeedForward変換の`lf`、`rf`相当の出力を割り当てる。

後側の`thruster3`は機体後方のx軸上にあり、yz平面内で回転するものとして、次の暫定変換を
行う。サーボ角0度・正Dutyの推力方向は+yとする。

```text
rear_horizontal = velocity.y - orientation.z
rear_vertical   = velocity.z + orientation.y
servo_angle     = atan(rear_vertical / rear_horizontal)
motor_duty      = signed_hypot(rear_horizontal, rear_vertical)
```

後部スラスタの機体上の正Duty方向が-yの場合は、`rear_thruster_reversed: true`を指定する。
実寸のモーメントアームが未確定なため、後部スラスタのpitch・yaw係数は暫定的に1としている。

`cmd/thruster_runnable_all`の`lf`、`rf`、`lb`を、それぞれ`thruster1`、`thruster2`、
`thruster3`の運転許可に使用する。

クローラー目標は次の別トピックへ送る。

- `cmd/crawler_target` (`sinsei_umiusi_msgs/msg/Target`)

`velocity.x`を前後入力、`orientation.z`を旋回入力として、以下の差動計算を行う。

```text
left  = velocity.x - orientation.z
right = velocity.x + orientation.z
```

既存の上流が4基すべてへ同じ運転許可を送っていることを利用し、暫定的に
`cmd/thruster_runnable_all.rb.esc`を左右クローラー共通の運転許可として使用する。

例としてクローラーへ前進目標0.2を送る場合:

```sh
ros2 topic pub -r 10 \
  /cmd/crawler_target \
  sinsei_umiusi_msgs/msg/Target \
  "{velocity: {x: 0.2}, orientation: {z: 0.0}}"
```

出力は`max_duty`で制限される。目標値または運転許可の更新が`command_timeout`秒間
途絶えると、該当アクチュエータへDuty 0を出力する。デフォルトはそれぞれ0.5、0.5秒。

## バラスト

2つのバラストは、[SteppingMotorDriver](https://github.com/rogy-AquaLab/SteppingMotorDriver)
基板1枚の2軸で動かす。`ballast1`がM1、`ballast2`がM2に対応する。基板とは同じCANバス上で
標準ID（0x310〜0x319）でやり取りする。CANパケットの詳細は基板側の
`docs/can_protocol.md`を参照。

暫定的に、`ballast_controller`が次のtopicで操作する（`ballastN`は`ballast1`か`ballast2`）。

| topic | 型 | 内容 |
|---|---|---|
| `cmd/ballastN/target` | `std_msgs/msg/Float64` | 取り込む水の量の目標。0.0（排水しきった状態）〜1.0（注水しきった状態）。範囲外は無視する |
| `cmd/ballastN/calibrate` | `std_msgs/msg/Empty` | ゼロ点を取り直す。異常停止からの復帰もこれで行う |
| `state/ballastN/position` | `std_msgs/msg/Float64` | 現在位置（0.0〜1.0）。キャリブレーション中・異常停止中・基板から状態が届かないときはNaN |
| `state/ballastN/state` | `std_msgs/msg/UInt8` | 基板の状態。5が準備完了、6が異常停止（値の意味は基板側の資料を参照） |
| `state/ballastN/fault_reason` | `std_msgs/msg/UInt8` | 異常停止の理由 |

- 目標は一度送れば保持される。目標を受け取るまでは何も送らない。
- 基板は準備完了前に受け取った目標を無視するため、同じ目標を約100 msごとに送り直す。
- 位置は取り込んだ水の量を表す。0.0（Minスイッチ側）が排水、1.0（Maxスイッチ側）が注水。

例として、`ballast1`を50 %の位置へ動かす場合:

```sh
ros2 topic pub --once /cmd/ballast1/target std_msgs/msg/Float64 "{data: 0.5}"
```
