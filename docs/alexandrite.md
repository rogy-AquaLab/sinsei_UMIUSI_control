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

UIと接続するため、`sinsei_umiusi_core`の代わりにrosbridge（ポート9090）も起動する。
カメラはUMIUSIと同じ2台（`pi_camera` → `cam1`、`usb_camera` → `cam2`）を`params/cameras.yaml`の
設定で起動する。RTSPサーバ（MediaMTX）は別途起動しておくこと。不要な場合は
`enable_rosbridge:=false`、`enable_cameras:=false`で止められる。

## 指令インターフェース

スラスタはUMIUSIと同じトピック・メッセージを使用する。

- `cmd/target` (`sinsei_umiusi_msgs/msg/Target`)
- `cmd/thruster_runnable_all` (`sinsei_umiusi_msgs/msg/ThrusterRunnableAll`)

### スラスタの配置と極性

機体座標は x = 前、y = 左、z = 上とする。スラスタはUMIUSIと同じく左前から反時計回りに数える。

```text
                前 (+x)
   thruster1 (左前)    thruster3 (右前)     ← 推力の軸がx軸から内側へφ傾いている
            ●（重心）
          thruster2 (後ろ)                  ← x軸上。yz平面内で回る
```

| スラスタ | 位置 | サーボ角0・正Dutyの推力方向 | 運転許可 |
|---|---|---|---|
| `thruster1` | 左前 (`front_thruster_x`, `front_thruster_y`) | (-cosφ, sinφ) | `lf` |
| `thruster2` | 後ろ (-`rear_thruster_x`, 0) | (0, -1) | `lb` |
| `thruster3` | 右前 (`front_thruster_x`, -`front_thruster_y`) | (cosφ, sinφ) | `rf` |

φは`front_thruster_angle`で、前側2基の推力の軸がx軸から内側へ傾いている角度（UMIUSIの45度配置ではφ = 45度）。

どのスラスタも、サーボ角0・正のDutyでz軸まわり反時計回り（+yaw）のモーメントを生む向きを正とする。
サーボ角θ [rad] のとき、推力の向きは cosθ・（上の表の方向）+ sinθ・(+z) になる。

### ミキサー

`cmd/target`の`velocity`（x, y, z）を力、`orientation`（x = roll、y = pitch、z = yaw）をモーメントの
目標として、3基の水平・垂直成分（計6つ）に割り振る。水平成分は前後・左右・ヨー、垂直成分は上下・
ロール・ピッチから一意に決まるため、例えば横移動でヨーは生じない。ただし後ろの1基はx方向の力を
出せないため、前後移動は前側2基だけで行う。

各スラスタの水平・垂直成分から、サーボ角 = atan(垂直 / 水平)、Duty = 符号付きの√(水平² + 垂直²) を求める。
どれかのDutyが`max_duty`（デフォルト0.5）を超える場合は、力・モーメントの向きを保つよう3基を同じ比率で縮める。

取り付け位置（`front_thruster_x`、`front_thruster_y`、`rear_thruster_x`）は比だけを使うため単位は任意。
`params/alexandrite_controllers.yaml`には実測値（前側2基: 重心から前へ200 mm・左右へ175 mm・内側へ20度、
後ろの1基: 重心から後ろへ300 mm）を入れている。

クローラー目標は次の別トピックへ送る。

- `cmd/crawler_target` (`sinsei_umiusi_msgs/msg/Target`)

`velocity.x`を前後入力、`orientation.z`を旋回入力（どちらも-1.0〜1.0）として、以下の差動計算を行う。

```text
left  = velocity.x - orientation.z
right = velocity.x + orientation.z
```

`left`、`right`のどちらかが1.0を超える場合は、左右の比を保ったまま大きいほうが1.0になるよう縮める。
そのうえで`crawler_max_duty`（デフォルト0.5）を掛けたものをDutyとして出力する。

既存の上流が4基すべてへ同じ運転許可を送っていることを利用し、暫定的に
`cmd/thruster_runnable_all.rb.esc`を左右クローラー共通の運転許可として使用する。

例としてクローラーへ前進入力0.2（デフォルトではDuty 0.1）を送る場合:

```sh
ros2 topic pub -r 10 \
  /cmd/crawler_target \
  sinsei_umiusi_msgs/msg/Target \
  "{velocity: {x: 0.2}, orientation: {z: 0.0}}"
```

スラスタとクローラーのDutyは、急激な加減速を抑えるため、1秒あたりの変化量を
`max_duty_step_per_sec`（スラスタ）、`crawler_max_duty_step_per_sec`（クローラー）までに制限する
（デフォルトはどちらも1.0で、UMIUSIの`thruster_controller`と同じ。0から0.5まで0.5秒かかる）。
各基を独立に制限するため、加減速の途中では3基の出力の比が一時的に崩れる。運転許可が無くなったときは、
ランプをかけずにすぐDuty 0にする。

目標値または運転許可の更新が
`command_timeout`秒間（デフォルト0.5秒）途絶えると、該当アクチュエータへDuty 0を出力する。

運転許可が無い（disarm中、または上記のタイムアウト）アクチュエータには、Duty 0を5回（約100 ms）送って
止めたあと、何も送らない。UMIUSIと同じく、VESCのタイムアウト（VESC側の設定）が過ぎるとモーターと
サーボが脱力する。

## 状態

スラスタは、UMIUSIと同じ`state/thruster_state_all`（`sinsei_umiusi_msgs/msg/ThrusterStateAll`）に、
指令と同じ対応（`thruster1`、`thruster2`、`thruster3` = `lf`、`lb`、`rf`）で出す。`rb`は存在しないため、
常に`DISABLED`になる。クローラーは`state/crawler_left`、`state/crawler_right`
（`sinsei_umiusi_msgs/msg/ThrusterState`）に出し、サーボが無いため`mode.servo`は常に`DISABLED`になる。

`mode`は実際にVESCへ出力を許可しているかどうか（目標・運転許可が途絶えていないことも含む）、
`duty_cycle`と`angle`はVESCへ送っている指令値、`rpm`、`input_voltage`、`water_leaked`はVESCから
読んだ値を表す。

## バラスト

2つのバラストは、[SteppingMotorDriver](https://github.com/rogy-AquaLab/SteppingMotorDriver)
基板1枚の2軸で動かす。`ballast1`がM1、`ballast2`がM2に対応する。基板とは同じCANバス上で
標準ID（0x320〜0x329）でやり取りする。CANパケットの詳細は基板側の
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

## BMS

Harmony16互換のBMS（CAN ID `harmony_bms_id`、省略時は3）の状態は、`bms_controller`がUMIUSIの
`gate_controller`と同じtopicに出す。

| topic | 型 | 内容 |
|---|---|---|
| `state/power/battery` | `sensor_msgs/msg/BatteryState` | パック電圧、電流（充電時を正）、SoC、セル電圧 |
| `state/power/bms` | `sinsei_umiusi_msgs/msg/BmsState` | SoH、温度、バランシング、パワースイッチ、フォルト |
