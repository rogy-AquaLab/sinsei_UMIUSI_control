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
