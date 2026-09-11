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

## 暫定指令インターフェース

次の5トピックへ`sinsei_umiusi_msgs/msg/ThrusterOutput`を送る。

- `cmd/direct/alexandrite_controller/thruster1`
- `cmd/direct/alexandrite_controller/thruster2`
- `cmd/direct/alexandrite_controller/thruster3`
- `cmd/direct/alexandrite_controller/crawler_left`
- `cmd/direct/alexandrite_controller/crawler_right`

スラスタでは`runnable.esc`、`duty_cycle`、`runnable.servo`、`angle`を使用する。
`angle`の単位はメッセージ定義どおりradで、VESC送信前にdegreeへ変換する。

クローラーでは`runnable.esc`と`duty_cycle`だけを使用し、サーボ関連フィールドは無視する。
左右の差動計算は行わないため、左右それぞれのDutyを送信側で指定する。

例として左クローラーをDuty 0.2で動かす場合:

```sh
ros2 topic pub -r 10 \
  /cmd/direct/alexandrite_controller/crawler_left \
  sinsei_umiusi_msgs/msg/ThrusterOutput \
  "{runnable: {esc: true, servo: false}, duty_cycle: 0.2, angle: 0.0}"
```

指令は`max_duty`で制限される。各トピックの更新が`command_timeout`秒間途絶えると、
該当アクチュエータへDuty 0を出力する。デフォルトはそれぞれ0.5、0.5秒。
