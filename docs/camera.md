# カメラ構成

## TL;DR

- カメラは `gst_camera_node` を2個起動して扱う。`launch/main.yaml` では `pi_camera` と `usb_camera` を同時に立ち上げる。
- 各ノードは画像をROS topicへpublishせず、GStreamer pipelineをそのまま実行する。
- 実際の入出力は `params/cameras.yaml` の `pipeline` パラメータで定義する。
- デフォルト設定では両カメラとも `rtspclientsink` を使うため、RTSPサーバを別途用意する必要がある。

## 起動構成

`launch/main.yaml` では以下の2ノードを起動する。

| ノード名 | 実行ファイル | 用途 |
| ---- | ---- | ---- |
| `pi_camera` | `gst_camera_node` | Raspberry Pi Camera用 |
| `usb_camera` | `gst_camera_node` | USB Camera用 |

どちらのノードも同じ実装を使い、ノード名ごとに `params/cameras.yaml` から別の `pipeline` を読む。

起動可否は `params/launch_args.yaml` で定義されている `enable_cameras` 引数で制御する。CIでは `false` を渡してカメラノードを起動しない。

## `gst_camera_node`

`gst_camera_node` は以下の初期化だけを行う。

1. `pipeline` パラメータを読む
2. `gst_parse_launch()` で pipeline を構築する
3. pipeline を `GST_STATE_PLAYING` に遷移させる
4. timer でGStreamer busをpollし、`ERROR` と `EOS` を監視する

このノードは「GStreamer pipelineをROS launchに載せるためのアダプタ」であり、エンコード方式や転送先、デバイス指定はすべて pipeline 文字列で設定する。

## パラメータ

各ノードが使うパラメータは現状 `pipeline` だけ。

| パラメータ | 型 | 用途 |
| ---- | ---- | ---- |
| `pipeline` | string | `gst_parse_launch()` にそのまま渡すGStreamer pipeline |

空文字列は不可。

## デフォルト pipeline

`params/cameras.yaml` の初期値は以下の意図を持つ。

### `pi_camera`

- `libcamerasrc` でlibcamera経由でカメラ映像を取得
- `videoconvert` を通す
- `v4l2h264enc` でH.264へエンコードする
- `rtspclientsink` で `rtsp://localhost:8554/cam1` へ送る

### `usb_camera`

- `v4l2src device=/dev/video4` でv4l2経由でH.264エンコードされたカメラ映像を取得
- `rtspclientsink` で `rtsp://localhost:8554/cam2` へ送る

## 運用上の注意

- このノードはRTSPサーバ自体は起動しない。`rtspclientsink` の接続先は別プロセスで用意する必要がある。

## 共有メモリ版 (`params/cameras_shm.yaml`)

自律側 (`umiusi_autonomy` の `camera_bridge_node`) が RTSP を受けて H.264 をデコードし直す代わりに、エンコード前の映像を共有メモリから読むための opt-in 設定。既定の `cameras.yaml` は変えていない。

```bash
ros2 launch sinsei_umiusi_control main.yaml \
  cameras_param_file:=$(ros2 pkg prefix sinsei_umiusi_control)/share/sinsei_umiusi_control/params/cameras_shm.yaml
```

- `pi_camera` だけ `tee` で分岐する。RTSP 側 (`cam1`、UI の映像) は `cameras.yaml` と同じ
- 分岐側: `queue leaky=downstream max-size-buffers=1` → 15 fps に間引き → BGR 320x240 → `shmsink socket-path=/tmp/umiusi_cam1.sock`
- 読む側が居ない / 止まっている / 落ちたときも RTSP 側は止まらない (分岐側の queue が古いフレームを捨てる)
- 分岐側でエラーが出るとパイプライン全体が止まるので、HW 専用要素 (`v4l2convert`) は使わず software の `videoconvertscale` にしている
- socket のパスと BGR 320x240 は読む側と揃える (`umiusi_autonomy` の `launch_common.SHM_SOCKET` と `camera_bridge_node` の `width` / `height`)
