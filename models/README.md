# 配備する RL 方策バンドル

`control_mode:=rl` (`logic::attitude::Rl`) が読む配備物。`ament_auto_package` の
`INSTALL_TO_SHARE` で `share/sinsei_umiusi_control/models/` へ入る。

- `deploy.pt` — ネット + 正規化パラメータ + `action_contract` の TorchScript
- `golden.pt` — 配備前検証に再生する golden vectors (`obs` / `act`、レンチモードは `mixed` も)

## 由来

**手で作らない。** `umiusi_sim` が学習して書き出したものを、そのままコピーしている。

```sh
# umiusi_sim にて
uv run python tools/export_deploy_bundle.py models/<name>
cp models/<name>/{deploy.pt,golden.pt} <control>/models/<name>/
```

元のバンドル (`export/weights.pt` + `obs_norm.npz` + `meta.json`、`golden.npz`、
`meta.yaml`) は `umiusi_sim` の `models/<name>/` に版管理されている。そちらが正本。

同梱時点の `umiusi_sim`: `430944e`

## 中身

| name | 観測 | action_mode | 用途 |
| --- | --- | --- | --- |
| `av_mode13` | 18-D | `modes` | 出荷。`rl.model_name` の既定 |
| `av_cal1_best_rep103` | 17-D | `direct` | 17-D 本命 |
| `att_cal1_best_rep103` | 14-D | `direct` | 姿勢専用フォールバック (速度指令を持たない) |
| `av_cal5_3d_rep103` | 17-D | `direct` | 降下専用 (EXPERIMENTAL)。水平指令と併用しない |
| `av_sim2real2_rep103` | 17-D | `direct` | B 案 (旧物理だが指令が最も平滑) |

## 差し替えるとき

`deploy.pt` と `golden.pt` は必ず同じ実行で作った組で入れ替える。片方だけ新しいと
配備前検証が落ちる (それが狙いなので、落ちたら組が揃っていないことを疑う)。
