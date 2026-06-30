# imrc_canabe

`imrc_canabe` は、ROS 2 トピックと CANable アダプタの CAN 通信を相互変換する Python ノードです。
`imrc_messages/msg/EcanCommand` を受け取り CAN フレームとして送信し、CAN から受信したフレームを同じメッセージ型で ROS 2 に再配信します。

現状の実装はクラシック CAN を前提としており、CANable は `slcan` インターフェースで接続します。

## 主な機能

- ROS 2 トピックから `EcanCommand` を購読して CAN フレームを送信
- CAN フレームをポーリング受信して `EcanCommand` として publish
- トピック名、CAN デバイス、ビットレート、ポーリング周期を ROS パラメータで変更可能

## 前提条件

- ROS 2 環境がセットアップ済みであること
- CANable などの `slcan` 対応アダプタが利用できること
- `imrc_messages/msg/EcanCommand` がビルド済みであること
- `python3-can` が利用可能であること

このパッケージは `package.xml` 上で以下に依存しています。

- `rclpy`
- `python3-can`
- `imrc_messages`

## EcanCommand メッセージ

このノードは `imrc_messages/msg/EcanCommand` を利用します。コード上は少なくとも次のフィールドを前提にしています。

- `unit_code`
- `unit_index`
- `payload_index`
- `payload_entry`
- `data`

想定している意味と許容範囲は次の通りです。

| フィールド | 型の想定 | 範囲 | 用途 |
| --- | --- | --- | --- |
| `unit_code` | 整数 | `0..63` | CAN ID 上位 6 bit |
| `unit_index` | 整数 | `0..15` | CAN ID 中位 4 bit |
| `payload_index` | 整数 | `0..7` | ペイロード先頭 1 byte の上位 3 bit |
| `payload_entry` | 整数 | `0..31` | ペイロード先頭 1 byte の下位 5 bit |
| `data` | byte 配列相当 | 送信時は `0..7` byte | CAN データ本体 |

## 変換仕様

### ROS 2 -> CAN

`EcanCommand` を受信すると、以下のルールで CAN フレームを組み立てます。

- Arbitration ID: `(unit_code << 5) | (unit_index << 1) | 0x01`
- Data[0]: `(payload_index << 5) | payload_entry`
- Data[1..]: `msg.data`

そのため、送信される CAN フレームは次の制約を持ちます。

- 標準 ID のみ対応
- 拡張 ID は未対応
- クラシック CAN のみ対応
- `msg.data` は最大 7 byte
- 実際の CAN payload は `1 + len(msg.data)` byte

### CAN -> ROS 2

受信した CAN フレームは以下のルールで `EcanCommand` に復元されます。

- `unit_code = (arbitration_id >> 5) & 0x3F`
- `unit_index = (arbitration_id >> 1) & 0x0F`
- `payload_index = (data[0] >> 5) & 0x07`
- `payload_entry = data[0] & 0x1F`
- `data = data[1:]`

受信時の制約は次の通りです。

- 拡張 ID フレームは破棄されます
- データ長 0 byte のフレームは破棄されます
- 8 byte を超えるフレームは破棄されます

## ノード情報

- パッケージ名: `imrc_canabe`
- 実行名: `canable_sender`
- ノード名: `canable_sender`

## ROS パラメータ

| パラメータ名 | デフォルト | 説明 |
| --- | --- | --- |
| `channel` | `/dev/ttyACM0` | CANable のデバイスパス |
| `bitrate` | `500000` | CAN ビットレート |
| `receive_topic` | `can_tx_demo` | 送信コマンド購読トピック |
| `publish_topic` | `can_rx_demo` | 受信フレーム配信トピック |
| `interface` | `slcan` | `python-can` のインターフェース名 |
| `poll_period_sec` | `0.01` | CAN 受信ポーリング周期 [s] |

## ビルド

ワークスペースのルートで実行します。

```bash
colcon build --packages-select imrc_canabe
source install/setup.bash
```

`imrc_messages` が同じワークスペースにある場合は、先に一緒にビルドしてください。

## 実行方法

デフォルト設定で起動する例です。

```bash
ros2 run imrc_canabe canable_sender
```

パラメータを変更して起動する例です。

```bash
ros2 run imrc_canabe canable_sender --ros-args \
  -p channel:=/dev/ttyACM0 \
  -p bitrate:=1000000 \
  -p receive_topic:=ecan_tx \
  -p publish_topic:=ecan_rx \
  -p interface:=slcan \
  -p poll_period_sec:=0.01
```

## トピック I/O

### Subscribe

- トピック名: `receive_topic` パラメータで指定
- デフォルト: `can_tx_demo`
- 型: `imrc_messages/msg/EcanCommand`

### Publish

- トピック名: `publish_topic` パラメータで指定
- デフォルト: `can_rx_demo`
- 型: `imrc_messages/msg/EcanCommand`

## 使用例

送信トピックへ 1 件 publish する例です。

```bash
ros2 topic pub --once /can_tx_demo imrc_messages/msg/EcanCommand \
  "{unit_code: 1, unit_index: 2, payload_index: 3, payload_entry: 4, data: [16, 32, 48]}"
```

受信トピックを監視する例です。

```bash
ros2 topic echo /can_rx_demo
```

起動時にトピック名を変更した場合は、上記コマンドのトピック名も合わせて変更してください。

## ログ出力

正常時は以下のような内容をログに出します。

- 接続した CAN インターフェース情報
- 送信した CAN フレームの ID / DLC / データ
- 受信した CAN フレームの ID / DLC / データ

異常時は以下をログに出します。

- `python-can` 未導入
- `EcanCommand` 型を import できない
- 範囲外フィールドや 7 byte 超過データ
- CAN デバイス接続エラー
- 受信フレーム形式エラー

## 注意事項

- `imrc_messages/msg/EcanCommand` の実体定義はこのリポジトリには含まれていません
- `slcan` 以外のインターフェースを使う場合は、`python-can` と接続先の要件を別途確認してください
- このノードは受信をタイマポーリングで処理しています。高頻度通信では `poll_period_sec` の調整が必要になる場合があります
- `unit_index` は 4 bit で復元されるため、ID 設計上 bit 0 は常に `1`、bit 1..4 が `unit_index`、bit 5..10 が `unit_code` として扱われます

## トラブルシュート

### `python-can is not installed` と表示される

`python3-can` を導入してからワークスペースを再度 source してください。

### `Custom message type "imrc_messages/msg/EcanCommand" is not available` と表示される

`imrc_messages` パッケージに `EcanCommand.msg` が存在することを確認し、ワークスペースを再ビルドして `source install/setup.bash` を実行してください。

### CAN デバイスを開けない

次を確認してください。

- `channel` が正しいデバイスパスか
- デバイスへのアクセス権があるか
- `bitrate` と相手機器の設定が一致しているか
- `interface` が接続方式に合っているか
