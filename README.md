# velocity_manager

複数の `geometry_msgs/msg/Twist` 入力をサービスで順次切り替え、単一出力へパススルーする ROS 2 ノードです。

## Build

```bash
cd ~/cursor_ws/test_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select velocity_manager
source install/setup.bash
```

## Launch

```bash
ros2 launch velocity_manager velocity_manager.launch.py
```

## Topics (publish)

| Topic | Type | Behavior |
|---|---|---|
| `cmd_vel` (configurable) | `geometry_msgs/msg/Twist` | 選択中ソースのパススルー、または停止時のゼロ指令 |
| `/vel_mng/enabled` | `std_msgs/msg/Bool` | `true`=enable / `false`=disable。起動時と状態変化時に配信（transient_local） |

## Services

| Service | Type | Behavior |
|---|---|---|
| `/vel_mng/switch` | `std_srvs/srv/Trigger` | 次の入力ソースへローテーション。`message` に切替後ソース名 |
| `/vel_mng/enable` | `std_srvs/srv/SetBool` | `false`: 即ゼロ出力してブロック / `true`: 再開（選択ソース維持） |

起動時の enable は `false` です。出力するには先に enable してください。

## Parameters

ROS 2 の制約により、要求書の `inputs: [{name, topic}, ...]` は並列配列で表現します（順序＝switch 順）。

- `output_topic` (string)
- `default_input` (string) — 存在しない場合は起動失敗
- `input_names` (string[])
- `input_topics` (string[])

## Smoke test

```bash
./src/velocity_manager/scripts/smoke_test.sh
```
