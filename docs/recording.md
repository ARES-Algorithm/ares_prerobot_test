# rosbag 录制与回放

[返回 README](../README.md)

## 网页控制如何录制

`control.launch.py` 默认启用按轮次录包，不需要传 `record:=true`，也不需要调用实验服务。

| 时机 | 是否录制 |
|---|---|
| 启动程序、取得控制权 | 不开始录制 |
| 首次“发送目标”（包括全零目标） | 在发布目标前创建 rosbag，创建失败则拒绝此次目标 |
| 后续修改目标、发送全零目标 | 继续写入当前包 |
| “受限减速”完成 | 继续录制遥测，不关包 |
| “立即零目标” | 写入 5 帧零指令，停止发送后约 0.2 秒关包 |
| 释放控制、租约失效、运行中 USB 失联、正常关闭控制器 | 结束当前录制 |

上一轮关闭后，再次“发送目标”会创建新包。关包后页面显示“未录制”并保留最近保存路径。强制杀进程或断电不保证正常完成索引。

默认保存到启动目录下的 `recordings/<时间_随机后缀>/`。从项目根目录启动时，例如：

```text
recordings/20261119_120000_123456_a1b2c3/
├── metadata.yaml
└── …_0.db3
```

修改 [control.yaml](../src/ares_prerobot_test/config/control.yaml) 的 `bag_directory` 可以指定绝对目录。

## 包内数据

| 话题（均以 `/prerobot/` 开头） | ROS 类型 | 内容 |
|---|---|---|
| `cmd_vel` | `geometry_msgs/msg/TwistStamped` | 控制器实际发布的指令 |
| `sent_cmd` | `geometry_msgs/msg/TwistStamped` | USB 桥接成功发送的指令，不代表下位机已执行 |
| `usb_rx` | `std_msgs/msg/Float32MultiArray` | 原始 USB 反馈，DataID 保存在 `layout.data_offset` |
| `chart_command` | `std_msgs/msg/String` | JSON：指令速度及派生 a/j |
| `chart_wheel` | `std_msgs/msg/String` | JSON：四轮各自的 RPM、轮缘速度及派生 a/j |
| `chart_current` | `std_msgs/msg/String` | JSON：四轮 Id/Is 及各自均值 |
| `recording_event` | `std_msgs/msg/String` | JSON：录制开始、结束及结束原因 |

存储格式为 SQLite3，序列化格式为 CDR。bag 时间戳使用主机系统时间；图表 JSON 的 `t`、`received` 使用控制器主机的单调时钟域，派生值另带窗口中心时间。离线对齐时不要把 JSON 时间直接当作 Unix 时间。

## 回放步骤

先点击“立即零目标”，等页面显示“未录制”，再关闭实车控制 launch。以下回放使用独立 ROS 域；示例 `219` 应选为未用于实车的域，所有回放和查看终端使用同一个值。

### 1. 查看包信息

```bash
cd /home/ares/Documents/ares_algo_ws/ares_prerobot_test
source /opt/ros/humble/setup.bash  # Ubuntu 24.04 使用 jazzy
export ROS_DOMAIN_ID=219

find recordings -name metadata.yaml
# 将下面路径替换为页面显示的实际包目录
BAG="$PWD/recordings/实际包目录"
ros2 bag info "$BAG"
```

`BAG` 指向含 `metadata.yaml` 的目录。检查时长、话题和消息数量；消息数为零表示该话题没有录到数据。

### 2. 另一终端先监听

例如查看电流数据：

```bash
source /opt/ros/humble/setup.bash  # Ubuntu 24.04 使用 jazzy
export ROS_DOMAIN_ID=219
ros2 topic echo /prerobot/chart_current std_msgs/msg/String
```

将话题换为 `chart_command`、`chart_wheel` 或 `recording_event` 可查看其他 JSON 数据。先监听再播放，避免错过包开头的数据。

### 3. 播放记录

回到设置了 `BAG` 的终端：

```bash
ros2 bag play "$BAG" --topics \
  /prerobot/chart_command \
  /prerobot/chart_wheel \
  /prerobot/chart_current \
  /prerobot/recording_event
```

默认原速播放。可增加 `--rate 0.5` 半速播放、`--loop` 循环播放；`Ctrl+C` 停止。

需要回放原始遥测时，可在话题列表中加入 `/prerobot/usb_rx`。不指定 `--topics` 会播放全部记录，其中包括控制指令话题，应继续使用与实车隔离的域。

## 当前回放限制

- 终端能回放、查看数据；网页没有历史包选择和播放入口。
- 当前网页不订阅 `chart_*` 话题，不能通过上述命令完整重现录制时的曲线。仅回放 `usb_rx` 也无法恢复原始指令曲线。
- `ros2 run ares_prerobot_test analyze` 读取自动实验生成的 CSV，不直接读取网页 rosbag。

## 与自动实验录包的区别

| 模式 | 开始方式 | 输出 |
|---|---|---|
| 网页 / CLI 控制 | `control.launch.py` 后首次发送目标 | 按轮次保存到 `recordings/` |
| 自动实验 CSV | `test.launch.py` 后调用 `/prerobot/start` | 每轮 `results/<时间>/` |
| 自动实验 rosbag | 启动 `test.launch.py record:=true` | launch 生命周期录制到 `results/bags/`，正常退出 launch 后完成索引 |

自动实验的 `/prerobot/stop` 服务只提前结束该轮实验，不结束 launch 级 rosbag。自动实验详情见[自动实验与离线分析](experiments.md)。
