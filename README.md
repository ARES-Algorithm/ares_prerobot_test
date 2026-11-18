# ares_prerobot_test

ARES 底盘测试工具：网页输入运动目标，查看轮速、电流和 v/a/j 曲线，并自动录制 rosbag。支持 Ubuntu 22.04 / ROS 2 Humble、Ubuntu 24.04 / ROS 2 Jazzy。

## 构建并启动网页控制

连接底盘 USB，确认 `src/ares_prerobot_test/config/control.yaml` 中的 `firmware_profile` 与实际固件一致：`r2v2_4f` 为四 float 命令，`r2_3f` 为三 float 命令；程序不会自动识别。

```bash
cd /home/ares/Documents/ares_algo_ws/ares_prerobot_test
# Ubuntu 24.04 将 humble 换为 jazzy
source /opt/ros/humble/setup.bash
./scripts/build.sh
source install/setup.bash
ros2 launch ares_prerobot_test control.launch.py
```

浏览器打开 **http://127.0.0.1:8766/**。远程连接见[网页控制说明](docs/control.md)。

## 完成一轮操作

1. 点击“取得控制权”，确认 USB 已就绪。
2. 输入 `vx / vy / wz / amax / jerkmax`，点击“发送目标”，自动开始录包；后续修改目标继续写入同一个包。
3. 观察实时曲线；需要平滑降速时点击“受限减速”。
4. 点击“立即零目标”，等待页面显示“未录制”，从“本轮录包”处查看保存路径。

**启动程序不会录包；发送零速度、点击“受限减速”都不会结束录包。** 网页操作不需要调用 `/prerobot/start` 或 `/prerobot/stop` 服务。

默认录包保存在项目根目录的 `recordings/<时间_后缀>/`。目前通过 `ros2 bag play` 回放，网页尚不支持完整历史曲线回放，具体命令见[录制与回放](docs/recording.md)。

## 配置与说明

- 运动限值、轮半径、传动比和录包目录以 [control.yaml](src/ares_prerobot_test/config/control.yaml) 为准。输入单位：线速度 m/s、角速度 rad/s、加速度 m/s²、jerk m/s³。
- “立即零目标”是发送零目标，不代表车体瞬间停车；控制端失联或链路失效时停止发送，也不等于已停车。
- 网页控制与自动实验使用同一命令话题，不要同时运行。

| 文档 | 内容 |
|---|---|
| [网页控制](docs/control.md) | 参数、按钮、实时曲线、远程访问与 CLI |
| [录制与回放](docs/recording.md) | 录包启停、数据内容、终端回放与当前限制 |
| [自动实验与离线分析](docs/experiments.md) | 预设工况、start/stop 服务、CSV、分析报告和验证 |
