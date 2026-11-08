# ARES 底盘交互控制

通过 CLI 或 Web GUI 输入 `vx vy wz amax jerkmax`，上位机生成加速度和 jerk 受限的速度指令，经 ARES USB 发送给下位机。无需定位、IMU 或模拟节点。启动后保持待命，不发送速度帧。

## 环境与构建

支持 Ubuntu 22.04 / ROS Humble 和 Ubuntu 24.04 / ROS Jazzy。安装对应 ROS 2、colcon 与 libusb 开发包后运行：

```bash
cd /home/ares/ares_prerobot_test
./scripts/build.sh
source /opt/ros/humble/setup.bash  # Ubuntu 24.04 使用 /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 启动

`src/ares_prerobot_test/config/default.yaml` 设置 USB 固件格式、Web 地址、控制权期限和速度上限。当前默认 `r2v2_4f`，对应现有 R2 的 `[vx,vy,wz,static]` 四 float 速度帧；若实际烧录三 float 固件，改为 `r2_3f`。下位机目标 DataID 沿用当前桥接约定。

```bash
ros2 launch ares_prerobot_test control.launch.py
```

另开终端运行 CLI：

```bash
source /opt/ros/humble/setup.bash
source /home/ares/ares_prerobot_test/install/setup.bash
ros2 run ares_prerobot_test chassis_cli
```

CLI 输入示例：`0.3 0 0 1.0 2.0`。也可输入 `status`、`stop`（受限减速）、`halt`（立即零目标）、`quit`（显式零目标后退出）。直接关闭终端或连接失效时不自动发送零目标。

Web GUI 默认仅监听机器人本机 `http://127.0.0.1:8766/`。在个人电脑上通过 SSH 转发访问：

```bash
ssh -L 8766:127.0.0.1:8766 ares@10.27.53.51
```

浏览器打开 `http://127.0.0.1:8766/`。CLI 和 Web GUI 共用一个控制权；先取得控制权，再发送目标。Web 页面关闭或控制连接失效后，控制权到期，停止发送。

## 指令定义

| 字段 | 单位 | 含义 |
| --- | --- | --- |
| `vx`, `vy` | m/s | 目标车体平面速度 |
| `wz` | rad/s | 目标角速度 |
| `amax` | m/s² | 上位机平面加速度上限 |
| `jerkmax` | m/s³ | 上位机平面 jerk 上限 |

`wz` 使用配置中的 `angular_accel` 和 `angular_jerk` 限幅。固件仍接收原有三或四 float 速度帧；`amax` 与 `jerkmax` 只在上位机生成指令时使用。Web 显示的“最近发送”是 USB 成功发送的目标值，不是车体实测速度。

## 断联行为

启动待命、控制端失联、控制权过期、USB 发送失败及进程退出时，上位机不补发零速度，不重放旧目标。桥在 50 ms 内未收到新速度时停止速度帧和协议心跳；恢复后须重新取得控制权并输入目标。只有操作员明确输入零目标、`stop` 或 `halt` 才会发送零速度。

**实车断联行为取决于下位机固件。** 当前仓库没有完整固件源码，尚不能确认下位机按速度帧超时还是按心跳超时处理旧目标。在验证固件行为前，不把“停止发送”视为“车体已停下”。

设计与实施安排见 `docs/plan.md`。
