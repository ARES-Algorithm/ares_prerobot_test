# 自动实验与离线分析

[返回 README](../README.md)

本流程按配置执行预设运动，采集定位、IMU 和控制数据，再通过 CSV 生成分析报告。**网页手动发速度不需要启动本流程，也不需要调用 `/prerobot/start` 或 `/prerobot/stop`。**

先按 README 完成构建。不要与 `control.launch.py` 同时运行，以免多个节点向同一底盘发送指令。

## 模拟实验

```bash
cd /home/ares/Documents/ares_algo_ws/ares_prerobot_test
# Ubuntu 22.04 使用 humble；Ubuntu 24.04 将 humble 换为 jazzy
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ares_prerobot_test test.launch.py mode:=mock record:=true
```

另一终端进入项目根目录，执行相同的两条 source 命令后：

```bash
ros2 service call /prerobot/start std_srvs/srv/Trigger '{}'
# 中途结束本轮：
ros2 service call /prerobot/stop std_srvs/srv/Trigger '{}'
```

结果保存到 `results/<时间>/`。正常单轮完成后仍保留节点，可再次调用 start；参数在启动时读取，换工况请重启节点。

```bash
ros2 run ares_prerobot_test analyze results/实际轮次目录
# 改变求导窗口（秒），比较 jerk 对测量带宽的敏感性：
ros2 run ares_prerobot_test analyze results/实际轮次目录 --window 0.31
```

输出 `report.json`、`derived.csv`、`curves.png`。`results/bags/` 保存 launch 生命周期的原始话题录包，停止 launch 后完成索引。

## 实车接入

1. 确认车上应用及实际烧录版本。`r2v2_4f` 对应 `2026R2v2` 的 `[vx,vy,wz,static]`；`r2_3f` 对应三 float 的 R2 USB 应用。此选项决定发包长度，不自动识别固件。
2. 复制 `src/ares_prerobot_test/config/default.yaml` 为本轮配置，填写 `firmware_revision`、实际 `firmware_accel_limit`、定位/IMU话题、场地行程和本轮参数。
3. 用低速工况核对轴向与单位后填写 `calibration_confirmed: true`。`direction_deg` 是测量 map 坐标中的运动方向；`command_yaw_deg`、两个 `command_*_sign` 将它映射到固件命令坐标。上位机本身不额外套用去年应用的轴取负，映射应由实车标定确定。
4. 确保定位源提供 `PoseStamped`，header 为真实采样时间、时钟与主机 ROS 时钟对齐，frame 与 `pose_frame` 相符。默认 `/transformed/pose`；不同机器的时钟需先同步。
5. 启动测试通信和采集：

```bash
ros2 launch ares_prerobot_test test.launch.py mode:=hardware \
  firmware_profile:=r2v2_4f config:=/绝对路径/本轮配置.yaml record:=true
```

本 launch 提供底盘通信与实验节点。定位和 IMU 驱动应按实车原有方式启动。底盘测试期间由测试节点独占命令来源。SBUS 控制源选择和遥控接管沿用下位机实现。

USB：`1209:0001`，端点 IN `0x81` / OUT `0x01`。继承去年桥接的 DataID 字节序：逻辑 0x0111 在旧协议库 `send_sync` 前交换成 0x1101。桥接每 10 ms 发送最新目标；默认 50 ms 没有新鲜命令即停止发送速度帧并禁用协议心跳；退出桥接也不会补发零目标。停止发送不等于底盘已停车，下位机的超时行为需单独核对。发送失败后锁定为未就绪，重新启动桥接开启下一轮；不重放积压命令。

## 工况配置

| 参数 | 含义 |
|---|---|
| `kind: speed` | 按 acceleration 上升到 speed，保持 hold，再斜坡降速 |
| `kind: acceleration` | 上升段直接请求 speed 阶跃，用于观测固件及车体响应；下降段按 acceleration 降速 |
| `kind: jerk` | 对称 S 曲线，acceleration 和 jerk 是目标曲线约束 |
| `speed / acceleration / jerk` | m/s、m/s²、m/s³；都是目标参数 |
| `direction_deg` | map 中 0° 为 +X，90° 为 +Y，180° 为 -X |
| `prealign_speed / prealign_duration` | 平滑小速度脉冲完成预转向；期间会有小位移 |
| `max_distance` | 相对本轮起点的直线距离，含预转向；达到后进入停止阶段 |
| `max_duration` | 达到后进入停止阶段；须容纳正常测试曲线 |
| `stop_deceleration / stop_duration` | 人工或行程终止时的目标降速及零目标保持时间 |
| `feedback_timeout` | 定位和桥接状态有效期限；失效时立即请求零目标 |
| `notes` | 填写载荷、电池电压、地面、轮胎及试验编号 |

每轮完成一个参数点。先按方向扫描速度，再扫描加速度与 jerk；每点至少重复三次。将软件限幅、驱动器限幅和车体达到的指标分别记录。固件限幅的升高应在实车固件配置中完成，本项目配置中的 `firmware_accel_limit` 仅记录该值。

## 测量口径

- 主要指标由地面定位得到，轮速辅助识别空转。拒绝重复/倒序定位时间戳；分析拒绝超过 `--max-gap`（默认 0.15 秒）的定位间断。
- 根据实际时间戳三次样条重采样，再用四阶 Savitzky–Golay 局部拟合计算一至三阶导数；默认窗口约 0.21 秒。边界半窗口剔除，窗口和实际采样周期写入报告。
- `peak_*` 是整轮记录中的滤波峰值，包含预转向及停止。`peak_acceleration` 是平面加速度模长；前向及制动指标投影到所配置的 map 运动方向。
- 稳定速度仅在 hold 阶段寻找至少 0.5 秒窗口：速度极差不超过 0.03 m/s、沿运动方向加速度绝对值不超过 0.15 m/s²。未找到则 `stable_speed: null`。单轮平台是该工况结果，物理极限需要多点扫描判断。
- IMU 必须发布含重力的原始线加速度，姿态四元数将传感器坐标变换到 ENU 世界坐标；分析旋转后减去 9.80665 m/s²。默认分析 `imu_link`，可用 `--imu-frame` 指定。没有有效姿态时只输出定位推导指标。
- IMU 结果使用 IMU 自身时间戳；目前没有自动估计定位与 IMU 的时延或外参。姿态、安装位置、采样率会影响 IMU jerk。最大可跟踪目标 jerk 需要结合多轮目标/实测曲线判断，报告不自动把目标 jerk 当作测得上限。

## 文件和反馈

- `metadata.json` / `outcome.json`：配置及终止原因。
- `pose.csv` / `imu.csv`：原始值、采样时间及主机单调到达时间。
- `commands.csv` / `sent.csv`：生成的目标、USB 成功发送的目标；后者不代表下位机已执行。
- `usb_rx.csv`：全部 USB 同步反馈，保留协议库原始 DataID。
- `events.csv`：执行阶段标记。
- 可选 `firmware.csv`：下述补丁提供的限幅后目标和轮反馈。

## 下位机遥测补丁

`firmware/2026R2v2-telemetry.patch` 为此前的 `2026R2v2` 源码生成的独立补丁，不会随上位机构建自动应用或刷写。它增加 DataID 0x0131 的 100 Hz 遥测，保持现有命令和加速度限幅。

```bash
cd /实际路径/2026R2v2源码仓库
git apply --check /home/ares/Documents/ares_algo_ws/ares_prerobot_test/firmware/2026R2v2-telemetry.patch
# 确认应用版本匹配后，在固件开发分支应用：
git apply /home/ares/Documents/ares_algo_ws/ares_prerobot_test/firmware/2026R2v2-telemetry.patch
```

60 字节 payload = 15 个 float32：序号（24 位循环）、uptime 整秒、秒内微秒、驱动内部 set X/Y/gyro、wheel1..4 实测速度、wheel1..4 实测角度、加速度限幅。时间使用秒与微秒分开传输以保留精度；内部 X/Y 是驱动几何坐标。序号/uptime 是采样批次标签，各电机反馈来自最新 CAN 缓存，并非硬件同步采样；三个 set 字段是执行目标。

历史记录中该补丁通过过 `git apply --check`，尚未完成固件编译验证。使用前须在目标固件版本重新检查、编译并验证。此旧补丁的 15 float 格式供实验节点生成 `firmware.csv`，与网页使用的 11 float 转速包不兼容。下位机时钟与主机的自动同步、轮速打滑分类不包含在当前报告中。

## 验证

以下命令在项目根目录执行，先完成构建；端到端脚本使用独立 ROS 域，确保示例域未被实车使用。

```bash
# Ubuntu 24.04 将 humble 换为 jazzy
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH="$PWD/src/ares_prerobot_test:$PYTHONPATH" /usr/bin/python3 -m pytest src/ares_prerobot_test/test -q
ROS_DOMAIN_ID=217 /usr/bin/python3 scripts/check_pipeline.py
ROS_DOMAIN_ID=218 /usr/bin/python3 scripts/check_launch.py
```

端到端验证使用模拟地面运动和 IMU，覆盖正常完成、人工停止、反馈中断、重新开始和生成报告。`results/verification*` 的数值为模拟验证数据。
