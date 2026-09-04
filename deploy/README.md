# Deploy 使用说明

本文档说明当前 G1-29DoF Groot 控制器的构建、启动和按键操作。

## 当前状态

G1 的 FSM 状态为：

```text
Passive -> FixStand -> Groot
```

`Groot` 内部包含三种控制模式：

- `Gamepad`：手柄速度控制下肢，手臂回到 Groot 上游定义的零姿态 `safe_home_q`。
- `Navigation`：使用 `rt/nav_cmd` 的 `vx/vy/wz`，手臂回到 Groot 上游定义的零姿态 `safe_home_q`。
- `VLA`：使用 ZMQ 6002 端口输入速度和 14 维手臂目标。

Groot 的 `safe_home_q` 手臂关节（15–28）遵循 LeRobot 的默认零位，便于 Pi0.5 的第一条有效手臂指令直接接管；下肢默认位仍使用 Groot 的策略默认值。

推理线程以 50 Hz 运行，FSM 线程以 1 kHz 合并并发布唯一的 `LowCmd`。模型已从 Hugging Face 仓库 `nepyope/GR00T-WholeBodyControl_g1` 下载，运行时不会再从网络下载模型。

## 模型文件

Groot ONNX 模型已放置在：

```text
deploy/robots/g1_29dof/config/policy/groot/exported/
├── GR00T-WholeBodyControl-Balance.onnx
└── GR00T-WholeBodyControl-Walk.onnx
```

模型来源：`nepyope/GR00T-WholeBodyControl_g1`，每个文件约 1.8 MB。模型必须是本地文件，输入 shape 为 `[1, 516]`，输出 shape 为 `[1, 15]`。缺少模型或 shape 不匹配时，程序会在进入 Groot 状态前报错退出；不要将其他 29DoF velocity policy 重命名替代。

## 系统依赖

Ubuntu/Debian 至少需要：

```bash
sudo apt update
sudo apt install -y \
  libboost-program-options-dev \
  libyaml-cpp-dev \
  libtinyxml2-dev \
  libzmq3-dev \
  cppzmq-dev
```

此外还需要 Unitree SDK2、DDS/iceoryx、Eigen3、fmt，以及仓库内对应架构的 ONNX Runtime。

如果 `cppzmq-dev` 无法找到，先启用 `universe` 仓库：

```bash
sudo add-apt-repository universe
sudo apt update
```

## 构建和启动

在 G1-29DoF 目录下执行：

```bash
cd deploy/robots/g1_29dof
cmake -S . -B build
cmake --build build -j$(nproc)
./build/g1_ctrl --network eth0 --domain_id 0
```

启动前请确认机器人未被其他进程占用 `LowCmd` 通道，并确认 `main.urdf` 能解析出 29 个可控关节。

## 按键

### FSM 状态

| 按键 | 操作 |
| --- | --- |
| `LT + Up` | `Passive -> FixStand` |
| `RB + X` | `FixStand -> Groot` |
| `LT + B` | `Groot -> Passive` |

### Groot 模式

手柄组合键：

| 按键 | 操作 |
| --- | --- |
| `LB + X` | 切换 `Gamepad` |
| `LB + Y` | 切换 `Navigation` |
| `LB + A` | 切换 `VLA` |
| `RB + Y` | 切换 `Auto` |
| `RB + B` | 切换 `Stand` |

键盘备用按键：

| 按键 | 操作 |
| --- | --- |
| `1` | 切换 `Gamepad` |
| `2` | 切换 `Navigation`；没有新鲜导航消息时拒绝切换 |
| `3` | 切换 `VLA`；没有新鲜且完整的 ZMQ 包时拒绝切换 |
| `m` | `Auto`：速度范数小于 `0.05` 使用 balance，否则使用 walk |
| `p` | `Stand`：速度强制为零并使用 balance |

从 `VLA` 切换到 `Gamepad` 或 `Navigation` 时，手臂从当前实际角度以默认 500 ms 五阶 Bezier 轨迹回到安全姿态。其他模式切换不会触发额外手臂轨迹；进入 VLA 后首个有效 `arm_q` 到达前仍保持安全姿态。

## 外部输入

### Navigation

订阅 `rt/nav_cmd`（`geometry_msgs/Twist`）：

```text
linear.x -> vx
linear.y -> vy
angular.z -> wz
```

默认超时为 300 ms；超时后速度置零，模式保持 `Navigation`，消息恢复后自动继续。

### VLA / ZMQ

监听 TCP PULL 端口 `6002`。当前采用 LeRobot `action` 帧格式：`action` 内含 14 个具名手臂关节（`<名字>.q`）与 `remote.lx/ly/rx/ry`，外加 `timestamp`（发送侧单调递增）：

```json
{
  "cmd": "action",
  "action": {
    "kLeftShoulderPitch.q": -0.206, "kLeftShoulderRoll.q": 0.540,
    "kLeftShoulderYaw.q": 0.287,    "kLeftElbow.q": -0.253,
    "kLeftWristRoll.q": 0.131,      "kLeftWristPitch.q": -0.044,
    "kLeftWristYaw.q": 0.274,
    "kRightShoulderPitch.q": -0.544, "kRightShoulderRoll.q": -0.519,
    "kRightShoulderYaw.q": -0.179,   "kRightElbow.q": 0.050,
    "kRightWristRoll.q": -0.091,     "kRightWristPitch.q": 0.030,
    "kRightWristYaw.q": -0.013,
    "remote.lx": 0.0, "remote.ly": 0.0, "remote.rx": -0.175, "remote.ry": 0.0
  },
  "timestamp": 1788514855.53
}
```

映射规则：
- 手臂关节按名映射到电机序号 `15..28`（左臂 7 个在前、右臂 7 个在后），每项必须是有限值且 `|v|<=3.2`，任一缺失即整包丢弃。
- 摇杆轴映射 `vx=remote.ly`、`vy=-remote.lx`、`wz=-remote.rx`；`remote.ry` 保留不参与行走。
- 该协议没有 `seq`，改用 `timestamp` 作为单调门限：`timestamp` 必须大于上一条，否则判为重复/乱序丢弃。
- 关节变化速率由 FSM 输出限速器统一处理。

> 关节序对应表见 `deploy/include/groot/JointNameMap.h`；不同模型（pi0.5 / LeRobot / Groot）与宇树 SDK 的关节名与顺序差异见 `deploy/docs/joint_naming_and_order.md`；可用 `deploy/scripts/check_joint_mapping.py` 交叉核对。

进入 `Groot` FSM 后，机器人还会通过 ZMQ `PUB`（端口 `6001`，配置 `Groot.zmq.state_port`）广播实时 LowState，供上位机（如 LeRobot rollout）读取本体状态。载荷与 LeRobot `rt/lowstate` 桥接协议一致：

```json
{
  "topic": "rt/lowstate",
  "data": {
    "motor_state": [{"q": 0.1, "dq": 0.0, "tau_est": 0.0, "temperature": 30.0}],
    "imu_state": {"quaternion": [1, 0, 0, 0], "gyroscope": [0, 0, 0],
                  "accelerometer": [0, 0, 0], "rpy": [0, 0, 0], "temperature": 0},
    "wireless_remote": "<base64>",
    "mode_machine": 5
  }
}
```

`motor_state` 固定 35 项（与 unitree `hg` LowState 布局一致），约 500 Hz 广播；无订阅者时静默丢弃。
