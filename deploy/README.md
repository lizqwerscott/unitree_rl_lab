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

监听 TCP PULL 端口 `6002`。消息必须包含递增 `seq`、有限 `timestamp`、`remote` 和 14 维 `arm_q`：

```json
{
  "seq": 123,
  "timestamp": 1720000000.12,
  "remote": {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0},
  "arm_q": [0.1, 0.2, -0.3, 0.5, 0.0, 0.0, 0.0,
            -0.1, -0.2, 0.3, 0.5, 0.0, 0.0, 0.0]
}
```

轴映射为 `vx=ly`、`vy=-lx`、`wz=-rx`；`ry` 当前保留但不参与行走控制。消息会进行角度范围和序号检查，非法包直接丢弃；关节变化速率由 FSM 输出限速器统一处理。
