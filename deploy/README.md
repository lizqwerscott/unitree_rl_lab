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

在 `Navigation` 和 `VLA` 模式下，手柄任一速度轴绝对值超过 `0.05` 时，手柄速度临时覆盖外部速度指令；松开摇杆后恢复当前模式的外部指令。`Stand` 模式仍会强制速度为零。

Groot 的 `safe_home_q` 手臂关节（15–28）遵循 LeRobot 的默认零位；进入 VLA 后，Pi0.5 的手臂指令直接作为目标下发，不做过渡插值；每条指令都会与实测姿态逐关节比较，任一关节偏差超过可配置阈值（`FSM.Groot.vla.max_arm_deviation_deg`，默认 `40°`）就不接管并退回 `Gamepad`（按退出 VLA 的插值回到安全姿态）＋打印告警日志。下肢默认位仍使用 Groot 的策略默认值。

推理线程以 50 Hz 运行，FSM 线程以 1 kHz 合并并发布唯一的 `LowCmd`。模型已从 Hugging Face 仓库 `nepyope/GR00T-WholeBodyControl_g1` 下载，运行时不会再从网络下载模型。

## Groot PD 增益

`deploy/robots/g1_29dof/config/policy/groot/params/deploy.yaml` 中的 `stiffness` / `damping`
按 SDK 的 29 关节顺序配置，并与 LeRobot
`src/lerobot/robots/unitree_g1/config_unitree_g1.py` 的 `_GAINS` 保持一致。进入 `Groot`
时，`State_Groot` 会将该组参数写入全部电机命令。

| 关节组 | Kp | Kd |
| --- | ---: | ---: |
| 左右腿 hip pitch / roll / yaw | 150 | 2 |
| 左右腿 knee | 300 | 4 |
| 左右腿 ankle pitch / roll | 40 | 2 |
| waist yaw / roll / pitch | 250 | 5 |
| 左右 shoulder pitch / roll | 50 | 3 |
| 左右 shoulder yaw / elbow | 80 | 3 |
| 左右 wrist roll / pitch / yaw | 40 | 1.5 |

不要将 `FixStand` 的独立过渡增益误作为 Groot 的策略增益；两者分别由
`FSM.FixStand` 和 Groot `deploy.yaml` 配置。

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
| `RB + A` | **手动松爪**：两侧夹爪目标 → `gripper.q_max`（按 `max_rate_rad_per_s` 爬坡张开） |

键盘备用按键：

| 按键 | 操作 |
| --- | --- |
| `1` | 切换 `Gamepad` |
| `2` | 切换 `Navigation`；没有新鲜导航包时速度为零 |
| `3` | 切换 `VLA`；没有新鲜 ZMQ 包时速度为零、手臂保持安全姿态 |
| `m` | `Auto`：速度范数小于 `0.05` 使用 balance，否则使用 walk |
| `p` | `Stand`：速度强制为零并使用 balance |
| `g` / `G` | **手动松爪**（同 `RB + A`） |

高度控制：

| 按键 | 操作 |
| --- | --- |
| 手柄十字键 `↑` | 高度增加 `0.001 m` |
| 手柄十字键 `↓` | 高度降低 `0.001 m` |
| 手柄 `RB + X` | 恢复默认高度 `0.74 m` |
| 键盘 `↑` / `↓` | 高度增加 / 降低 `0.001 m` |
| 键盘 `r` | 恢复默认高度 `0.74 m` |

高度范围限制为 `0.50–1.00 m`。

进入 `VLA` 后，首个有效 `arm_q` 默认直接作为手臂目标下发，不再使用五阶 Bezier 轨迹插值：手臂按 `publish_targets` 中各关节的 URDF 速度上限逐周期逼近目标，位移大时表现为以关节速度上限运动。

**偏差保护**：阈值可配置，见 `config.yaml` 的 `FSM.Groot.vla.max_arm_deviation_deg`（单位：度，默认 `40`，有效范围 `1..180`；设为 `0` 或负数表示关闭该保护）。`VLA` 内**每一条**有效包都拿目标与**实测关节角**逐关节比较，任一关节超过阈值就判定这条目标会让手臂大幅跳变，**不接管手臂**，直接退出到 `Gamepad`（由退出 VLA 的既有插值把手臂平滑带回安全姿态），并打印一条 `warn` 日志，逐关节列出所有超限关节（SDK 关节名、电机序号、偏差角度）：

```yaml
    vla:
      max_arm_deviation_deg: 40   # 0 或负数 = 关闭该保护
```

```text
Groot VLA: arm target deviates more than 40 deg from the measured pose on 2 joint(s), exiting to Gamepad: left_shoulder_pitch_joint(motor 15) +52.5 deg right_elbow_joint(motor 25) -41.7 deg
```

启动时会打印生效值：`Groot VLA: arm deviation guard 40 deg (...)`；关闭时打印 `Groot VLA: arm deviation guard disabled (...)`。

这是刻意的"目标必须与机器人实际姿态相容"闸门，因此有一个必然结果：进入 VLA 时手臂停在 `safe_home_q`（手臂段为零位），若策略当前目标离它超过阈值（例如肘 `1.0 rad ≈ 57°`，默认阈值下就会超），**这一条就不会被接管，模式当拍退回 `Gamepad`**，VLA 进不去。要让 VLA 能接管，需满足其一：

- 进入 VLA 时策略目标与手臂实际姿态相差在阈值以内（例如把 `safe_home_q` 手臂段设成策略的起始位姿，或先摆好手臂）；
- 调大 `FSM.Groot.vla.max_arm_deviation_deg`。

保护只在 `VLA` 模式下生效，且每次触发后模式已退回 `Gamepad`，不会重复刷日志；退出到 `Gamepad` 后需要手动按键才能重新进入 `VLA`。

**退出后回安全姿态期间禁止进入 `VLA`**：一旦离开 `VLA`（手动切换或偏差保护回退），手臂会按 Bezier 插值回到 `safe_home_q`；在这段过渡跑完之前，按 `VLA` 会被拒绝（`request_mode()` 返回 `false`，日志为 `Groot: VLA entry ignored, arm is still returning to safe_home_q`），避免被取消的过渡与 VLA 目标抢同一个手臂。过渡完成后即可正常进入。若离开 VLA 时手臂本来就在 `safe_home_q`，过渡时长为零，可以立即进入。

首帧到达前手臂保持安全姿态；该首帧指进入 VLA 之后收到的新序号指令，进入前缓存的旧包不会被采用。VLA 指令超时后，下肢速度归零，手臂持续保持最后一条有效 `arm_q`，并持续保持手臂位置刚度，不会自动回到 `safe_home_q`。反向切换到 `Gamepad` 或 `Navigation` 时，手臂仍按 `arm_transition.velocity_scale`（默认 `0.02`）的 Bezier 轨迹回到安全姿态。

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

### 控制状态 / ZMQ

进入 `Groot` 后，控制器还会通过 ZMQ `PUB`（端口 `6000`，配置 `Groot.zmq.control_state_port`）以 **50 Hz** 持续广播当前操控状态，供上位机（VLA 推理、导航、遥操作 UI）判断当前由哪一路输入接管机器人：

```json
{
  "state": "vla"
}
```

`state` 只有三个取值，与 [`GrootModeManager.h`](include/groot/GrootModeManager.h) 的 `ControlMode` 一一对应：

| 控制模式 | `state` |
| --- | --- |
| `Gamepad` | `gamepad` |
| `Navigation` | `nav` |
| `VLA` | `vla` |

之所以定频重发而不是仅在切换时发一条：`PUB` 不保存历史，晚接入的订阅方在下次切换前收不到任何东西；50 Hz 心跳让订阅方在一个周期（20 ms）内就能拿到当前状态。无订阅者时静默丢弃，发送为 `dontwait`，不会阻塞控制线程。状态读取自 `ControlMode`，`Stand` / `Auto` 属于移动模式（`LocomotionMode`），不体现在该字段。

命令行自测（需要 `python3-zmq`）：

```bash
python3 -c "
import zmq
s = zmq.Context().socket(zmq.SUB)
s.setsockopt_string(zmq.SUBSCRIBE, '')
s.connect('tcp://127.0.0.1:6000')
while True: print(s.recv_string())
"
```

### Dex1_1 夹爪 / ZMQ

夹爪沿用"对外全 ZMQ、内部 DDS"的范式：**下行目标复用 6002 的 action 帧**（新增一个可选
`gripper` 块，不新开端口），**上行状态新开 PUB `6004`**。桥接层是忠实的协议转换器，只搬运
硬件原始量纲（弧度），不做"张开/闭合"之类的语义换算——语义映射属于上位机适配层。

#### 下行：6002 action 帧内的可选 `gripper` 块

在原有 14 个手臂关节 + `remote.*` 之外，可选地多带一个 `gripper` 键：

```json
{
  "cmd": "action",
  "action": {
    "kLeftShoulderPitch.q": -0.20, "kRightWristYaw.q": -0.01,
    "remote.lx": 0.0, "remote.ly": 0.0, "remote.rx": 0.0, "remote.ry": 0.0,
    "gripper": { "right": { "q": 0.0 }, "left": { "q": 5.0 } }
  },
  "timestamp": 1788514855.53
}
```

| 字段 | 必填 | 说明 |
| --- | --- | --- |
| `gripper` | 否 | 整块可选；缺失则该帧不产生任何夹爪更新，其余照旧 |
| `gripper.<side>` | 否 | `right` / `left` 各自可选；**只给一侧时另一侧保持上次目标**（latch） |
| `gripper.<side>.q` | 是 | 目标角度，**弧度**，输出侧量纲；必须是有限值（`NaN`/`inf` 该侧作废） |

**帧里只有 `q`**。`kp` / `kd` 是机器人侧的硬件整定参数，**唯一来源是 `config.yaml` 的
`FSM.Groot.gripper`**（默认 `5.0` / `0.05`，与 `dex1_1_service/test/test_gripper.cpp` 的参考值一致）；
`mode` 固定为 `1`（FOC）。帧内出现 `kp`/`kd`/`mode` 会被**忽略并打一条 warn**——这样做是为了
避免上位机（尤其是 VLA 策略）每帧重复传常量、或在运行中把刚度改坏。

**量程是 clamp，不是丢帧**。`q` 超出 `FSM.Groot.gripper.q_min`/`q_max`（默认 `0.0` / `5.6217 rad`，
即 `322°` 标定值）时，夹爪**不会**被拒绝：和手臂 `publish_targets()` 一样，先做绝对限位、再做逐周期速率限位，
并打印一条日志（同一侧持续超限只打一次，回到范围内后再次超限会重新打）：

```text
[GripperBridge] right q=9.9000 rad is outside the calibrated range [0.0000, 5.6217]; clamped to 5.6217
```

**这里的 clamp 是唯一的机械限位保护**：`dex1_1_service` 的 `loop_()` 只做 `q * gear_ratio` 后直接发串口，
**不做任何范围校验**。因此进入 `Groot` 前**必须现场实测一次**"完全闭合 / 完全张开"的读数，据此设定 `q_min` / `q_max`；
照搬默认值有顶到机械限位的风险。

三条必须记住的规则：

- **量程独立**：手臂沿用 `|q| <= 3.2` 整帧丢弃，夹爪走上面的 clamp，两者互不影响——夹爪的合法大值不会拖累手臂，手臂的超限也不会被夹爪的 clamp "救回来"。
- **失败隔离**：夹爪块非法（非 map、缺 `q`、类型错误、非有限值）只降级为"本帧不更新夹爪"并打一条 warn，**手臂照常生效**。
- **手臂 14 关节仍然必填**：不存在"只发夹爪"的帧。想合一下爪子也要把 14 个手臂关节一起重发（用当前/上一拍的值即可）。

桥接层**逐周期限速**（`FSM.Groot.gripper.max_rate_rad_per_s`，默认 `6.0 rad/s`，约 `1.7 s` 走完全行程），
所以跳变指令表现为**渐变**逼近而非瞬跳。

#### 上行：6004 夹爪实测状态

进入 `Groot` 后通过 ZMQ `PUB`（端口 `6004`，配置 `Groot.zmq.gripper_state_port`）以 **100 Hz** 广播：

```json
{
  "topic": "rt/dex1/state",
  "data": {
    "right": { "q": 0.501, "dq": 0.0, "tau_est": 0.02 },
    "left":  { "q": 0.500, "dq": 0.0, "tau_est": 0.02 }
  }
}
```

字段直接透传 DDS `MotorStates_.states()[0]` 的 `q / dq / tau_est`，**不做单位换算**；数值用 `%.9g`，
非有限值归零；无订阅者时静默丢弃，发送为 `dontwait`，不会阻塞。

```bash
python3 -c "
import zmq
s = zmq.Context().socket(zmq.SUB); s.setsockopt_string(zmq.SUBSCRIBE, '')
s.connect('tcp://127.0.0.1:6004')
for _ in range(3): print(s.recv_string())
"
```

#### 命令行客户端 `deploy/scripts/groot_client.py`

需要 `pyzmq`（`pip3 install --user pyzmq` 或 `uv pip install pyzmq`）。

```bash
# 订阅并打印：夹爪状态（6004）+ 控制模式（6000）+ 可选手臂 14 关节（6001）
./deploy/scripts/groot_client.py watch
./deploy/scripts/groot_client.py watch --arms

# 下发夹爪目标（向 6002 发一条完整 action 帧）
./deploy/scripts/groot_client.py grip --right 5.0            # 右爪张开
./deploy/scripts/groot_client.py grip --right 0.0 --left 0.0 # 双爪闭合
./deploy/scripts/groot_client.py grip --right 5.0 --follow   # 下发后继续观察
```

`grip` 的几点说明：

- **手臂 14 关节默认填 0**（= Groot 的 `safe_home`；`publish_targets()` 还会再过一遍 URDF 限幅，不会顶限位），所以不依赖 6001 也能用。
- `--echo` 改为用 6001 的**实测姿态**回填手臂。**当前是 VLA 模式时建议加它**：VLA 下 `update_vla()` 会拿帧里的 `arm_q` 与实测姿态逐关节比较，偏差超过 `FSM.Groot.vla.max_arm_deviation_deg`（默认 `40°`）就判定该目标会让手臂跳变，**不接管手臂并退出到 `Gamepad`**。填 0 时若手臂不在零位就会触发这一点；脚本检测到 `vla` 且未加 `--echo` 时会给出警告（不拦截）。
- `remote.lx/ly/rx/ry` 填 0，因此在 VLA 模式下这条帧会让速度短暂归零（VLA 主机的下一条帧约 20 ms 内覆盖）。
- 帧里超量程的 `q` 只打印提示，真正的 clamp 由 `GripperBridge` 执行。
- 夹爪目标下发**一条帧就够**：桥接层会 latch 并以 100 Hz 定频重发（`--repeat` 默认 3 只是为了抗丢包）。

夹爪目标与当前控制模式无关，`Gamepad` / `Navigation` / `VLA` 下都会生效（arm 通路才受模式约束）。

#### 保持行为（重要）

- **100 Hz 无条件重发**：`dex1_1_service` 有超时看门狗，指令流一断就切 `BRAKE` 并清零。因此即使没有新指令、即使 6002 静默，桥接层也持续按**最后的目标**重发——这正是"上位机暂停 / VLA 超时"期间**保住抓取力**的机制。
- **指令超时（`FSM.Groot.gripper.command_timeout_s`，默认 `0.5 s`）后不松爪**：保持最后目标继续重发，不回零、不 BRAKE；该参数只用于打一条日志提示。
- **首个夹爪目标之前不发任何指令**：进入 `Groot` 后、尚未收到任何带 `gripper` 的帧之前，桥接层保持静默（不发零位、不发默认值），只打一条日志说明在等待首个目标——避免给夹爪一个未经上位机确认的意外动作。该规则**每次进入 `Groot` 都重新成立**：退出再进入时不会用上一会话的旧目标自动恢复抓取，必须等上位机重新下发一帧。
- **刻意与手臂状态机解耦**：夹爪不参与 `GrootModeManager` 的 VLA 偏差保护、Bezier 回归安全姿态与 `arm_transition.velocity_scale`。
- **退出 VLA 不自动松爪**：`LB+X` 切回 `Gamepad`、或偏差保护触发退出 VLA 时，夹爪都**保持**最后目标继续重发。这是刻意的——偏差保护触发说明 VLA 策略正在输出离谱的手臂目标，而那恰恰是最可能正握着东西的时刻，此时自动张开等于主动掉件。
- **松爪是显式动作**：手柄 `RB + A` 或键盘 `g`，把两侧目标压到 `gripper.q_max`，且仍走限速爬坡（不会瞬间弹开）。它和帧目标共用同一套 latch + 限速路径，所以**上位机的下一条带 `gripper` 的帧会立刻接管**（夹爪语义仍属于上位机）。
- **退出 `Groot`（`LT + B` → `Passive`）时桥接层停止**：与手臂"平滑回安全姿态"不同，夹爪不会被命令到某个安全位置；但桥接线程停止后 100 Hz 重发也停止，`dex1_1_service` 的超时看门狗随后会接管该话题（切 `BRAKE`）。抓取力的保持因此只在 `Groot` 状态内成立，这是当前实现的既有边界。

### 输入源小结

| 端口 | 方向 | 用途 |
| --- | --- | --- |
| `6000` | 本机 `PUB` | 广播当前控制状态（50 Hz） |
| `6001` | 本机 `PUB` | LowState 广播（约 500 Hz） |
| `6002` | 本机 `PULL` | 接收 VLA `action` 帧（含可选 `gripper` 块） |
| `6004` | 本机 `PUB` | Dex1_1 夹爪实测状态广播（100 Hz） |
