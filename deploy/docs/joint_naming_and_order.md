# G1 关节命名与顺序对照（模型 / LeRobot / 宇树 SDK）

本文档说明在 `deploy`（g1_ctrl, G1-29dof）上运行时，不同来源的关节名与关节顺序
如何对应到宇树 SDK 的电机序，以及它们在名字写法、顺序、字段约定上的差异。

**核心结论**：本部署的关节序是 **SDK 电机序（恒等映射）**，即 `joint_ids_map = [0..28]`、
`motor_cmd()[i]` / `motor_state()[i]` / LowState 的 `motor_state[i]` 都指向同一个物理关节 `i`。
不同模型/框架只是用不同的名字和顺序描述同一个物理关节，接入时必须按文档对齐到 SDK 序。

---

## 1. 涉及的几套命名 / 顺序

| 体系 | 关节数 | 名字风格 | 顺序 | 用途 |
| --- | --- | --- | --- | --- |
| **宇树 SDK**（`g1_joint_sdk_names`） | 29 | `left_hip_pitch_joint`…`right_wrist_yaw_joint` | SDK 电机序（权威） | `motor_cmd`/`motor_state` 下标、`UrdfLimits`、部署配置数组顺序 |
| **LeRobot / lerobot 29dof**（`G1_29_JointIndex`） | 29 | `kLeftHipPitch.q`…`kRightWristYaw.q` | 与 SDK 序一致（仅名字不同） | lerobot rollout 主机读 LowState、组 action 字典 |
| **pi0.5 action**（`action_feature_names`） | 14 关节 + 4 摇杆 | `kLeftShoulderPitch.q`…`kRightWristYaw.q` + `remote.lx/ly/rx/ry` | 手臂 = SDK 臂部子集 `15..28`，左臂在前 | 6002 下发的 VLA 手臂目标 + 行走速度 |
| **Groot 全身策略**（`JointPositionAction`） | 15（腿+腰 `0..14`） | 数组，无名字 | 与 SDK 序一致 | 躯干平衡/行走；**不输出手臂** |

> 注意：pi0.5 的 `observation.state` 是 29 维（全身），但它的 **action 只有手臂 14 维 + 摇杆**。
> 手臂 15..28 在本部署里不由 Groot 策略驱动，而是由 6002 帧直接下发（见 §3）。

---

## 2. 完整对照表（SDK 电机序 0..28）

手臂列 `arm slot` 表示 6002 帧解析后的 `arm_q[]` 下标；
`dispatch` 表示该值最终写到的电机：`lowcmd->msg_.motor_cmd()[15 + slot]`。

| 电机 | 宇树 SDK 名 | LeRobot 29dof 名 | pi0.5 action（仅手臂） | 6002 arm slot / dispatch |
| ---: | --- | --- | --- | --- |
| 0 | left_hip_pitch_joint | kLeftHipPitch.q | — | — |
| 1 | left_hip_roll_joint | kLeftHipRoll.q | — | — |
| 2 | left_hip_yaw_joint | kLeftHipYaw.q | — | — |
| 3 | left_knee_joint | kLeftKnee.q | — | — |
| 4 | left_ankle_pitch_joint | kLeftAnklePitch.q | — | — |
| 5 | left_ankle_roll_joint | kLeftAnkleRoll.q | — | — |
| 6 | right_hip_pitch_joint | kRightHipPitch.q | — | — |
| 7 | right_hip_roll_joint | kRightHipRoll.q | — | — |
| 8 | right_hip_yaw_joint | kRightHipYaw.q | — | — |
| 9 | right_knee_joint | kRightKnee.q | — | — |
| 10 | right_ankle_pitch_joint | kRightAnklePitch.q | — | — |
| 11 | right_ankle_roll_joint | kRightAnkleRoll.q | — | — |
| 12 | waist_yaw_joint | kWaistYaw.q | — | — |
| 13 | waist_roll_joint | kWaistRoll.q | — | — |
| 14 | waist_pitch_joint | kWaistPitch.q | — | — |
| 15 | left_shoulder_pitch_joint | kLeftShoulderPitch.q | kLeftShoulderPitch.q | slot 0 / motor 15 |
| 16 | left_shoulder_roll_joint | kLeftShoulderRoll.q | kLeftShoulderRoll.q | slot 1 / motor 16 |
| 17 | left_shoulder_yaw_joint | kLeftShoulderYaw.q | kLeftShoulderYaw.q | slot 2 / motor 17 |
| 18 | left_elbow_joint | kLeftElbow.q | kLeftElbow.q | slot 3 / motor 18 |
| 19 | left_wrist_roll_joint | kLeftWristRoll.q | kLeftWristRoll.q | slot 4 / motor 19 |
| 20 | left_wrist_pitch_joint | kLeftWristPitch.q | kLeftWristPitch.q | slot 5 / motor 20 |
| 21 | left_wrist_yaw_joint | kLeftWristYaw.q | kLeftWristyaw.q* | slot 6 / motor 21 |
| 22 | right_shoulder_pitch_joint | kRightShoulderPitch.q | kRightShoulderPitch.q | slot 7 / motor 22 |
| 23 | right_shoulder_roll_joint | kRightShoulderRoll.q | kRightShoulderRoll.q | slot 8 / motor 23 |
| 24 | right_shoulder_yaw_joint | kRightShoulderYaw.q | kRightShoulderYaw.q | slot 9 / motor 24 |
| 25 | right_elbow_joint | kRightElbow.q | kRightElbow.q | slot 10 / motor 25 |
| 26 | right_wrist_roll_joint | kRightWristRoll.q | kRightWristRoll.q | slot 11 / motor 26 |
| 27 | right_wrist_pitch_joint | kRightWristPitch.q | kRightWristPitch.q | slot 12 / motor 27 |
| 28 | right_wrist_yaw_joint | kRightWristYaw.q | kRightWristYaw.q | slot 13 / motor 28 |

\* **拼写差异**：pi0.5 数据集把左侧手腕 Yaw 写作 `kLeftWristyaw`（小写 y），而
lerobot 运行时/线上帧用 `kLeftWristYaw`。两者是同一个关节；6002 解析按
**大小写不敏感**匹配，两种写法都接受。

---

## 3. 模型输出顺序与本部署的差别

### 3.1 名字写法的差别
- SDK 用全小写下划线：`left_shoulder_pitch_joint`。
- lerobot / pi0.5 用驼峰 `k` 前缀 + `.q` 后缀：`kLeftShoulderPitch.q`。
- 语义对应关系就是去掉前缀 `k`、大小写与分隔符规范化（见
  `deploy/scripts/check_joint_mapping.py` 的 `kname_to_sdk()`）。

### 3.2 顺序的差别
- SDK 与 lerobot 29dof 的顺序**逐位一致**（同一物理关节同一编号），只是名字不同。
- pi0.5 的 14 个手臂特征 **顺序**也与 SDK 臂部子集 `15..28` 逐位一致：先左臂 7 个
  （ShoulderPitch→WristYaw），再右臂 7 个。
- **腿/腰（0..14）不在 pi0.5 的 action 里**：由 Groot 全身策略内部驱动（策略只包含
  `joint_ids 0..14`）；`remote.lx/ly/rx/ry` 只表达行走速度，不直接写关节。

### 3.3 手臂下发闭环
6002 收到 LeRobot action 帧 → 按名字把 14 个值填进 `arm_q[slot]`
（slot = 电机号 − 15）→ `State_Groot::run()` 对 `motor_cmd()[15+slot]` 做
URDF 限位与速率限制后下发。因此：
```
arm_q[slot]  →  motor_cmd()[15+slot]  →  物理关节 = sdk_names[15+slot]
```

### 3.4 摇杆轴的差别（不是关节）
pi0.5 / LeRobot 帧把 4 个摇杆轴拼在 action 尾部：`remote.lx, remote.ly, remote.rx, remote.ry`。
本部署映射（与手柄约定一致）：`vx = remote.ly`、`vy = -remote.lx`、`wz = -remote.rx`；
`remote.ry` 保留不使用。不同模型可能自行定义轴序或符号，接入前需核对。

---

## 4. 其他易混淆点

- **LowState 有 35 个电机槽**（unitree `hg` 消息 `motor_state[0..34]`），本部署（29dof）
  只使用 `0..28`，`29..34` 保留（空/零）。6001 端口按 lerobot 协议原样广播 35 槽，
  lerobot 客户端也只读 `0..28`。
- **训练侧可能有置换**：某些模型/仿真管线（如 lerobot `g1_utils.py` 里的
  `ISAACLAB_TO_MUJOCO` / `MUJOCO_TO_ISAACLAB`）在 29dof 上定义了自己的
  isaaclab↔mujoco 排列。导出/接入这类模型时，先把模型关节向量置换回 SDK 序
  （本部署 `joint_ids_map` 为恒等）再运行，否则腿/腰/臂会对应错位。
- **验证工具**：`deploy/scripts/check_joint_mapping.py` 会把上面三份名单逐字读入，
  校验 lerobot↔SDK 语义转换、pi0.5 手臂列↔`15..28`、以及 deploy.yaml 的
  `joint_ids_map` 恒等与 `JointPositionAction.joint_ids == 0..14`，并打印上述 dispatch 表。

---

## 5. 单一事实来源

- 名字表：`deploy/include/groot/JointNameMap.h`（SDK 名 + lerobot `k` 名，按电机序）。
- 交叉校验：`deploy/scripts/check_joint_mapping.py`。
- 解析器：`deploy/include/groot/RemoteCommandReceiver.h`（6002，按名、大小写不敏感）。
- 状态广播：`deploy/include/groot/LowStateBroadcaster.h`（6001，rt/lowstate JSON）。
