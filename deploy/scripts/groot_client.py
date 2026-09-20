#!/usr/bin/env python3
"""
Groot 夹爪 / 状态的命令行客户端。

两个子命令：

  watch   订阅并打印机器人状态
          6004 夹爪实测（rt/dex1/state JSON）
          6000 当前控制模式（{"state":"gamepad"|"nav"|"vla"}）
          6001 全身 LowState（可选，`--arms` 时只打印手臂 14 关节）

  grip    向 6002 下发一条**完整的** LeRobot action 帧，只改动夹爪目标
          手臂 14 关节默认填 0（= Groot 的 safe_home，publish_targets 还会过一遍
          URDF 限幅，不会顶限位），所以不需要 6001 也能用。
          夹爪目标会被桥接层 latch，并由 Bridge 以 100 Hz 定频重发，所以
          **一条帧就够**，不需要持续发（`--repeat` 只是为了抗丢包）。

          `--echo` 改为用 6001 的实测姿态回填手臂 14 关节。**当前是 VLA 模式时建议
          开启**：VLA 下 update_vla() 会拿帧里的 arm_q 和实测姿态逐关节比，偏差超过
          FSM.Groot.vla.max_arm_deviation_deg（默认 40°）就判定该目标会让手臂跳变，
          不接管手臂并**退出到 Gamepad**。填 0 时若手臂不在零位就会触发这一点。
          本脚本检测到 vla 模式且未加 --echo 时会给出警告（不拦截）。

依赖：pyzmq        （pip3 install --user pyzmq / uv pip install pyzmq）

用法：

  # 只订阅观察（Groot 在跑，6004 由 GripperStateBroadcaster 提供）
  ./groot_client.py watch
  ./groot_client.py watch --arms            # 附带手臂 14 关节 q

  # 张开 / 闭合右爪（q=0 闭合，量程上限约 5.62 为张开）
  ./groot_client.py grip --right 5.0
  ./groot_client.py grip --right 0.0 --left 0.0
  ./groot_client.py grip --right 5.0 --follow    # 下发后继续打印状态
  ./groot_client.py grip --right 5.0 --echo      # 手臂用实测姿态回填（VLA 下推荐）

注意：
  - 必须先进入 Groot 状态（RB + X），否则 6001/6002 未绑定。
  - 夹爪目标与当前控制模式无关：Gamepad/Navigation/VLA 下都会生效。
  - q 超出 config.yaml 的 FSM.Groot.gripper.q_min/q_max 时，本脚本只做提示，
    真正的 clamp 由 GripperBridge 执行（会打一条日志）。
"""

import argparse
import json
import sys
import time

import zmq

# G1 29-DoF 手臂 14 关节的 LeRobot 名字，顺序 = 电机 15..28（左臂 7 个在前）。
# 单一事实来源是 deploy/include/groot/JointNameMap.h 的 kLerobotJointNames[15..28]；
# 6002 解析按名匹配，大小写不敏感。
ARM_LE_ROBOT_NAMES = [
    "kLeftShoulderPitch", "kLeftShoulderRoll", "kLeftShoulderYaw", "kLeftElbow",
    "kLeftWristRoll", "kLeftWristPitch", "kLeftWristYaw",
    "kRightShoulderPitch", "kRightShoulderRoll", "kRightShoulderYaw", "kRightElbow",
    "kRightWristRoll", "kRightWristPitch", "kRightWristYaw",
]

ARM_MOTOR_START = 15          # 手臂在 motor_state / motor_cmd 里的起始下标
NUM_ARM_JOINTS = len(ARM_LE_ROBOT_NAMES)

# GrootModeManager 的 VLA 偏差保护阈值，默认取 config.yaml 的
# FSM.Groot.vla.max_arm_deviation_deg（40）。只用于本脚本的提示文案。
VLA_DEVIATION_GUARD_DEG = 40.0


class Subscriber:
    """SUB 一个 PUB 端口；用 CONFLATE 保证永远只留着最新一帧。"""

    def __init__(self, ctx, host, port):
        self.sock = ctx.socket(zmq.SUB)
        self.sock.setsockopt(zmq.SUBSCRIBE, b"")
        self.sock.setsockopt(zmq.CONFLATE, 1)
        self.sock.setsockopt(zmq.RCVTIMEO, 0)
        self.sock.connect(f"tcp://{host}:{port}")
        self.latest = None
        self.count = 0

    def poll(self):
        """取回最新一帧（没有新帧则返回 False）。"""
        got = False
        while True:
            try:
                payload = self.sock.recv_string(zmq.NOBLOCK)
            except zmq.Again:
                break
            try:
                self.latest = json.loads(payload)
                self.count += 1
                got = True
            except json.JSONDecodeError:
                continue
        return got


def fmt(value, width=8, digits=4):
    if value is None:
        return f"{'--':>{width}}"
    return f"{value:>+{width}.{digits}f}"


def cmd_watch(args):
    ctx = zmq.Context()
    subs = {}
    if not args.no_gripper:
        subs["gripper"] = Subscriber(ctx, args.host, args.gripper_port)
    if not args.no_mode:
        subs["mode"] = Subscriber(ctx, args.host, args.mode_port)
    subs["lowstate"] = Subscriber(ctx, args.host, args.lowstate_port)

    print(f"# groot_client watch: {args.host} gripper={args.gripper_port} "
          f"mode={args.mode_port} lowstate={args.lowstate_port}  (Ctrl-C 退出)")
    if args.arms:
        print("# " + "  ".join(f"{n[1:]:>10}" for n in ARM_LE_ROBOT_NAMES))
    print("# " + f"{'time':>10}  {'mode':>7}  {'r.q':>8} {'r.dq':>8} {'r.tau':>8}  "
          f"{'l.q':>8} {'l.dq':>8} {'l.tau':>8}")

    period = 1.0 / args.hz
    try:
        while True:
            for sub in subs.values():
                sub.poll()

            gripper = subs.get("gripper")
            right = left = {}
            if gripper and gripper.latest:
                data = gripper.latest.get("data", {})
                right = data.get("right", {}) or {}
                left = data.get("left", {}) or {}

            mode = "-"
            if "mode" in subs and subs["mode"].latest:
                mode = subs["mode"].latest.get("state", "-")

            line = (f"  {time.strftime('%H:%M:%S')}  {mode:>7}  "
                    f"{fmt(right.get('q'))} {fmt(right.get('dq'))} {fmt(right.get('tau_est'))}  "
                    f"{fmt(left.get('q'))} {fmt(left.get('dq'))} {fmt(left.get('tau_est'))}")
            print(line)

            if args.arms:
                arm_q = read_arm_q(subs["lowstate"])
                if arm_q is None:
                    print("           手臂：等待 6001 LowState ...")
                else:
                    print("           " + "  ".join(f"{q:>+10.4f}" for q in arm_q))

            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        for sub in subs.values():
            sub.sock.close()
        ctx.term()
    return 0


def read_arm_q(lowstate_sub):
    """从 6001 的 LowState 里取手臂 14 关节实测 q；没有数据返回 None。"""
    if not lowstate_sub or not lowstate_sub.latest:
        return None
    motors = lowstate_sub.latest.get("data", {}).get("motor_state")
    if not motors or len(motors) < ARM_MOTOR_START + NUM_ARM_JOINTS:
        return None
    return [motors[ARM_MOTOR_START + i].get("q") for i in range(NUM_ARM_JOINTS)]


def build_action_frame(arm_q, right=None, left=None, timestamp=None):
    """拼一条 6002 的 LeRobot action 帧；只带指定侧的 gripper 目标。"""
    action = {}
    for name, value in zip(ARM_LE_ROBOT_NAMES, arm_q):
        action[f"{name}.q"] = float(value)
    action["remote.lx"] = 0.0
    action["remote.ly"] = 0.0
    action["remote.rx"] = 0.0
    action["remote.ry"] = 0.0
    gripper = {}
    if right is not None:
        gripper["right"] = {"q": float(right)}
    if left is not None:
        gripper["left"] = {"q": float(left)}
    if gripper:
        action["gripper"] = gripper
    frame = {"cmd": "action", "action": action,
             "timestamp": time.time() if timestamp is None else float(timestamp)}
    return json.dumps(frame, separators=(",", ":"))


def cmd_grip(args):
    if args.right is None and args.left is None:
        print("grip: 至少要给 --right 或 --left 之一", file=sys.stderr)
        return 2

    for side, value in (("right", args.right), ("left", args.left)):
        if value is None:
            continue
        if not (args.q_min <= value <= args.q_max):
            print(f"grip: 警告 {side} q={value:.4f} 超出配置量程 "
                  f"[{args.q_min}, {args.q_max}]；桥接层会 clamp 到边界并打日志",
                  file=sys.stderr)

    ctx = zmq.Context()

    # 先看一眼当前控制模式：VLA 下这条帧的 arm_q 会被 update_vla() 拿去和实测姿态
    # 比较，偏差超阈值就会把机器人踢出 VLA，所以填 0 在这个模式下不是无害的。
    mode = None
    if not args.echo:
        mode_sub = Subscriber(ctx, args.host, args.mode_port)
        deadline = time.time() + 0.6
        while time.time() < deadline:
            mode_sub.poll()
            if mode_sub.latest:
                mode = mode_sub.latest.get("state")
                break
            time.sleep(0.02)
        mode_sub.sock.close()
        if mode == "vla":
            print("grip: 警告 当前是 VLA 模式，这条帧的手臂 14 关节会填 0。"
                  f"若手臂不在零位（偏差 > {VLA_DEVIATION_GUARD_DEG:.0f} deg），"
                  "update_vla() 的偏差保护会判定该目标会让手臂跳变，"
                  "不接管手臂并**退出到 Gamepad**。想避免请加 --echo（用实测姿态回填）。",
                  file=sys.stderr)

    if args.echo:
        lowstate = Subscriber(ctx, args.host, args.lowstate_port)
        print(f"# 等待 6001 LowState（{args.host}:{args.lowstate_port}）以回读手臂实测姿态 ...")
        arm_q = None
        deadline = time.time() + args.wait
        while time.time() < deadline:
            lowstate.poll()
            arm_q = read_arm_q(lowstate)
            if arm_q is not None:
                break
            time.sleep(0.05)
        if arm_q is None:
            print(f"grip: {args.wait:.1f}s 内没等到 LowState，拒绝下发（--echo 需要实测姿态回填）。"
                  f"请确认已进入 Groot（RB + X），或用 --wait 加长等待。", file=sys.stderr)
            lowstate.sock.close()
            ctx.term()
            return 1
        lowstate.sock.close()
        print("# 手臂 echo: " + "  ".join(f"{q:>+7.4f}" for q in arm_q))
    else:
        # 手臂 14 关节填 0 = Groot 的 safe_home（publish_targets 还会过一次 URDF 限幅）。
        arm_q = [0.0] * NUM_ARM_JOINTS
        print("# 手臂 14 关节填 0（safe_home）；远程摇杆轴填 0")

    push = ctx.socket(zmq.PUSH)
    push.setsockopt(zmq.LINGER, 500)          # 让已入队的帧有机会发出去再关闭
    push.connect(f"tcp://{args.host}:{args.cmd_port}")

    sent = 0
    for i in range(args.repeat):
        frame = build_action_frame(arm_q, args.right, args.left, timestamp=time.time() + i * 1e-3)
        push.send_string(frame)
        sent += 1
        if sent < args.repeat:
            time.sleep(0.05)
    print(f"# 已向 {args.host}:{args.cmd_port} 发送 {sent} 条 action 帧"
          f"（gripper: right={args.right} left={args.left}）")

    if args.follow:
        push.close()
        ctx.term()
        return cmd_watch(args)

    time.sleep(0.3)                           # 等 LINGER 把队列发完
    push.close()
    ctx.term()
    return 0


def main():
    # 行缓冲：这样 `| head` / 重定向到日志时也能实时看到状态。
    sys.stdout.reconfigure(line_buffering=True)

    parser = argparse.ArgumentParser(
        description="Groot 夹爪 / 状态命令行客户端",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("用法：", 1)[-1])
    parser.add_argument("--host", default="127.0.0.1", help="控制器所在主机（默认 127.0.0.1）")
    sub = parser.add_subparsers(dest="command", required=True)

    # watch 与 grip --follow 共用的一组「订阅 / 打印」选项。
    display = argparse.ArgumentParser(add_help=False)
    display.add_argument("--gripper-port", type=int, default=6004, help="夹爪状态 PUB 端口")
    display.add_argument("--mode-port", type=int, default=6000, help="控制模式 PUB 端口")
    display.add_argument("--lowstate-port", type=int, default=6001, help="LowState PUB 端口")
    display.add_argument("--hz", type=float, default=5.0, help="打印频率（默认 5 Hz）")
    display.add_argument("--arms", action="store_true", help="额外打印手臂 14 关节 q")
    display.add_argument("--no-gripper", action="store_true", help="不订阅夹爪状态")
    display.add_argument("--no-mode", action="store_true", help="不订阅控制模式")

    watch = sub.add_parser("watch", parents=[display], help="订阅并打印状态")
    watch.set_defaults(func=cmd_watch)

    grip = sub.add_parser("grip", parents=[display],
                          help="向 6002 下发夹爪目标（手臂 14 关节默认填 0）")
    grip.add_argument("--right", type=float, default=None, help="右爪目标 q（rad，0=闭合）")
    grip.add_argument("--left", type=float, default=None, help="左爪目标 q（rad，0=闭合）")
    grip.add_argument("--cmd-port", type=int, default=6002, help="action 帧 PULL 端口")
    grip.add_argument("--repeat", type=int, default=3, help="重发条数（默认 3，抗丢包）")
    grip.add_argument("--echo", action="store_true",
                      help="手臂 14 关节用 6001 的实测姿态回填（默认填 0 = safe_home）；"
                           "VLA 模式下建议开启，避免触发偏差保护被踢出 VLA")
    grip.add_argument("--wait", type=float, default=3.0, help="--echo 时等 LowState 的秒数（默认 3）")
    grip.add_argument("--q-min", type=float, default=0.0, help="提示用的量程下界（默认 0.0）")
    grip.add_argument("--q-max", type=float, default=5.6217, help="提示用的量程上界（默认 5.6217）")
    grip.add_argument("--follow", action="store_true", help="下发后继续 watch")
    grip.set_defaults(func=cmd_grip)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
