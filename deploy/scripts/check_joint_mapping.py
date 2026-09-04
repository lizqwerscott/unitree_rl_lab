#!/usr/bin/env python3
"""
Cross-check the G1 29-DoF joint-name correspondence used by the deploy's VLA path.

The three orderings below must all align to the SAME physical motor index 0..28:
  1. pi0.5 action_feature_names   (arms only: 14 joints + 4 remote axes)
  2. g1_joint_sdk_names           (unitree SDK / rt motor_state & motor_cmd index)
  3. lerobot 29 "k.../.q" names   (G1_29_JointIndex)

Deploy VLA contract:
  - arm slot i (0..13)  ->  motor index 15+i  ->  lowcmd->msg_.motor_cmd()[15+i]
  - controller policy only drives legs+waist (joint_ids 0..14); arms 15..28 are
    driven externally by these 6002 packets.

Run:  python3 deploy/scripts/check_joint_mapping.py
"""

from pathlib import Path

# --- 1. pi0.5 policy output feature order (verbatim) -------------------------
ACTION_FEATURE_NAMES = [
    "kLeftShoulderPitch.q",
    "kLeftShoulderRoll.q",
    "kLeftShoulderYaw.q",
    "kLeftElbow.q",
    "kLeftWristRoll.q",
    "kLeftWristPitch.q",
    "kLeftWristyaw.q",
    "kRightShoulderPitch.q",
    "kRightShoulderRoll.q",
    "kRightShoulderYaw.q",
    "kRightElbow.q",
    "kRightWristRoll.q",
    "kRightWristPitch.q",
    "kRightWristYaw.q",
    "remote.lx",
    "remote.ly",
    "remote.rx",
    "remote.ry",
]

# --- 2. unitree SDK joint names in motor index order (verbatim) --------------
G1_SDK_JOINT_NAMES = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

# --- 3. lerobot 29 "k.../.q" names in motor index order (verbatim) -----------
LEROBO_29_NAMES = [
    "kLeftHipPitch.q", "kLeftHipRoll.q", "kLeftHipYaw.q", "kLeftKnee.q",
    "kLeftAnklePitch.q", "kLeftAnkleRoll.q",
    "kRightHipPitch.q", "kRightHipRoll.q", "kRightHipYaw.q", "kRightKnee.q",
    "kRightAnklePitch.q", "kRightAnkleRoll.q",
    "kWaistYaw.q", "kWaistRoll.q", "kWaistPitch.q",
    "kLeftShoulderPitch.q", "kLeftShoulderRoll.q", "kLeftShoulderYaw.q",
    "kLeftElbow.q", "kLeftWristRoll.q", "kLeftWristPitch.q", "kLeftWristyaw.q",
    "kRightShoulderPitch.q", "kRightShoulderRoll.q", "kRightShoulderYaw.q",
    "kRightElbow.q", "kRightWristRoll.q", "kRightWristPitch.q", "kRightWristYaw.q",
]

ARM_START = 15
ARM_COUNT = 14

# k-name body -> sdk joint body (semantic translation, case-insensitive).
_BODY = {
    "HipPitch": "hip_pitch", "HipRoll": "hip_roll", "HipYaw": "hip_yaw",
    "Knee": "knee", "AnklePitch": "ankle_pitch", "AnkleRoll": "ankle_roll",
    "ShoulderPitch": "shoulder_pitch", "ShoulderRoll": "shoulder_roll",
    "ShoulderYaw": "shoulder_yaw", "Elbow": "elbow",
    "WristRoll": "wrist_roll", "WristPitch": "wrist_pitch", "WristYaw": "wrist_yaw",
}
_WAIST = {"Yaw": "yaw", "Roll": "roll", "Pitch": "pitch"}


def kname_to_sdk(name: str) -> str:
    """Translate a lerobot k-name (optionally with .q) to the sdk joint name."""
    base = name[:-2] if name.endswith(".q") else name
    assert base.startswith("k"), base
    body = base[1:]
    if body.startswith("Waist"):
        token = body[len("Waist"):]
        for k, v in _WAIST.items():
            if token.lower() == k.lower():
                return f"waist_{v}_joint"
        raise AssertionError(f"cannot translate waist {base}")
    for side in ("Left", "Right"):
        if body.startswith(side):
            rest = body[len(side):]
            for token, sdk in _BODY.items():
                if rest.lower() == token.lower():
                    return f"{side.lower()}_{sdk}_joint"
    raise AssertionError(f"cannot translate {base}")


def main() -> None:
    assert len(ACTION_FEATURE_NAMES) == 18, len(ACTION_FEATURE_NAMES)
    assert len(G1_SDK_JOINT_NAMES) == 29, len(G1_SDK_JOINT_NAMES)
    assert len(LEROBO_29_NAMES) == 29, len(LEROBO_29_NAMES)
    arm_features = ACTION_FEATURE_NAMES[:14]
    remote_features = ACTION_FEATURE_NAMES[14:]
    assert remote_features == ["remote.lx", "remote.ly", "remote.rx", "remote.ry"]

    # (a) semantic: every lerobot 29 name must translate to the sdk name at same index
    for i, (lname, sname) in enumerate(zip(LEROBO_29_NAMES, G1_SDK_JOINT_NAMES)):
        assert kname_to_sdk(lname) == sname, f"index {i}: {lname} -> {kname_to_sdk(lname)} != {sname}"

    # (b) arm order: pi0.5 arm features vs lerobot arm names (15..28), case-insensitive
    arm_sdk = G1_SDK_JOINT_NAMES[ARM_START:ARM_START + ARM_COUNT]
    arm_lerobot = LEROBO_29_NAMES[ARM_START:ARM_START + ARM_COUNT]
    for i, (f, l) in enumerate(zip(arm_features, arm_lerobot)):
        assert f[:-2].lower() == l[:-2].lower(), f"slot {i}: {f} vs {l}"

    # (c) deploy config: joint_ids_map identity and policy driving only 0..14
    root = Path(__file__).resolve().parents[1]
    cfg = root / "robots" / "g1_29dof" / "config" / "policy" / "groot" / "params" / "deploy.yaml"
    import yaml
    d = yaml.safe_load(cfg.read_text())
    jmap = d["joint_ids_map"]
    assert jmap == list(range(29)), f"joint_ids_map is not identity: {jmap}"
    joint_ids = d["actions"]["JointPositionAction"]["joint_ids"]
    assert joint_ids == list(range(15)), f"policy must drive only 0..14, got {joint_ids}"

    print(f"{'motor':>5}  {'pi0.5 feature':<22} {'lerobot 29':<22} {'sdk joint / dispatch':<28}")
    print("-" * 88)
    for i in range(29):
        if ARM_START <= i < ARM_START + ARM_COUNT:
            j = i - ARM_START
            feat = arm_features[j]
            print(f"{i:>5}  {feat:<22} {LEROBO_29_NAMES[i]:<22} {G1_SDK_JOINT_NAMES[i]:<28} (arm slot {j}, motor_cmd()[{i}])")
        else:
            print(f"{i:>5}  {'-':<22} {LEROBO_29_NAMES[i]:<22} {G1_SDK_JOINT_NAMES[i]:<28}")
    print("-" * 88)
    print("OK: 3 lists + deploy.yaml all consistent. arm slot i -> motor 15+i is correct.")
    print("NOTE: index 21 left-wrist-yaw differs only in casing across sources "
          "(kLeftWristYaw vs kLeftWristyaw); parser matches case-insensitively.")


if __name__ == "__main__":
    main()
