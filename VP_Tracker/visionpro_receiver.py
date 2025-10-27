#!/usr/bin/env python3
"""
Vision Pro Tracking Streamer -> LeRobot SO-101 (Feetech) teleop
Head-relative wrist + roll + pinch -> 6 joints, with:
 - Pitch-vs-vertical decoupling (learned slope) + speed gate for wrist_flex
 - Asymmetric shoulder_lift limits & gains (less back, more forward)
 - Reduced backward elbow swing

Requires:
  pip install avp_stream lerobot numpy
"""

import time
import math
import json
from pathlib import Path
import numpy as np

from avp_stream import VisionProStreamer
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

# ----------------- USER CONFIG -----------------
AVP_IP  = "128.179.223.208"                # Vision Pro IP (shown in the app)
PORT    = "/dev/tty.usbmodem58FD0170541"    # Follower USB port
ROBOT_ID = "dabrius"

UPDATE_HZ      = 20
SMOOTHING      = 0.8      # 0=no smoothing, 0.9=very smooth
DEADZONE_DEG   = 2.0

# Conservative safety limits in normalized degrees (-100..100)
# Asymmetric shoulder_lift: much less backward, more forward
SAFE = {
    "shoulder_pan":  (-60,  60),
    "shoulder_lift": (-20,  85),   # <- asymmetric: restrict back, allow more forward
    "elbow_flex":    (-20,  75),   # tighter backward range
    "wrist_flex":    (-60,  60),
    "wrist_roll":    (-75,  75),
    "gripper":       (-100, 100),
}

# Motor IDs/models for SO-101 follower (Feetech STS3215)
MOTORS = {
    "shoulder_pan":  {"id": 1, "model": "sts3215"},
    "shoulder_lift": {"id": 2, "model": "sts3215"},
    "elbow_flex":    {"id": 3, "model": "sts3215"},
    "wrist_flex":    {"id": 4, "model": "sts3215"},
    "wrist_roll":    {"id": 5, "model": "sts3215"},
    "gripper":       {"id": 6, "model": "sts3215"},
}

# Mapping gains (flip signs if needed)
DEG_PER_M_X =  220.0   # wrist lateral  -> shoulder_pan
# Asymmetric gains for shoulder_lift (dy > 0 forward / dy < 0 backward)
GAIN_UP_Y     = 260.0  # dy >= 0 (favor forward)
GAIN_DOWN_Y   = 140.0  # dy < 0  (dampen backward)
SHOULDER_LIFT_BIAS_DEG = +8.0  # small forward bias (optional)

DEG_PER_M_Z = -120.0   # wrist fwd/back -> elbow_flex (reduced to avoid overshoot)
DEG_PER_RAD_PITCH = 180.0 / math.pi

# Invert wrist axes if required
WRIST_FLEX_INVERT = True
WRIST_ROLL_INVERT = True

# Gripper mapping (pinch distance meters)
PINCH_CLOSE_M = 0.03
PINCH_OPEN_M  = 0.09
GRIPPER_INVERT = False

# Wrist decoupling / gating
VERT_SPEED_GATE_MPS = 0.15   # if |dy_rate| > this, freeze wrist_flex update
DECOUPLING_SAMPLES  = 60     # ~3 s @ 20 Hz to learn dy->pitch leakage

# Shoulder_lift asymmetric dead-zone (helps avoid small backward drift)
ASYM_LIFT_DEADZONE = True
LIFT_BACK_DZ_SCALE = 1.5     # dead-zone multiplier when command < 0 (backward)

# ----------------- HELPERS -----------------
def clamp(x, lo, hi): return max(lo, min(hi, x))

def deadzone(curr, target, dz): return curr if abs(target - curr) < dz else target

def smooth(curr, target, a): return a*curr + (1-a)*target

def rot_to_pitch_deg(R: np.ndarray) -> float:
    """Intrinsic ZYX; pitch = asin(-R[2,0])."""
    v = -R[2,0]
    v = clamp(v, -1.0, 1.0)
    return math.degrees(math.asin(v))

def rad_or_deg_to_deg(x: float) -> float:
    return math.degrees(x) if abs(x) < 6.3 else x

def load_calibration(robot_id: str):
    f = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower" / f"{robot_id}.json"
    if f.exists():
        with open(f) as fd:
            data = json.load(fd)
        return {k: type("Cal", (), v) for k, v in data.items()}
    return None

def map_pinch_to_gripper(pinch_m: float) -> float:
    pinch_m = clamp(pinch_m, 0.0, 0.2)
    if PINCH_OPEN_M == PINCH_CLOSE_M:
        g = 0.0
    else:
        g = 200.0 * (pinch_m - PINCH_CLOSE_M) / (PINCH_OPEN_M - PINCH_CLOSE_M) - 100.0
    if GRIPPER_INVERT:
        g = -g
    return clamp(g, -100.0, 100.0)

def wrist_in_head(r):
    H = r["head"][0]         # 4x4 head in world
    W = r["right_wrist"][0]  # 4x4 wrist in world
    T_rel = np.linalg.inv(H) @ W
    t = T_rel[:3, 3]
    R = T_rel[:3, :3]
    return t, R

# ----------------- MAIN -----------------
def main():
    print(">>> Connecting Vision Pro stream...")
    stream = VisionProStreamer(ip=AVP_IP, record=False)
    time.sleep(0.2)

    print(">>> Connecting SO-101 motors...")
    motors = {name: Motor(cfg["id"], cfg["model"], MotorNormMode.RANGE_M100_100)
              for name, cfg in MOTORS.items()}
    calibration = load_calibration(ROBOT_ID)
    bus = FeetechMotorsBus(port=PORT, motors=motors, calibration=calibration)
    bus.connect()
    print("✓ Connected.")

    pos = {name: 0.0 for name in MOTORS.keys()}

    # -------- Home capture (1s) in head frame --------
    print(">>> Hold a NEUTRAL pose (arm forward, hand relaxed) for ~1 s...")
    xs, ys, zs, Rs = [], [], [], []
    t0 = time.time()
    while time.time() - t0 < 1.0:
        r = stream.latest
        if r and isinstance(r.get("right_wrist"), np.ndarray) and isinstance(r.get("head"), np.ndarray):
            t, R = wrist_in_head(r)
            xs.append(t[0]); ys.append(t[1]); zs.append(t[2]); Rs.append(R)
        time.sleep(0.01)
    home = {
        "x": float(np.median(xs) if xs else 0.0),
        "y": float(np.median(ys) if ys else 0.0),
        "z": float(np.median(zs) if zs else 0.0),
        "R": np.median(np.stack(Rs), axis=0) if Rs else np.eye(3),
    }
    print("✓ Home captured.")

    # -------- Decoupling calibration (up/down sweep) --------
    print(">>> CALIBRATION: move hand UP/DOWN (no wrist bend) for ~3 s...")
    dy_list, pitch_list = [], []
    samples = 0
    while samples < DECOUPLING_SAMPLES:
        r = stream.latest
        if r and isinstance(r.get("right_wrist"), np.ndarray) and isinstance(r.get("head"), np.ndarray):
            t, R = wrist_in_head(r)
            dy = t[1] - home["y"]
            pitch = rot_to_pitch_deg(R)
            dy_list.append(dy); pitch_list.append(pitch)
            samples += 1
        time.sleep(1.0/UPDATE_HZ)
    dy_arr = np.array(dy_list); pitch_arr = np.array(pitch_list)
    var_dy = float(np.var(dy_arr)) if len(dy_arr) else 0.0
    if var_dy > 1e-6:
        COUPLING_Y2PITCH = float(np.cov(dy_arr, pitch_arr, ddof=0)[0,1] / var_dy)
    else:
        COUPLING_Y2PITCH = 0.0
    print(f"✓ Decoupling learned: pitch -= {COUPLING_Y2PITCH:.2f} * dy")

    dt = 1.0 / UPDATE_HZ
    last = time.time()
    prev_dy = 0.0

    print("\nControls:")
    print("  • Hand left/right     -> shoulder_pan")
    print("  • Hand up/down        -> shoulder_lift (head-relative, asymmetric)")
    print("  • Hand forward/back   -> elbow_flex (reduced backward)")
    print("  • Wrist pitch         -> wrist_flex (decoupled from vertical + speed-gated)")
    print("  • Wrist roll          -> wrist_roll")
    print("  • Pinch thumb-index   -> gripper open/close")
    print("  • Ctrl+C              -> neutral + exit\n")

    try:
        while True:
            now = time.time()
            if now - last < dt:
                time.sleep(0.001)
                continue
            last = now

            r = stream.latest
            if not r or not isinstance(r.get("right_wrist"), np.ndarray) or not isinstance(r.get("head"), np.ndarray):
                continue

            t, R = wrist_in_head(r)
            dx = float(t[0] - home["x"])
            dy = float(t[1] - home["y"])
            dz = float(t[2] - home["z"])

            # Vertical speed gate
            dy_rate = (dy - prev_dy) / dt
            prev_dy = dy

            target = {}
            # Shoulder pan
            target["shoulder_pan"] = dx * DEG_PER_M_X

            # Shoulder lift with asymmetric gain + bias
            gain_y = GAIN_UP_Y if dy >= 0.0 else GAIN_DOWN_Y
            target["shoulder_lift"] = dy * gain_y + SHOULDER_LIFT_BIAS_DEG

            # Elbow flex (reduced gain; SAFE already asymmetric)
            target["elbow_flex"] = dz * DEG_PER_M_Z

            # Wrist flex from pitch, decoupled from dy; speed-gated
            flex_deg = rot_to_pitch_deg(R) - COUPLING_Y2PITCH * dy
            if WRIST_FLEX_INVERT:
                flex_deg = -flex_deg
            if abs(dy_rate) > VERT_SPEED_GATE_MPS:
                flex_deg = pos["wrist_flex"]  # hold while moving vertically fast
            target["wrist_flex"] = flex_deg

            # Wrist roll
            roll_deg = rad_or_deg_to_deg(float(r.get("right_wrist_roll", 0.0)))
            if WRIST_ROLL_INVERT:
                roll_deg = -roll_deg
            target["wrist_roll"] = roll_deg

            # Gripper
            pinch = r.get("right_pinch_distance", None)
            if pinch is not None:
                try:
                    target["gripper"] = map_pinch_to_gripper(float(pinch))
                except (TypeError, ValueError):
                    target["gripper"] = pos["gripper"]
            else:
                target["gripper"] = pos["gripper"]

            # Apply safety, deadzone (asymmetric for shoulder_lift), smoothing and send
            for name, val in target.items():
                lo, hi = SAFE[name]
                val = clamp(val, lo, hi)

                if ASYM_LIFT_DEADZONE and name == "shoulder_lift":
                    dz_local = DEADZONE_DEG * (LIFT_BACK_DZ_SCALE if val < 0 else 1.0)
                else:
                    dz_local = DEADZONE_DEG

                val = deadzone(pos[name], val, dz_local)
                pos[name] = smooth(pos[name], val, SMOOTHING)

                try:
                    bus.write("Goal_Position", name, pos[name])
                except RuntimeError as e:
                    print(f"[WARN] {name}: {e}")

    except KeyboardInterrupt:
        print("\nNeutral & exit...")
    finally:
        neutral = {k: 0.0 for k in pos}
        for _ in range(40):
            for name in pos:
                pos[name] = smooth(pos[name], neutral[name], 0.6)
                try:
                    bus.write("Goal_Position", name, pos[name])
                except RuntimeError:
                    pass
            time.sleep(0.03)
        bus.disconnect()
        print("Bye.")

if __name__ == "__main__":
    main()