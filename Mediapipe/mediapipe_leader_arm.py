#!/usr/bin/env python3
"""
Leader arm via MediaPipe -> 6-DoF targets for LeRobot S0101, with a live PREVIEW window.

What you get:
  - Webcam window with skeleton overlay
  - Separate "S0101 Preview" window showing per-motor mapping and values
  - Motor 6 (Gripper) shows OPEN/CLOSE based on --grip_threshold (default 50%)

Outputs (degrees unless noted):
  base_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper(0..100%)

Backends:
  - print : print the JSON (dry-run)
  - udp   : send JSON datagrams to host:port

Keys:
  c = capture current offsets (zero here)
  z = clear offsets
  space = toggle sending on/off
  q = quit
"""

import argparse
import json
import math
import socket
import time
from dataclasses import dataclass
from typing import Dict, Tuple, Optional, List

import cv2
import numpy as np
import mediapipe as mp


# --------------------------- Utility math ---------------------------

def _norm(v):
    n = np.linalg.norm(v)
    if n < 1e-8:
        return v
    return v / n

def angle_between(v1, v2) -> float:
    """Unsigned angle (deg) between two vectors."""
    n1, n2 = _norm(v1), _norm(v2)
    dot = float(np.clip(np.dot(n1, n2), -1.0, 1.0))
    return math.degrees(math.acos(dot))

def signed_angle_around_axis(u, v, axis) -> float:
    """Signed angle (deg) from u to v around 'axis' (right-hand rule)."""
    a = _norm(axis)
    u_perp = u - np.dot(u, a) * a
    v_perp = v - np.dot(v, a) * a
    u_perp = _norm(u_perp)
    v_perp = _norm(v_perp)
    unsigned = angle_between(u_perp, v_perp)
    s = np.dot(a, np.cross(u_perp, v_perp))
    return unsigned if s >= 0 else -unsigned

def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# --------------------------- Filters & config ---------------------------

class EMA:
    def __init__(self, alpha=0.2, init=None):
        self.alpha = float(alpha)
        self.x = init
    def update(self, v):
        if self.x is None:
            self.x = float(v)
        else:
            self.x = self.alpha * float(v) + (1 - self.alpha) * self.x
        return self.x

@dataclass
class JointLimits:
    lo: float
    hi: float
    invert: bool = False

@dataclass
class Offsets:
    base: float = 0.0
    shoulder: float = 0.0
    elbow: float = 0.0
    wrist_flex: float = 0.0
    wrist_roll: float = 0.0
    gripper: float = 0.0  # percentage offset rarely used


DEFAULT_LIMITS = {
    "base":       JointLimits(lo=-90, hi= 90, invert=False),
    "shoulder":   JointLimits(lo=  0, hi=130, invert=True),
    "elbow":      JointLimits(lo=  0, hi=150, invert=False),
    "wrist_flex": JointLimits(lo=-90, hi= 90, invert=True),
    "wrist_roll": JointLimits(lo=-90, hi= 90, invert=False),
    "gripper":    JointLimits(lo=  0, hi=100, invert=False),  # 0..100 (%)
}

# Motor mapping you provided (IDs and names)
MOTOR_MAP: List[Tuple[int, str, str]] = [
    (1, "Base / Shoulder Pan", "base"),
    (2, "Shoulder Lift",       "shoulder"),
    (3, "Elbow Flex",          "elbow"),
    (4, "Wrist Flex",          "wrist_flex"),
    (5, "Wrist Roll",          "wrist_roll"),
    (6, "Gripper",             "gripper"),
]


# --------------------------- Backends ---------------------------

class BackendBase:
    def send(self, payload: Dict): ...
    def close(self): ...

class PrintBackend(BackendBase):
    def send(self, payload: Dict):
        print(json.dumps(payload, separators=(',', ':')))

class UDPBackend(BackendBase):
    def __init__(self, host: str, port: int):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    def send(self, payload: Dict):
        self.sock.sendto(json.dumps(payload).encode('utf-8'), self.addr)
    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


# --------------------------- Angle extraction ---------------------------

POSE = mp.solutions.pose.Pose
L = mp.solutions.pose.PoseLandmark

RIGHT = {
    "SHOULDER": L.RIGHT_SHOULDER,
    "ELBOW":    L.RIGHT_ELBOW,
    "WRIST":    L.RIGHT_WRIST,
    "HIP":      L.RIGHT_HIP,
    "INDEX":    L.RIGHT_INDEX,
    "PINKY":    L.RIGHT_PINKY,
    "THUMB":    L.RIGHT_THUMB,
}

LEFT = {
    "SHOULDER": L.LEFT_SHOULDER,
    "ELBOW":    L.LEFT_ELBOW,
    "WRIST":    L.LEFT_WRIST,
    "HIP":      L.LEFT_HIP,
    "INDEX":    L.LEFT_INDEX,
    "PINKY":    L.LEFT_PINKY,
    "THUMB":    L.LEFT_THUMB,
}

def vec_of(wlm, lid) -> np.ndarray:
    lm = wlm.landmark[lid]
    return np.array([lm.x, lm.y, lm.z], dtype=np.float32)

def compute_arm_angles(world_lms, side: str) -> Optional[Dict[str, float]]:
    """
    Returns joint angles (deg) + gripper% from MediaPipe 'pose_world_landmarks'.
    World coords: x right, y up, z forward (negative toward camera).
    """
    ids = RIGHT if side == "right" else LEFT

    S = vec_of(world_lms, ids["SHOULDER"])
    E = vec_of(world_lms, ids["ELBOW"])
    W = vec_of(world_lms, ids["WRIST"])
    I = vec_of(world_lms, ids["INDEX"])
    P = vec_of(world_lms, ids["PINKY"])
    T = vec_of(world_lms, ids["THUMB"])

    u = E - S           # upper arm
    f = W - E           # forearm
    hand_x = I - P      # across-palm (index->pinky)
    palm_n = np.cross(hand_x, (I + P)/2 - W)  # rough palm normal
    a = _norm(f)        # forearm axis

    # Base pan (yaw): azimuth of upper arm in the horizontal (x-z) plane.
    yaw = math.degrees(math.atan2(u[0], -u[2]))  # z forward is negative

    # Shoulder lift (pitch): elevation of the upper arm.
    pitch = math.degrees(math.atan2(u[1], math.hypot(u[0], u[2])))

    # Elbow flex: angle between -u and f (0=straight).
    elbow = angle_between(-u, f)

    # Wrist flex (pitch): angle between forearm and finger direction.
    finger_dir = _norm(((I - W) + (P - W)) * 0.5)
    wrist_flex = signed_angle_around_axis(a, finger_dir, axis=np.cross(a, [0,1,0]) + 1e-8)

    # Wrist roll (pronation/supination): rotation of palm normal around forearm axis.
    ref = np.array([0, -1, 0], dtype=np.float32)  # "down"
    wrist_roll = signed_angle_around_axis(ref, _norm(palm_n), axis=a)

    # Gripper: pinch (thumb–index) distance -> 0..100%
    pinch = np.linalg.norm(T - I)
    flen = np.linalg.norm(f) + 1e-6
    pinch_norm = clamp(1.0 - (pinch / flen), 0.0, 1.0)
    grip_pct = 100.0 * pinch_norm

    return {
        "base": yaw,
        "shoulder": pitch,
        "elbow": elbow,
        "wrist_flex": wrist_flex,
        "wrist_roll": wrist_roll,
        "gripper": grip_pct,
    }


# --------------------------- Preview rendering ---------------------------

def _norm_for_bar(key: str, value: float) -> float:
    """Map joint value to 0..1 for bar rendering."""
    lim = DEFAULT_LIMITS[key]
    if lim.hi == lim.lo:
        return 0.5
    return clamp((value - lim.lo) / (lim.hi - lim.lo), 0.0, 1.0)

def draw_preview(adj: Dict[str, float],
                 active_keys: List[str],
                 grip_threshold: float = 50.0,
                 side: str = "right"):
    """
    Render a compact dashboard with one row per motor:
      M#  Name                value + bar   (OPEN/CLOSE for gripper)
    Active (moving) rows are highlighted.
    """
    H, W = 320, 560
    img = np.full((H, W, 3), 22, dtype=np.uint8)  # dark background

    # Header
    title = f"S0101 Preview  |  Side: {side.capitalize()}"
    cv2.putText(img, title, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (230, 230, 230), 2, cv2.LINE_AA)

    # Column positions
    y = 60
    line_h = 38
    name_x = 14
    bar_x = 260
    bar_w = 270
    bar_h = 16

    for motor_id, name, key in MOTOR_MAP:
        is_active = key in active_keys

        # Row background (highlight if active)
        if is_active:
            cv2.rectangle(img, (8, y - 22), (W - 8, y + 16), (60, 90, 200), thickness=-1)

        # Motor ID + name
        label = f"M{motor_id}  {name}"
        cv2.putText(img, label, (name_x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1, cv2.LINE_AA)

        # Value + bar
        v = adj.get(key, 0.0)

        if key != "gripper":
            val_text = f"{v:+6.1f}°"
            cv2.putText(img, val_text, (bar_x - 110, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (220, 220, 220), 1, cv2.LINE_AA)
            p = _norm_for_bar(key, v)
            x0, y0 = bar_x, y - bar_h + 4
            cv2.rectangle(img, (x0, y0), (x0 + bar_w, y0 + bar_h), (80, 80, 80), 1)
            cv2.rectangle(img, (x0, y0), (x0 + int(bar_w * p), y0 + bar_h), (120, 200, 120), -1)
            # min/zero/max ticks
            cv2.line(img, (x0, y0 + bar_h + 2), (x0, y0 + bar_h + 6), (150,150,150), 1)
            cv2.line(img, (x0 + bar_w//2, y0 + bar_h + 2), (x0 + bar_w//2, y0 + bar_h + 6), (150,150,150), 1)
            cv2.line(img, (x0 + bar_w, y0 + bar_h + 2), (x0 + bar_w, y0 + bar_h + 6), (150,150,150), 1)
        else:
            # Gripper: show OPEN/CLOSE + percent + bar
            state = "CLOSE" if v >= grip_threshold else "OPEN"
            val_text = f"{state} ({v:4.0f}%)"
            cv2.putText(img, val_text, (bar_x - 140, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (220, 220, 220), 1, cv2.LINE_AA)
            p = clamp(v / 100.0, 0.0, 1.0)
            x0, y0 = bar_x, y - bar_h + 4
            cv2.rectangle(img, (x0, y0), (x0 + bar_w, y0 + bar_h), (80, 80, 80), 1)
            cv2.rectangle(img, (x0, y0), (x0 + int(bar_w * p), y0 + bar_h), (200, 150, 120), -1)
            # threshold tick
            tx = x0 + int(bar_w * clamp(grip_threshold / 100.0, 0.0, 1.0))
            cv2.line(img, (tx, y0 - 2), (tx, y0 + bar_h + 2), (180, 180, 180), 1)

        y += line_h

    cv2.imshow("S0101 Preview", img)


# --------------------------- Main loop ---------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", choices=["right", "left"], default="right",
                    help="Which arm to track from the camera view.")
    ap.add_argument("--backend", choices=["print", "udp"], default="print")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=7777)
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--alpha", type=float, default=0.3, help="EMA smoothing factor (0..1).")
    ap.add_argument("--max_fps", type=float, default=30.0)
    ap.add_argument("--active_eps_deg", type=float, default=1.5,
                    help="Change threshold (deg) to highlight a joint as ACTIVE.")
    ap.add_argument("--active_eps_pct", type=float, default=5.0,
                    help="Change threshold (percent) for the gripper to be ACTIVE.")
    ap.add_argument("--grip_threshold", type=float, default=50.0,
                    help="Gripper CLOSE threshold in percent (>= means CLOSE).")
    args = ap.parse_args()

    # Backend
    if args.backend == "udp":
        backend = UDPBackend(args.host, args.port)
    else:
        backend = PrintBackend()

    # Filters per joint
    filters = {k: EMA(alpha=args.alpha) for k in ["base", "shoulder", "elbow", "wrist_flex", "wrist_roll", "gripper"]}
    offsets = Offsets()
    sending = True
    last_adj: Optional[Dict[str, float]] = None

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FPS, args.max_fps)

    mp_drawing = mp.solutions.drawing_utils
    mp_pose = mp.solutions.pose

    last_send_t = 0.0
    try:
        with mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        ) as pose:

            while True:
                ok, frame = cap.read()
                if not ok:
                    print("No frame from camera.")
                    break

                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                res = pose.process(rgb)

                hud_lines = []
                adj = None

                if res.pose_world_landmarks and res.pose_landmarks:
                    mp_drawing.draw_landmarks(
                        frame, res.pose_landmarks, mp_pose.POSE_CONNECTIONS
                    )

                    raw = compute_arm_angles(res.pose_world_landmarks, side=args.side)
                    if raw is not None:
                        # Apply offsets/limits/filtering
                        adj = {}
                        for k, v in raw.items():
                            if k != "gripper":
                                v = v - getattr(offsets, k)
                            lim = DEFAULT_LIMITS[k]
                            if lim.invert:
                                v = -v
                            v = clamp(v, lim.lo, lim.hi)
                            v = filters[k].update(v)
                            adj[k] = v

                        payload = {
                            "t": time.time(),
                            "side": args.side,
                            "units": {"joints": "deg", "gripper": "percent"},
                            "joints_deg": {
                                "base_pan": float(adj["base"]),
                                "shoulder_lift": float(adj["shoulder"]),
                                "elbow_flex": float(adj["elbow"]),
                                "wrist_flex": float(adj["wrist_flex"]),
                                "wrist_roll": float(adj["wrist_roll"]),
                            },
                            "gripper_pct": float(adj["gripper"]),
                        }

                        now = time.time()
                        if sending and (now - last_send_t) >= (1.0 / max(args.max_fps, 1.0)):
                            backend.send(payload)
                            last_send_t = now

                        # HUD
                        hud_lines.append(f"SEND: {'ON' if sending else 'OFF'}")
                        hud_lines.append(f"Base  (pan): {adj['base']:6.1f}°")
                        hud_lines.append(f"Should(lift): {adj['shoulder']:6.1f}°")
                        hud_lines.append(f"Elbow (flex): {adj['elbow']:6.1f}°")
                        hud_lines.append(f"Wrist (flex): {adj['wrist_flex']:6.1f}°")
                        hud_lines.append(f"Wrist (roll): {adj['wrist_roll']:6.1f}°")
                        state = "CLOSE" if adj['gripper'] >= args.grip_threshold else "OPEN"
                        hud_lines.append(f"Gripper   (%): {adj['gripper']:6.1f}  [{state}]")
                    else:
                        hud_lines.append("Angles: not available")
                else:
                    hud_lines.append("Pose not detected")

                # Draw HUD on camera frame
                y = 20
                for line in hud_lines:
                    cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)
                    y += 20

                cv2.putText(frame, "Keys: [c]=capture zero  [z]=reset offsets  [space]=send on/off  [q]=quit",
                            (10, frame.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)

                cv2.imshow("Leader Arm (MediaPipe)", frame)

                # --- Preview window (uses last known adjusted values) ---
                if adj is None and last_adj is not None:
                    adj_for_preview = last_adj
                elif adj is not None:
                    adj_for_preview = adj
                else:
                    # Nothing yet; render zeros
                    adj_for_preview = {k: 0.0 for k in ["base","shoulder","elbow","wrist_flex","wrist_roll","gripper"]}

                # Determine "active" (recently changed) joints
                active_keys = []
                if last_adj is not None and adj_for_preview is not None:
                    for k in ["base","shoulder","elbow","wrist_flex","wrist_roll"]:
                        if abs(adj_for_preview[k] - last_adj[k]) >= args.active_eps_deg:
                            active_keys.append(k)
                    if abs(adj_for_preview["gripper"] - last_adj["gripper"]) >= args.active_eps_pct:
                        active_keys.append("gripper")

                draw_preview(adj_for_preview, active_keys, grip_threshold=args.grip_threshold, side=args.side)
                last_adj = adj_for_preview

                # Keys
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord(' '):
                    sending = not sending
                elif key == ord('z'):
                    offsets = Offsets()
                    for f in filters.values():
                        f.x = None
                    print("[reset] offsets cleared.")
                elif key == ord('c'):
                    if res.pose_world_landmarks:
                        raw_now = compute_arm_angles(res.pose_world_landmarks, side=args.side)
                        if raw_now:
                            offsets = Offsets(
                                base=raw_now["base"],
                                shoulder=raw_now["shoulder"],
                                elbow=raw_now["elbow"],
                                wrist_flex=raw_now["wrist_flex"],
                                wrist_roll=raw_now["wrist_roll"],
                                gripper=0.0,
                            )
                            for f in filters.values():
                                f.x = None
                            print("[calibrate] current pose captured as zero-offset.")
                        else:
                            print("[calibrate] pose not stable.")
    finally:
        backend.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
