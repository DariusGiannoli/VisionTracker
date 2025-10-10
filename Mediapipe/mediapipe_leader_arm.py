#!/usr/bin/env python3
"""
Leader arm via MediaPipe -> 6-DoF targets for LeRobot S0101, with live PREVIEW.

Robust fist detection (CLOSE) using MediaPipe Hands 3D:
- OPEN  if (fingers_extended >= open_min_fingers) OR (avg_tip_palm_ratio >= open_ratio_thresh)
- CLOSE if (fingers_extended <= 1) AND (avg_tip_palm_ratio <= fist_ratio_thresh)
- else keep previous state (hysteresis)

Keys:
  c = capture current pose as zero-offset for angles
  z = clear offsets
  space = toggle sending on/off
  q = quit
"""

import argparse, json, math, socket, time
from dataclasses import dataclass
from typing import Dict, Tuple, Optional, List
import cv2, numpy as np, mediapipe as mp

# ---------- small math helpers ----------
def _norm(v):
    n = np.linalg.norm(v);  return v if n < 1e-8 else v / n

def angle_between(v1, v2) -> float:
    n1, n2 = _norm(v1), _norm(v2)
    dot = float(np.clip(np.dot(n1, n2), -1.0, 1.0))
    return math.degrees(math.acos(dot))

def signed_angle_around_axis(u, v, axis) -> float:
    a = _norm(axis)
    u_perp = u - np.dot(u, a) * a
    v_perp = v - np.dot(v, a) * a
    u_perp, v_perp = _norm(u_perp), _norm(v_perp)
    unsigned = angle_between(u_perp, v_perp)
    s = np.dot(a, np.cross(u_perp, v_perp))
    return unsigned if s >= 0 else -unsigned

def clamp(x, lo, hi): return max(lo, min(hi, x))

def angle_at_3d(a, b, c) -> float:
    ba, bc = a - b, c - b
    nba, nbc = _norm(ba), _norm(bc)
    cosang = float(np.clip(np.dot(nba, nbc), -1.0, 1.0))
    return math.degrees(math.acos(cosang))

# ---------- filters & config ----------
class EMA:
    def __init__(self, alpha=0.2, init=None): self.alpha, self.x = float(alpha), init
    def update(self, v):
        self.x = float(v) if self.x is None else self.alpha * float(v) + (1 - self.alpha) * self.x
        return self.x

@dataclass
class JointLimits: lo: float; hi: float; invert: bool=False
@dataclass
class Offsets:
    base: float=0.0; shoulder: float=0.0; elbow: float=0.0
    wrist_flex: float=0.0; wrist_roll: float=0.0; gripper: float=0.0

DEFAULT_LIMITS = {
    "base":       JointLimits(-90,  90, False),
    "shoulder":   JointLimits(  0, 130, True ),
    "elbow":      JointLimits(  0, 150, False),
    "wrist_flex": JointLimits(-90,  90, True ),
    "wrist_roll": JointLimits(-90,  90, False),
    "gripper":    JointLimits(  0, 100, False),
}

MOTOR_MAP: List[Tuple[int, str, str]] = [
    (1, "Base / Shoulder Pan", "base"),
    (2, "Shoulder Lift",       "shoulder"),
    (3, "Elbow Flex",          "elbow"),
    (4, "Wrist Flex",          "wrist_flex"),
    (5, "Wrist Roll",          "wrist_roll"),
    (6, "Gripper",             "gripper"),
]

# ---------- IO backends ----------
class BackendBase:  # pragma: no cover
    def send(self, payload: Dict): ...
    def close(self): ...
class PrintBackend(BackendBase):
    def send(self, payload: Dict): print(json.dumps(payload, separators=(',', ':')))
class UDPBackend(BackendBase):
    def __init__(self, host: str, port: int):
        self.addr = (host, port); self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    def send(self, payload: Dict): self.sock.sendto(json.dumps(payload).encode('utf-8'), self.addr)
    def close(self): 
        try: self.sock.close()
        except Exception: pass

# ---------- MediaPipe setup ----------
POSE, HANDS = mp.solutions.pose.Pose, mp.solutions.hands.Hands
PL = mp.solutions.pose.PoseLandmark
RIGHT = {"SHOULDER":PL.RIGHT_SHOULDER,"ELBOW":PL.RIGHT_ELBOW,"WRIST":PL.RIGHT_WRIST,"INDEX":PL.RIGHT_INDEX,"PINKY":PL.RIGHT_PINKY,"THUMB":PL.RIGHT_THUMB}
LEFT  = {"SHOULDER":PL.LEFT_SHOULDER, "ELBOW":PL.LEFT_ELBOW, "WRIST":PL.LEFT_WRIST, "INDEX":PL.LEFT_INDEX, "PINKY":PL.LEFT_PINKY, "THUMB":PL.LEFT_THUMB}

# ---------- Pose angles ----------
def vec_of(world_lms, lid) -> np.ndarray:
    lm = world_lms.landmark[lid]
    return np.array([lm.x, lm.y, lm.z], dtype=np.float32)

def extract_arm_angles(world_lms, side: str) -> Optional[Dict[str, float]]:
    ids = RIGHT if side == "right" else LEFT
    S, E, W = vec_of(world_lms, ids["SHOULDER"]), vec_of(world_lms, ids["ELBOW"]), vec_of(world_lms, ids["WRIST"])
    I, P    = vec_of(world_lms, ids["INDEX"]), vec_of(world_lms, ids["PINKY"])
    u, f    = E - S, W - E
    hand_x  = I - P
    palm_n  = np.cross(hand_x, (I + P)/2 - W)
    a       = _norm(f)
    yaw     = math.degrees(math.atan2(u[0], -u[2]))                   # base pan
    pitch   = math.degrees(math.atan2(u[1], math.hypot(u[0], u[2])))  # shoulder lift
    elbow   = angle_between(-u, f)                                     # elbow flex
    finger_dir = _norm(((I - W) + (P - W)) * 0.5)
    wrist_flex = signed_angle_around_axis(a, finger_dir, axis=np.cross(a, [0,1,0]) + 1e-8)
    wrist_roll = signed_angle_around_axis(np.array([0,-1,0],np.float32), _norm(palm_n), axis=a)
    return {"base":yaw, "shoulder":pitch, "elbow":elbow, "wrist_flex":wrist_flex, "wrist_roll":wrist_roll}

# ---------- Hands helpers ----------
# indices (MediaPipe Hands)
WRIST_I = 0
TH_TIP, IN_TIP, MI_TIP, RI_TIP, PI_TIP = 4, 8, 12, 16, 20
IN_MCP, MI_MCP, RI_MCP, PI_MCP = 5, 9, 13, 17
IN_PIP, MI_PIP, RI_PIP, PI_PIP = 6, 10, 14, 18
FINGER_SETS = [(IN_MCP, IN_PIP, IN_TIP),(MI_MCP, MI_PIP, MI_TIP),(RI_MCP, RI_PIP, RI_TIP),(PI_MCP, PI_PIP, PI_TIP)]

def pick_hand_for_side(hands_res, side: str) -> Optional[int]:
    if not hands_res or not hands_res.multi_hand_landmarks or not hands_res.multi_handedness: return None
    target = side.lower()
    for i, hd in enumerate(hands_res.multi_handedness):
        if hd.classification[0].label.lower() == target: return i
    return None

def hand_world_array(world_lms) -> np.ndarray:
    """Nx3 array of 3D world landmarks (meters)."""
    return np.array([[lm.x, lm.y, lm.z] for lm in world_lms.landmark], dtype=np.float32)

def count_extended_fingers_3d(hand_world_pts: np.ndarray, pip_angle_open_deg: float = 160.0) -> int:
    ext = 0
    for (mcp_i, pip_i, tip_i) in FINGER_SETS:
        a, b, c = hand_world_pts[mcp_i], hand_world_pts[pip_i], hand_world_pts[tip_i]
        if angle_at_3d(a, b, c) >= pip_angle_open_deg:
            ext += 1
    return ext

def palm_center_3d(hand_world_pts: np.ndarray) -> np.ndarray:
    # average of wrist + 4 MCPs is a decent palm proxy
    idxs = [WRIST_I, IN_MCP, MI_MCP, RI_MCP, PI_MCP]
    return np.mean(hand_world_pts[idxs], axis=0)

def hand_size_scale(hand_world_pts: np.ndarray) -> float:
    # scale ~ average MCP distance from palm center (size-invariant normalization)
    c = palm_center_3d(hand_world_pts)
    idxs = [IN_MCP, MI_MCP, RI_MCP, PI_MCP]
    return float(np.mean([np.linalg.norm(hand_world_pts[i]-c) for i in idxs]) + 1e-6)

def avg_tip_palm_ratio(hand_world_pts: np.ndarray) -> float:
    c = palm_center_3d(hand_world_pts)
    tips = [IN_TIP, MI_TIP, RI_TIP, PI_TIP]
    d = np.mean([np.linalg.norm(hand_world_pts[i]-c) for i in tips])
    return float(d / hand_size_scale(hand_world_pts))

# ---------- Preview ----------
def _norm_for_bar(key: str, value: float) -> float:
    lim = DEFAULT_LIMITS[key]
    return 0.5 if lim.hi == lim.lo else clamp((value - lim.lo) / (lim.hi - lim.lo), 0.0, 1.0)

def draw_preview(adj: Dict[str, float], active_keys: List[str], side: str,
                 hand_method: str, hand_state: Optional[str],
                 fingers_extended: Optional[int], avg_ratio: Optional[float],
                 open_min_fingers: int, fist_ratio_thresh: float, open_ratio_thresh: float):
    H, W = 400, 760
    img = np.full((H, W, 3), 22, dtype=np.uint8)
    method_txt = f"{hand_method}" + (f" ({hand_state})" if hand_state else "")
    title = f"S0101 Preview  |  Side: {side.capitalize()}  |  Hand: {method_txt}"
    cv2.putText(img, title, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (230,230,230), 2, cv2.LINE_AA)

    if fingers_extended is not None and avg_ratio is not None:
        info = f"Fingers: {fingers_extended}  avg_tip_palm_ratio: {avg_ratio:.2f}  (OPEN if >= {open_min_fingers} or ratio>={open_ratio_thresh:.2f}; CLOSE if <=1 and ratio<={fist_ratio_thresh:.2f})"
        cv2.putText(img, info, (12, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200,200,200), 1, cv2.LINE_AA)

    y = 76
    line_h = 42
    name_x, bar_x, bar_w, bar_h = 14, 300, 360, 16

    for motor_id, name, key in MOTOR_MAP:
        if key in active_keys: cv2.rectangle(img, (8, y-22), (W-8, y+16), (60,90,200), -1)
        cv2.putText(img, f"M{motor_id}  {name}", (name_x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255,255,255), 1, cv2.LINE_AA)
        v = adj.get(key, 0.0)
        if key != "gripper":
            cv2.putText(img, f"{v:+6.1f}°", (bar_x-120, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (220,220,220), 1, cv2.LINE_AA)
            p = _norm_for_bar(key, v)
            x0, y0 = bar_x, y - bar_h + 4
            cv2.rectangle(img, (x0, y0), (x0+bar_w, y0+bar_h), (80,80,80), 1)
            cv2.rectangle(img, (x0, y0), (x0+int(bar_w*p), y0+bar_h), (120,200,120), -1)
            cv2.line(img, (x0+bar_w//2, y0+bar_h+2), (x0+bar_w//2, y0+bar_h+6), (150,150,150), 1)
        else:
            state = "CLOSE" if v >= 50.0 else "OPEN"
            cv2.putText(img, f"{state} ({v:4.0f}%)", (bar_x-180, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (220,220,220), 1, cv2.LINE_AA)
            x0, y0 = bar_x, y - bar_h + 4
            cv2.rectangle(img, (x0, y0), (x0+bar_w, y0+bar_h), (80,80,80), 1)
            cv2.rectangle(img, (x0, y0), (x0+int(bar_w*clamp(v/100.0,0,1)), y0+bar_h), (200,150,120), -1)
        y += line_h

    cv2.imshow("S0101 Preview", img)

# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", choices=["right","left"], default="right")
    ap.add_argument("--backend", choices=["print","udp"], default="print")
    ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=7777)
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--alpha", type=float, default=0.3)  # EMA smoothing for angles
    ap.add_argument("--max_fps", type=float, default=30.0)
    ap.add_argument("--active_eps_deg", type=float, default=1.5)
    ap.add_argument("--open_min_fingers", type=int, default=2, help="OPEN if extended >= this")
    ap.add_argument("--pip_angle_open_deg", type=float, default=160.0, help="Per-finger PIP angle (3D) to count as extended")
    ap.add_argument("--fist_ratio_thresh", type=float, default=0.28, help="CLOSE if avg_tip_palm_ratio <= this (and extended <= 1)")
    ap.add_argument("--open_ratio_thresh", type=float, default=0.45, help="OPEN if avg_tip_palm_ratio >= this")
    args = ap.parse_args()

    backend = UDPBackend(args.host, args.port) if args.backend == "udp" else PrintBackend()
    filters = {k: EMA(alpha=args.alpha) for k in ["base","shoulder","elbow","wrist_flex","wrist_roll","gripper"]}
    offsets, sending, last_adj = Offsets(), True, None
    last_grip_pct = 0.0  # for hysteresis keep-state when ambiguous

    cap = cv2.VideoCapture(args.camera); cap.set(cv2.CAP_PROP_FPS, args.max_fps)
    mp_drawing, mp_pose, mp_hands = mp.solutions.drawing_utils, mp.solutions.pose, mp.solutions.hands

    last_send_t = 0.0
    try:
        with mp_pose.Pose(static_image_mode=False, model_complexity=1,
                          enable_segmentation=False, min_detection_confidence=0.5,
                          min_tracking_confidence=0.5) as pose, \
             mp_hands.Hands(static_image_mode=False, max_num_hands=2, model_complexity=0,
                             min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:

            while True:
                ok, frame = cap.read()
                if not ok: print("No frame from camera."); break
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                pose_res  = pose.process(rgb)
                hands_res = hands.process(rgb)
                hand_idx  = pick_hand_for_side(hands_res, side=args.side)
                hand_method = "Hands" if hand_idx is not None else "Fallback"
                hand_state = None
                fingers_extended, avg_ratio = None, None

                if pose_res.pose_landmarks is not None:
                    mp_drawing.draw_landmarks(frame, pose_res.pose_landmarks, mp_pose.POSE_CONNECTIONS)
                if hand_idx is not None:
                    mp_drawing.draw_landmarks(frame, hands_res.multi_hand_landmarks[hand_idx], mp_hands.HAND_CONNECTIONS)

                hud_lines, adj = [], None
                arm = extract_arm_angles(pose_res.pose_world_landmarks, side=args.side) if pose_res.pose_world_landmarks else None

                if arm is not None:
                    # --- Gripper via hands 3D ---
                    if hand_idx is not None and hands_res.multi_hand_world_landmarks:
                        hw = hand_world_array(hands_res.multi_hand_world_landmarks[hand_idx])
                        fingers_extended = count_extended_fingers_3d(hw, pip_angle_open_deg=args.pip_angle_open_deg)
                        avg_ratio = avg_tip_palm_ratio(hw)

                        # Decision with hysteresis
                        if fingers_extended >= args.open_min_fingers or avg_ratio >= args.open_ratio_thresh:
                            grip_pct = 0.0; hand_state = "OPEN"
                        elif fingers_extended <= 1 and avg_ratio <= args.fist_ratio_thresh:
                            grip_pct = 100.0; hand_state = "CLOSE"
                        else:
                            # ambiguous -> keep previous state
                            grip_pct = last_grip_pct
                            hand_state = "OPEN" if grip_pct < 50.0 else "CLOSE"
                    else:
                        # Fallback: pinch vs hand width (coarser, from pose)
                        ids = RIGHT if args.side == "right" else LEFT
                        if pose_res.pose_world_landmarks:
                            I, T, P = vec_of(pose_res.pose_world_landmarks, ids["INDEX"]), vec_of(pose_res.pose_world_landmarks, ids["THUMB"]), vec_of(pose_res.pose_world_landmarks, ids["PINKY"])
                            pinch, hand_width = float(np.linalg.norm(T - I)), float(np.linalg.norm(I - P)) + 1e-6
                            ratio = pinch / hand_width
                            grip_pct = 100.0 * clamp(1.0 - ratio, 0.0, 1.0)
                            hand_state = "OPEN" if grip_pct < 50.0 else "CLOSE"
                        else:
                            grip_pct = last_grip_pct
                            hand_state = "OPEN" if grip_pct < 50.0 else "CLOSE"

                    # Apply offsets/limits + filtering (snap gripper instantly)
                    raw = {"base":arm["base"], "shoulder":arm["shoulder"], "elbow":arm["elbow"],
                           "wrist_flex":arm["wrist_flex"], "wrist_roll":arm["wrist_roll"], "gripper":grip_pct}
                    adj = {}
                    for k, v in raw.items():
                        if k != "gripper": v = v - getattr(offsets, k)
                        lim = DEFAULT_LIMITS[k]
                        if lim.invert: v = -v
                        v = clamp(v, lim.lo, lim.hi)
                        if k == "gripper":
                            filters[k].x = float(v); adj[k] = float(v)  # no smoothing for gripper
                            last_grip_pct = float(v)
                        else:
                            adj[k] = filters[k].update(v)

                    payload = {
                        "t": time.time(), "side": args.side,
                        "units": {"joints": "deg", "gripper": "percent"},
                        "joints_deg": { "base_pan":float(adj["base"]), "shoulder_lift":float(adj["shoulder"]),
                                        "elbow_flex":float(adj["elbow"]), "wrist_flex":float(adj["wrist_flex"]),
                                        "wrist_roll":float(adj["wrist_roll"]) },
                        "gripper_pct": float(adj["gripper"]),
                        "meta": {"hand_method": hand_method, "fingers_extended": fingers_extended, "avg_tip_palm_ratio": avg_ratio}
                    }

                    now = time.time()
                    if sending and (now - last_send_t) >= (1.0 / max(args.max_fps, 1.0)):
                        backend.send(payload); last_send_t = now

                    # HUD
                    hud_lines.append(f"SEND: {'ON' if sending else 'OFF'}")
                    hud_lines.append(f"Base  (pan): {adj['base']:6.1f}°")
                    hud_lines.append(f"Should(lift): {adj['shoulder']:6.1f}°")
                    hud_lines.append(f"Elbow (flex): {adj['elbow']:6.1f}°")
                    hud_lines.append(f"Wrist (flex): {adj['wrist_flex']:6.1f}°")
                    hud_lines.append(f"Wrist (roll): {adj['wrist_roll']:6.1f}°")
                    state = "CLOSE" if adj['gripper'] >= 50.0 else "OPEN"
                    extra = f"  method={hand_method}"
                    if fingers_extended is not None: extra += f"  fingers={fingers_extended}"
                    if avg_ratio is not None:       extra += f"  ratio={avg_ratio:.2f}"
                    hud_lines.append(f"Gripper   (%): {adj['gripper']:6.1f}  [{state}]{extra}")
                else:
                    hud_lines.append("Pose not detected")

                # HUD on camera
                y = 20
                for line in hud_lines:
                    cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,0), 1, cv2.LINE_AA); y += 20
                cv2.putText(frame, "Keys: [c]=zero angles  [z]=reset  [space]=send  [q]=quit",
                            (10, frame.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255,255,255), 1, cv2.LINE_AA)
                cv2.imshow("Leader Arm (MediaPipe)", frame)

                # --- Preview ---
                if adj is None and last_adj is not None: adj_for_preview = last_adj
                elif adj is not None: adj_for_preview = adj
                else: adj_for_preview = {k:0.0 for k in ["base","shoulder","elbow","wrist_flex","wrist_roll","gripper"]}

                active_keys = []
                if last_adj is not None and adj_for_preview is not None:
                    for k in ["base","shoulder","elbow","wrist_flex","wrist_roll"]:
                        if abs(adj_for_preview[k] - last_adj[k]) >= args.active_eps_deg: active_keys.append(k)
                    if abs(adj_for_preview["gripper"] - last_adj["gripper"]) >= 5.0: active_keys.append("gripper")

                draw_preview(adj_for_preview, active_keys, side=args.side,
                             hand_method=hand_method, hand_state=("OPEN" if adj_for_preview["gripper"]<50 else "CLOSE"),
                             fingers_extended=fingers_extended, avg_ratio=avg_ratio,
                             open_min_fingers=args.open_min_fingers,
                             fist_ratio_thresh=args.fist_ratio_thresh, open_ratio_thresh=args.open_ratio_thresh)
                last_adj = adj_for_preview

                # Keys
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'): break
                elif key == ord(' '): sending = not sending
                elif key == ord('z'):
                    offsets = Offsets()
                    for f in filters.values(): f.x = None
                    print("[reset] offsets cleared.")
                elif key == ord('c') and pose_res.pose_world_landmarks:
                    ang = extract_arm_angles(pose_res.pose_world_landmarks, side=args.side)
                    if ang:
                        offsets = Offsets(base=ang["base"], shoulder=ang["shoulder"], elbow=ang["elbow"],
                                          wrist_flex=ang["wrist_flex"], wrist_roll=ang["wrist_roll"], gripper=0.0)
                        for f in filters.values(): f.x = None
                        print("[calibrate] current pose captured as zero-offset for angles.")
    finally:
        backend.close(); cap.release(); cv2.destroyAllWindows()

if __name__ == "__main__": main()
