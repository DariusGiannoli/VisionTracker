from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Dict, Tuple
import numpy as np

def rot_to_pitch_deg(R: np.ndarray) -> float:
    v = -R[2,0]
    v = float(max(-1.0, min(1.0, v)))
    return math.degrees(math.asin(v))

@dataclass
class HeadRelative:
    home_t: np.ndarray   # (3,)
    home_R: np.ndarray   # (3,3)

    @staticmethod
    def from_samples(ts: list[np.ndarray], Rs: list[np.ndarray]) -> "HeadRelative":
        t0 = np.median(np.stack(ts), axis=0) if ts else np.zeros(3, dtype=np.float32)
        R0 = np.median(np.stack(Rs), axis=0) if Rs else np.eye(3, dtype=np.float32)
        return HeadRelative(home_t=t0.astype(np.float32), home_R=R0.astype(np.float32))

    @staticmethod
    def to_head_components(wrist44_headrel: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        return wrist44_headrel[:3,3].astype(np.float32), wrist44_headrel[:3,:3].astype(np.float32)

@dataclass
class TeleopMapper:
    # Shoulder pan / lift / elbow gains
    DEG_PER_M_X: float = 220.0
    GAIN_UP_Y: float = 260.0
    GAIN_DOWN_Y: float = 140.0
    SHOULDER_LIFT_BIAS_DEG: float = 8.0
    DEG_PER_M_Z: float = -120.0

    # Wrist options
    WRIST_FLEX_INVERT: bool = True
    WRIST_ROLL_INVERT: bool = True
    VERT_SPEED_GATE_MPS: float = 0.15
    COUPLING_Y2PITCH: float = 0.0

    def learn_decoupling(self, dy: np.ndarray, pitch_deg: np.ndarray) -> None:
        var_dy = float(np.var(dy)) if dy.size else 0.0
        self.COUPLING_Y2PITCH = float(np.cov(dy, pitch_deg, ddof=0)[0,1] / var_dy) if var_dy > 1e-6 else 0.0

    def to_targets(
        self, wrist44_headrel: np.ndarray, home_t: np.ndarray,
        prev_dy: float, dt: float, prev_wrist_flex: float,
        wrist_roll_deg: float, pinch_m: float | None
    ) -> Tuple[Dict[str, float], float]:
        t_rel, R_rel = HeadRelative.to_head_components(wrist44_headrel)
        dx, dy, dz = float(t_rel[0]-home_t[0]), float(t_rel[1]-home_t[1]), float(t_rel[2]-home_t[2])
        dy_rate = (dy - prev_dy)/dt

        tgt: Dict[str, float] = {}
        tgt["shoulder_pan"] = dx * self.DEG_PER_M_X
        tgt["shoulder_lift"] = dy * (self.GAIN_UP_Y if dy>=0 else self.GAIN_DOWN_Y) + self.SHOULDER_LIFT_BIAS_DEG
        tgt["elbow_flex"] = dz * self.DEG_PER_M_Z

        pitch_deg = rot_to_pitch_deg(R_rel) - self.COUPLING_Y2PITCH * dy
        if self.WRIST_FLEX_INVERT: pitch_deg = -pitch_deg
        if abs(dy_rate) > self.VERT_SPEED_GATE_MPS: pitch_deg = prev_wrist_flex
        tgt["wrist_flex"] = pitch_deg

        roll_deg = float(wrist_roll_deg)
        if self.WRIST_ROLL_INVERT: roll_deg = -roll_deg
        tgt["wrist_roll"] = roll_deg

        if pinch_m is not None:
            close, open_ = 0.03, 0.09
            p = max(0.0, min(0.2, float(pinch_m)))
            g = 200.0 * (p - close) / (open_ - close) - 100.0
            tgt["gripper"] = float(max(-100.0, min(100.0, g)))

        return tgt, dy
