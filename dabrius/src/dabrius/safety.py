from __future__ import annotations
from typing import Dict

SAFE = {
    "shoulder_pan":  (-60.0,  60.0),
    "shoulder_lift": (-20.0,  85.0),
    "elbow_flex":    (-20.0,  75.0),
    "wrist_flex":    (-60.0,  60.0),
    "wrist_roll":    (-75.0,  75.0),
    "gripper":       (-100.0, 100.0),
}

def clamp_targets(targets: Dict[str, float]) -> Dict[str, float]:
    for k,(lo,hi) in SAFE.items():
        if k in targets:
            v = targets[k]
            if v < lo: v = lo
            if v > hi: v = hi
            targets[k] = v
    return targets

def deadzone(curr: float, target: float, dz: float) -> float:
    return curr if abs(target - curr) < dz else target

def smooth(curr: float, target: float, a: float) -> float:
    return a*curr + (1-a)*target
