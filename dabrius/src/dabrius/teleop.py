#!/usr/bin/env python3
"""
Teleop loop for LeRobot SO-101 using the Dabrius Vision Pro stream.

Adds proper Feetech calibration loading so normalized commands (-100..100)
map to real servo positions. Falls back to conservative ranges if the JSON
calibration file is missing.

Calibration file (created by lerobot-calibrate) is expected at:
~/.cache/huggingface/lerobot/calibration/robots/so101_follower/<robot_id>.json
"""

from __future__ import annotations
import asyncio, time, json
from typing import Dict
from pathlib import Path

import numpy as np

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

from .client import VisionStreamClient
from .mapping import HeadRelative, TeleopMapper
from .safety import SAFE, clamp_targets, deadzone, smooth


# ----------------------------- Calibration -----------------------------

class CalibrationData:
    """Simple attribute-holder compatible with FeetechMotorsBus expectations."""
    def __init__(self, data: dict):
        self.id = data["id"]
        self.drive_mode = data.get("drive_mode", 0)
        self.homing_offset = data.get("homing_offset", 0)
        self.range_min = data["range_min"]
        self.range_max = data["range_max"]

class SimpleCalibration:
    """Indexable by motor name: calibration['shoulder_pan'] -> CalibrationData."""
    def __init__(self, per_motor_dict: dict[str, dict]):
        self.data = {name: CalibrationData(d) for name, d in per_motor_dict.items()}
    def __getitem__(self, key: str) -> CalibrationData | None:
        return self.data.get(key)

# Conservative fallback (based on your previous values/logs)
FALLBACK_CALIB = {
    "shoulder_pan":  {"id": 1, "drive_mode": 0, "homing_offset": 0,   "range_min":  679, "range_max": 3411},
    "shoulder_lift": {"id": 2, "drive_mode": 0, "homing_offset": 0,   "range_min":  519, "range_max": 3042},
    "elbow_flex":    {"id": 3, "drive_mode": 0, "homing_offset": 0,   "range_min":  969, "range_max": 3196},
    "wrist_flex":    {"id": 4, "drive_mode": 0, "homing_offset": 0,   "range_min":  816, "range_max": 3168},
    "wrist_roll":    {"id": 5, "drive_mode": 0, "homing_offset": 0,   "range_min":  552, "range_max": 3795},
    # Gripper homing_offset (980) kept from your logs so open/close centers correctly
    "gripper":       {"id": 6, "drive_mode": 0, "homing_offset": 980, "range_min": 1999, "range_max": 3443},
}

def load_calibration(robot_id: str, motor_keys: list[str]) -> SimpleCalibration:
    """
    Load per-motor calibration for the given robot_id.
    Any missing motor in the file falls back to FALLBACK_CALIB.
    """
    path = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower" / f"{robot_id}.json"
    per_motor: dict[str, dict] = {}
    if path.exists():
        with path.open() as f:
            data = json.load(f)
        print(f"✓ Calibration loaded: {path}")
        for k in motor_keys:
            if k in data:
                per_motor[k] = data[k]
            else:
                print(f"⚠️  Missing '{k}' in {path.name}, using fallback.")
                per_motor[k] = FALLBACK_CALIB[k]
    else:
        print(f"⚠️  No calibration file at {path} — using fallback for all motors.")
        for k in motor_keys:
            per_motor[k] = FALLBACK_CALIB[k]
    return SimpleCalibration(per_motor)


# ------------------------------ Motors --------------------------------

DEFAULT_MOTORS = {
    "shoulder_pan":  {"id": 1, "model": "sts3215"},
    "shoulder_lift": {"id": 2, "model": "sts3215"},
    "elbow_flex":    {"id": 3, "model": "sts3215"},
    "wrist_flex":    {"id": 4, "model": "sts3215"},
    "wrist_roll":    {"id": 5, "model": "sts3215"},
    "gripper":       {"id": 6, "model": "sts3215"},
}


# ------------------------------- Loop ---------------------------------

async def _run(
    serial_port: str,
    ws_host: str,
    ws_port: int,
    ws_path: str,
    update_hz: float = 20.0,
    smoothing: float = 0.8,
    deadzone_deg: float = 2.0,
    robot_id: str = "dabrius",     # <— default robot ID for calibration file
):
    # Connect to Vision Pro stream
    client = VisionStreamClient(ws_host, ws_port, ws_path)
    await client.connect()

    # Prepare motors and calibration
    motors = {
        name: Motor(cfg["id"], cfg["model"], MotorNormMode.RANGE_M100_100)
        for name, cfg in DEFAULT_MOTORS.items()
    }
    calibration = load_calibration(robot_id, motor_keys=list(motors.keys()))

    # Bus
    bus = FeetechMotorsBus(port=serial_port, motors=motors, calibration=calibration)
    bus.connect()
    pos = {name: 0.0 for name in motors}

    # Home capture (~1s) from head-relative wrist
    ts, Rs = [], []
    t0 = time.time()
    while time.time() - t0 < 1.0:
        msg = await client.ws.recv()
        data = json.loads(msg)
        wrist44 = np.asarray(data["head_rel"]["right_wrist_44"], dtype=np.float32)
        t_rel, R_rel = HeadRelative.to_head_components(wrist44)
        ts.append(t_rel); Rs.append(R_rel)
    headrel = HeadRelative.from_samples(ts, Rs)

    # Learn decoupling (~3s at update_hz)
    mapper = TeleopMapper()
    DY, P = [], []
    for _ in range(int(3 * update_hz)):
        msg = await client.ws.recv()
        data = json.loads(msg)
        wrist44 = np.asarray(data["head_rel"]["right_wrist_44"], dtype=np.float32)
        t_rel, R_rel = HeadRelative.to_head_components(wrist44)
        DY.append(t_rel[1] - headrel.home_t[1])
        v = -R_rel[2, 0]
        v = float(max(-1.0, min(1.0, v)))
        P.append(np.degrees(np.arcsin(v)))
    mapper.learn_decoupling(np.asarray(DY), np.asarray(P))

    dt = 1.0 / update_hz
    prev_dy = 0.0

    try:
        while True:
            msg = await client.ws.recv()
            data = json.loads(msg)
            hr = data["head_rel"]
            wrist44 = np.asarray(hr["right_wrist_44"], dtype=np.float32)
            wrist_roll = float(hr.get("right_wrist_roll_deg", 0.0))
            pinch_m = float(hr["right_pinch_m"]) if "right_pinch_m" in hr and hr["right_pinch_m"] is not None else None

            targets, prev_dy = mapper.to_targets(
                wrist44_headrel=wrist44,
                home_t=headrel.home_t,
                prev_dy=prev_dy,
                dt=dt,
                prev_wrist_flex=pos["wrist_flex"],
                wrist_roll_deg=wrist_roll,
                pinch_m=pinch_m,
            )
            targets = clamp_targets(targets)

            # Optional: honor active motor flags sent by the headset app
            active = data.get("active", None)  # dict like {"shoulder_pan": true, ...}

            for name, val in targets.items():
                if isinstance(active, dict) and active.get(name) is False:
                    continue  # skip disabled motor

                # Asymmetric dead-zone for shoulder_lift when commanding backward
                dz_local = deadzone_deg * (1.5 if (name == "shoulder_lift" and val < 0) else 1.0)
                val = deadzone(pos[name], val, dz_local)
                pos[name] = smooth(pos[name], val, smoothing)

                try:
                    bus.write("Goal_Position", name, pos[name])
                except RuntimeError as e:
                    print(f"[WARN] {name}: {e}")

    finally:
        # Return to neutral and cleanup
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
        await client.close()


def run_teleop(
    ws_host: str,
    serial_port: str,
    ws_port: int = 8211,
    ws_path: str = "/stream",
    robot_id: str = "dabrius",   # keep default for CLI compatibility
):
    asyncio.run(
        _run(
            serial_port=serial_port,
            ws_host=ws_host,
            ws_port=ws_port,
            ws_path=ws_path,
            robot_id=robot_id,
        )
    )
