from __future__ import annotations
import asyncio, json, math, threading
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

import numpy as np
from .config_vpro import VProTeleopConfig

# Try to import LeRobot interfaces (if present)
try:
    from lerobot.teleoperators.teleoperator import Teleoperator  # type: ignore
except Exception:
    # Minimal shim so you can still run the control server without LeRobot
    class Teleoperator:
        def __init__(self, config): self.config = config
        def connect(self): pass
        def disconnect(self): pass
        @property
        def action_features(self) -> dict: return {}
        def get_action(self) -> dict: return {}

class _UDPStore(asyncio.DatagramProtocol):
    def __init__(self):
        self.latest: Optional[dict] = None

    def datagram_received(self, data, addr):
        try:
            msg = json.loads(data.decode("utf-8"))
        except Exception:
            return
        if msg.get("type") == "pose":
            self.latest = msg

def _quat_to_rpy(qw, qx, qy, qz):
    siny_cosp = 2*(qw*qz + qx*qy);  cosy_cosp = 1 - 2*(qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    sinp = 2*(qw*qy - qz*qx);       pitch = math.asin(max(-1.0, min(1.0, sinp)))
    sinr_cosp = 2*(qw*qx + qy*qz);  cosr_cosp = 1 - 2*(qx*qx + qy*qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    return roll, pitch, yaw  # (wx, wy, wz)

@dataclass
class VProTeleop(Teleoperator):  # type: ignore[misc]
    config: VProTeleopConfig
    _store: _UDPStore = field(default_factory=_UDPStore, init=False)
    _transport: Any = field(default=None, init=False)
    _loop: Any = field(default=None, init=False)
    _enabled: bool = field(default=True, init=False)
    _origin_pos: np.ndarray | None = field(default=None, init=False)   # 3
    _origin_quat: np.ndarray | None = field(default=None, init=False)  # 4
    _hand: str = field(default="right", init=False)

    # Safety mirror (to push into your LeRobot processors)
    _ee_bounds: dict | None = field(default=None, init=False)
    _max_step: float | None = field(default=None, init=False)
    _smooth: dict | None = field(default=None, init=False)

    def __post_init__(self):
        self._hand = self.config.hand
        self._ee_bounds = self.config.ee_bounds
        self._max_step = self.config.max_ee_step_m
        self._smooth = self.config.smoothing

    def connect(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        listen = self._loop.create_datagram_endpoint(
            lambda: self._store, local_addr=(self.config.udp_host, self.config.udp_port)
        )
        self._transport, _ = self._loop.run_until_complete(listen)

        def pump():
            self._loop.run_forever()
        threading.Thread(target=pump, daemon=True).start()

    def disconnect(self):
        if self._transport:
            self._transport.close()
        if self._loop:
            self._loop.call_soon_threadsafe(self._loop.stop)

    @property
    def action_features(self) -> dict:
        # End-effector pose (x,y,z, wx,wy,wz) + gripper
        feats = {f"ee.{k}": {"shape": (1,)} for k in ["x","y","z","wx","wy","wz","gripper_pos"]}
        return feats

    def set_hand(self, hand: str):
        if hand in ("right","left"):
            self._hand = hand

    def set_safety(self, ee_bounds=None, max_ee_step_m=None, smoothing=None):
        if ee_bounds is not None: self._ee_bounds = ee_bounds
        if max_ee_step_m is not None: self._max_step = max_ee_step_m
        if smoothing is not None: self._smooth = smoothing

    def capture_origin_from_current_wrist(self) -> bool:
        r = self._store.latest
        if not r: return False
        self._origin_pos = np.array(r["pos"], dtype=np.float64)
        self._origin_quat = np.array(r["rot"], dtype=np.float64)  # (w,x,y,z)
        return True

    def enable(self, flag: bool): self._enabled = flag
    def estop(self): self._enabled = False

    def _apply_origin(self, pos: np.ndarray, quat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        # Simple relative transform: subtract origin pos; ignore origin quat for stability
        if self._origin_pos is not None:
            pos = pos - self._origin_pos
        return pos, quat

    def get_action(self) -> Dict[str, float]:
        if not self._enabled:
            return {"ee.x":0,"ee.y":0,"ee.z":0,"ee.wx":0,"ee.wy":0,"ee.wz":0,"gripper_pos":0.0}

        r = self._store.latest
        if not r or r.get("hand") != self._hand:
            return {"ee.x":0,"ee.y":0,"ee.z":0,"ee.wx":0,"ee.wy":0,"ee.wz":0,"gripper_pos":0.0}

        pos = np.array(r["pos"], dtype=np.float64)
        quat = np.array(r["rot"], dtype=np.float64)  # (w,x,y,z)
        pos, quat = self._apply_origin(pos, quat)

        wx, wy, wz = _quat_to_rpy(*quat.tolist())
        s = float(self.config.pos_scale)
        ee = {
            "ee.x": s * float(pos[0]),
            "ee.y": s * float(pos[1]),
            "ee.z": s * float(pos[2]),
            "ee.wx": float(wx),
            "ee.wy": float(wy),
            "ee.wz": float(wz),
        }

        # Pinch → gripper in [0..1] with hysteresis
        d = float(r.get("pinch_mm", 1e9))
        o, c = float(self.config.pinch_open_mm), float(self.config.pinch_close_mm)
        g = (o - d) / max(1e-6, (o - c))
        ee["gripper_pos"] = float(np.clip(g, 0.0, 1.0))
        return ee
