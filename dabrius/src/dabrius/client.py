from __future__ import annotations
import asyncio, json
from typing import AsyncIterator, Any
import numpy as np
import websockets

from .schemas import Frame

def _mat44(x: Any) -> np.ndarray:
    a = np.asarray(x, dtype=np.float32)
    if a.size == 16: a = a.reshape(4,4)
    assert a.shape == (4,4), f"expected 4x4, got {a.shape}"
    return a

class VisionStreamClient:
    """
    Minimal WebSocket JSON client for Dabrius Streamer.
    Messages (20–60 Hz):
    {
      "schema": "dabrius/v1",
      "t": 123456.7,
      "head_rel": {
        "right_wrist_44": [[...],[...],[...],[...]],
        "right_wrist_roll_deg": 12.3,
        "right_pinch_m": 0.07
      },
      "quality": {...}
    }
    """
    def __init__(self, host: str, port: int = 8211, path: str = "/stream", timeout: float = 5.0):
        self.url = f"ws://{host}:{port}{path}"
        self.timeout = timeout
        self.ws = None

    async def connect(self):
        self.ws = await asyncio.wait_for(websockets.connect(self.url, max_size=2**23), timeout=self.timeout)

    async def close(self):
        if self.ws:
            try: await self.ws.close()
            except Exception: pass

    async def frames(self) -> AsyncIterator[Frame]:
        while True:
            raw = await self.ws.recv()
            if isinstance(raw, (bytes, bytearray)): raw = raw.decode("utf-8")
            data = json.loads(raw)
            hr = data.get("head_rel", {})
            # These are already HEAD-RELATIVE (good!)
            rw44 = _mat44(hr["right_wrist_44"])
            # head matrix not strictly needed (head-rel), but keep identity for API symmetry
            head44 = np.eye(4, dtype=np.float32)

            fr = Frame(
                head=head44,
                right_wrist=rw44,
                right_wrist_roll_deg=float(hr.get("right_wrist_roll_deg", 0.0)),
                right_pinch_m=float(hr["right_pinch_m"]) if hr.get("right_pinch_m") is not None else None,
                extra={"schema": data.get("schema", "dabrius/v1")}
            )
            fr.validate()
            yield fr
