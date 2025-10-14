from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Dict, Any
import numpy as np

@dataclass
class Frame:
    head: np.ndarray                 # 4x4 (world->head)
    right_wrist: np.ndarray          # 4x4 (world->wrist)
    right_wrist_roll_deg: float
    right_pinch_m: Optional[float] = None
    extra: Optional[Dict[str, Any]] = None

    def validate(self) -> None:
        assert self.head.shape == (4,4)
        assert self.right_wrist.shape == (4,4)
