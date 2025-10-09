from dataclasses import dataclass

try:
    # If LeRobot exposes a TeleoperatorConfig base (recommended path)
    from lerobot.teleoperators.config import TeleoperatorConfig
except Exception:
    # Fallback minimal struct for local testing
    @dataclass
    class TeleoperatorConfig:
        pass

@dataclass
class VProTeleopConfig(TeleoperatorConfig):  # type: ignore[misc]
    udp_host: str = "0.0.0.0"
    udp_port: int = 8765
    pinch_close_mm: float = 25.0
    pinch_open_mm: float = 55.0
    pos_scale: float = 1.0

    # Safety / smoothing (applied downstream in your pipeline)
    ee_bounds: dict | None = None
    max_ee_step_m: float | None = None
    smoothing: dict | None = None

    hand: str = "right"  # "right" or "left"
