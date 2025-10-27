# Dabrius

Stream head-relative wrist pose, wrist roll, and pinch from Apple Vision Pro to control a LeRobot SO-101 arm with safety mapping.

## Requirements

**Vision Pro**
- DabriusStreamer app (visionOS target in this project)

**Laptop**
- Same Wi-Fi network as Vision Pro
- Python environment with dependencies:

```bash
pip install -e .   # from dabrius/ directory
```

Dependencies include: `numpy`, `websockets>=12`, `lerobot`

## Setup

### Vision Pro

1. Build and run DabriusStreamer from Xcode (visionOS target)
2. Grant permissions on first launch:
   - World Sensing
   - Hands Tracking
   - Main Camera
   - Local Network
3. The window will hide after starting (no persistent UI)
4. Reopen controls: double-tap anywhere, long-press (0.5s), or use top-left hot-corner
5. Note the WebSocket URL displayed before starting: `ws://<IP>:8211/stream`

### Laptop

**Test connection (optional)**

```python
python - <<'PY'
import json
from websockets.sync.client import connect
IP = "<<HEADSET_IP>>"
with connect(f"ws://{IP}:8211/stream", open_timeout=5) as ws:
    print(list(json.loads(ws.recv()).keys()))
PY
```

**Start teleoperation**

```bash
dabrius-teleop --ws-host <<HEADSET_IP>> \
               --serial-port /dev/tty.usbmodem58FD0170541 \
               --robot-id dabrius
```

## Control Mapping

| Body Movement | Robot Joint |
|---------------|-------------|
| Hand left/right | `shoulder_pan` |
| Hand up/down | `shoulder_lift` (asymmetric, favors forward) |
| Hand forward/back | `elbow_flex` (backward limited) |
| Wrist pitch | `wrist_flex` (decoupled from vertical + speed gate) |
| Wrist roll | `wrist_roll` |
| Pinch (thumb–index) | `gripper` |

Toggle individual motors on/off using the Active Motors control in the headset UI.

## Calibration

Normalized commands (−100 to 100) are mapped using your LeRobot calibration file:

```
~/.cache/huggingface/lerobot/calibration/robots/so101_follower/<robot_id>.json
```

Specify `--robot-id` if your calibration file differs from the default `dabrius`. A safe fallback is used if the file is missing
---

© Darius Giannoli & Gabriel Taieb
