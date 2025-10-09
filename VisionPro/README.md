# Repository layout

```
lerobot-visionpro-teleop/
├── README.md
├── visionos-app/
│   ├── LeRobotVPApp.swift
│   ├── StreamerView.swift
│   ├── DiscoveryView.swift
│   ├── Networking/
│   │   ├── UDPClient.swift
│   │   └── ControlClient.swift
│   ├── Models/
│   │   └── AppState.swift
│   └── Info.plist.snippet.xml
└── python-teleop/
    ├── pyproject.toml
    ├── run_with_lerobot_cli.sh
    ├── control_server.py
    ├── mdns_advertise.py         # optional (Bonjour/mDNS)
    └── lerobot_teleoperator_vpro/
        ├── __init__.py
        ├── config_vpro.py
        └── vpro.py
```

---

# visionOS app (Swift / RealityKit / ARKit / Network)

## `visionos-app/LeRobotVPApp.swift`

```swift
import SwiftUI

@main
struct LeRobotVPApp: App {
    @StateObject private var appState = AppState()

    var body: some SwiftUI.Scene {
        // An Immersive Space so we can access ARKit hand tracking
        ImmersiveSpace(id: "MainSpace") {
            StreamerView()
                .environmentObject(appState)
        }
        .upperLimbVisibility(.hidden) // optional: hide system arms

        // A small window to manage connection & settings
        WindowGroup("Control") {
            DiscoveryView()
                .environmentObject(appState)
        }
    }
}
```

## `visionos-app/Models/AppState.swift`

```swift
import Foundation
import simd

final class AppState: ObservableObject {
    // Discovered/selected server
    @Published var serverHost: String = ""
    @Published var controlPort: UInt16 = 8766
    @Published var dataPort: UInt16 = 8765

    // Runtime toggles
    @Published var useRightHand: Bool = true
    @Published var streamingEnabled: Bool = false

    // Calibration: transform from app-origin to robot-base (set on "Calibrate")
    @Published var originFromApp: simd_float4x4 = matrix_identity_float4x4

    // Pinch thresholds (mm)
    @Published var pinchCloseMM: Float = 25
    @Published var pinchOpenMM:  Float = 55

    // Safety / mapping (also mirrored to Python via control plane)
    @Published var posScale: Float = 1.0
    @Published var eeBounds: [String:[Float]] = [
        "x":[-0.25, 0.25], "y":[0.00, 0.40], "z":[0.00, 0.35]
    ]
    @Published var maxEEStepM: Float = 0.01
    @Published var smoothingAlpha: Float = 0.2
}
```

## `visionos-app/Networking/UDPClient.swift`

```swift
import Foundation
import Network

final class UDPClient {
    private var connection: NWConnection?

    func connect(host: String, port: UInt16) {
        let params = NWParameters.udp
        let h = NWEndpoint.Host(host)
        let p = NWEndpoint.Port(rawValue: port)!
        let c = NWConnection(host: h, port: p, using: params)
        c.stateUpdateHandler = { _ in }
        c.start(queue: .main)
        self.connection = c
    }

    func send(_ data: Data) {
        connection?.send(content: data, completion: .contentProcessed { _ in })
    }
}
```

## `visionos-app/Networking/ControlClient.swift`

```swift
import Foundation
import Network

final class ControlClient {
    private var connection: NWConnection?

    func connect(host: String, port: UInt16) {
        let h = NWEndpoint.Host(host)
        let p = NWEndpoint.Port(rawValue: port)!
        let c = NWConnection(host: h, port: p, using: .tcp)
        c.stateUpdateHandler = { _ in }
        c.start(queue: .main)
        connection = c
    }

    func sendJSON(_ json: [String: Any]) {
        guard let data = try? JSONSerialization.data(withJSONObject: json) else { return }
        // newline-delimited JSON for easy framing
        let payload = data + Data("\n".utf8)
        connection?.send(content: payload, completion: .contentProcessed { _ in })
    }
}
```

## `visionos-app/StreamerView.swift`

```swift
import SwiftUI
import RealityKit
import ARKit
import simd

struct StreamerView: View {
    @EnvironmentObject var app: AppState

    private let session = ARKitSession()
    private let hands = HandTrackingProvider()

    @State private var udp = UDPClient()
    @State private var ctl = ControlClient()

    var body: some View {
        RealityView { _ in
            // no virtual content needed
        }
        .task {
            guard HandTrackingProvider.isSupported else { return }
            try? await session.run([hands])
        }
        .task {
            // Connect control & data channels when serverHost is set
            guard !app.serverHost.isEmpty else { return }
            udp.connect(host: app.serverHost, port: app.dataPort)
            ctl.connect(host: app.serverHost, port: app.controlPort)
            // say hello & push config
            sendHelloAndConfig()
        }
        .task {
            // Main hand-tracking loop
            for await update in hands.anchorUpdates {
                guard app.streamingEnabled else { continue }
                guard update.event == .added || update.event == .updated else { continue }

                let anchor = update.anchor
                if app.useRightHand && anchor.chirality != .right { continue }
                if !app.useRightHand && anchor.chirality != .left  { continue }

                // Wrist pose in app origin
                var worldFromWrist = anchor.originFromAnchorTransform
                // Apply calibration offset (align app ↔ robot base)
                worldFromWrist = app.originFromApp * worldFromWrist

                // Position
                let p = worldFromWrist.columns.3
                // Rotation → quaternion (w,x,y,z)
                let q = quaternion(from: worldFromWrist)

                // Pinch distance (mm)
                var pinchMM: Double = .nan
                if let skel = anchor.handSkeleton,
                   let thumb = skel.joint(.thumbTip),
                   let index = skel.joint(.indexTip) {
                    let wT = anchor.originFromAnchorTransform * thumb.anchorFromJointTransform
                    let wI = anchor.originFromAnchorTransform * index.anchorFromJointTransform
                    let d = length(SIMD3<Float>(wT.columns.3.x - wI.columns.3.x,
                                               wT.columns.3.y - wI.columns.3.y,
                                               wT.columns.3.z - wI.columns.3.z))
                    pinchMM = Double(d * 1000.0)
                }

                let msg: [String: Any] = [
                    "type": "pose",
                    "seq": Int(Date().timeIntervalSince1970 * 1000) & 0x7fffffff,
                    "t_client": Date().timeIntervalSince1970,
                    "hand": (anchor.chirality == .right ? "right" : "left"),
                    "pos": [Double(p.x), Double(p.y), Double(p.z)],
                    "rot": [Double(q.0), Double(q.1), Double(q.2), Double(q.3)],
                    "pinch_mm": pinchMM
                ]
                if let data = try? JSONSerialization.data(withJSONObject: msg) {
                    udp.send(data)
                }
            }
        }
        .overlay(alignment: .bottom) {
            HStack(spacing: 12) {
                Button(app.streamingEnabled ? "Stop" : "Start") {
                    app.streamingEnabled.toggle()
                    ctl.sendJSON(["type": app.streamingEnabled ? "start" : "stop"])
                }
                Button("Calibrate origin") {
                    Task {
                        if let latest = try? await hands.latestAnchors.first {
                            // Capture inverse of current wrist as origin offset
                            app.originFromApp = latest.originFromAnchorTransform.inverse
                            ctl.sendJSON(["type":"calibrate","mode":"capture_wrist_as_origin"])
                        }
                    }
                }
                Toggle(app.useRightHand ? "Right hand" : "Left hand", isOn: $app.useRightHand)
                    .toggleStyle(.switch)
                    .onChange(of: app.useRightHand) { _ in pushHandSetting() }
                Button("E-STOP") {
                    app.streamingEnabled = false
                    ctl.sendJSON(["type":"estop"])
                }
            }
            .padding()
            .glassBackgroundEffect()
        }
    }

    private func sendHelloAndConfig() {
        ctl.sendJSON(["type":"hello","app":"LeRobotVP","ver":"1.0"])
        pushConfig()
        pushHandSetting()
    }

    private func pushHandSetting() {
        ctl.sendJSON(["type":"configure", "hand": app.useRightHand ? "right" : "left"])
    }

    private func pushConfig() {
        ctl.sendJSON([
            "type":"configure",
            "ee_bounds": app.eeBounds,
            "max_ee_step_m": app.maxEEStepM,
            "smoothing": ["alpha": app.smoothingAlpha],
            "pinch": ["close_mm": app.pinchCloseMM, "open_mm": app.pinchOpenMM],
            "pos_scale": app.posScale
        ])
    }
}

// Quaternion extraction from 4x4 transform (rotation only)
fileprivate func quaternion(from m: simd_float4x4) -> (Double, Double, Double, Double) {
    let r = simd_float3x3(m)
    let q = simd_quatf(r)
    return (Double(q.vector.w), Double(q.vector.x), Double(q.vector.y), Double(q.vector.z))
}
```

## `visionos-app/DiscoveryView.swift`

```swift
import SwiftUI
import Network

struct DiscoveryView: View {
    @EnvironmentObject var app: AppState
    @State private var candidates: [String] = []
    @State private var browser: NWBrowser?

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("LeRobot Control Connection").font(.title2)
            HStack {
                TextField("Server IP (e.g., 192.168.1.50)", text: $app.serverHost)
                    .textFieldStyle(.roundedBorder)
                Button("Connect") { connect() }.disabled(app.serverHost.isEmpty)
            }
            Divider()
            Text("mDNS discovery (optional)").font(.headline)
            List(candidates, id: \.self) { host in
                Button(host) { app.serverHost = host; connect() }
            }.frame(height: 200)

            Divider()
            HStack {
                Text("Control Port: \(app.controlPort)")
                Text("Data Port: \(app.dataPort)")
            }
            Divider()
            Text("Pinch thresholds (mm)").font(.headline)
            HStack {
                Text("Close"); Slider(value: $app.pinchCloseMM, in: 10...80)
                Text("Open");  Slider(value: $app.pinchOpenMM,  in: 10...120)
            }
            Divider()
            Text("Safety").font(.headline)
            HStack {
                Text("pos scale"); Slider(value: $app.posScale, in: 0.5...2.0)
                Text("max step (m)"); Slider(value: $app.maxEEStepM, in: 0.002...0.05)
            }
            HStack {
                Text("smoothing α"); Slider(value: $app.smoothingAlpha, in: 0.0...0.8)
            }
            Spacer()
        }
        .padding()
        .onAppear { startBonjour() }
        .onDisappear { browser?.cancel() }
    }

    private func connect() {
        // StreamerView will observe serverHost and connect both channels
    }

    private func startBonjour() {
        let params = NWParameters.tcp
        let b = NWBrowser(for: .bonjour(type: "_lerobot._tcp", domain: nil), using: params)
        b.browseResultsChangedHandler = { results, _ in
            var found: [String] = []
            for r in results {
                if case .service(let name, let type, let domain, let interface) = r.endpoint {
                    _ = (name, type, domain, interface)
                    if let meta = r.metadata as? NWBrowser.Result.Metadata,
                       case .bonjour(let txt) = meta {
                        if let addrStr = txt["host"]?.first {
                            found.append(String(decoding: addrStr, as: UTF8.self))
                        }
                    }
                }
            }
            candidates = Array(Set(found)).sorted()
        }
        b.start(queue: .main)
        browser = b
    }
}
```

## `visionos-app/Info.plist.snippet.xml`

```xml
<!-- Add these keys to your app's Info.plist -->
<key>NSHandsTrackingUsageDescription</key>
<string>We use hand tracking to control the robot arm.</string>
```

---

# Python teleop + control server

> ✅ Assumes you’ve installed LeRobot already (with SO-101/Feetech support). The teleop is a standard BYOH plugin named `lerobot_teleoperator_vpro`.

## `python-teleop/pyproject.toml`

```toml
[build-system]
requires = ["setuptools>=68", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "lerobot_teleoperator_vpro"
version = "0.1.0"
description = "Vision Pro teleoperation plugin for LeRobot (pose+pinch over UDP)"
authors = [{ name = "You" }]
requires-python = ">=3.9"
dependencies = ["numpy>=1.24"]

[tool.setuptools.packages.find]
where = ["."]
include = ["lerobot_teleoperator_vpro*"]
```

## `python-teleop/lerobot_teleoperator_vpro/__init__.py`

```python
__all__ = ["config_vpro", "vpro"]
```

## `python-teleop/lerobot_teleoperator_vpro/config_vpro.py`

```python
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
```

## `python-teleop/lerobot_teleoperator_vpro/vpro.py`

```python
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
```

## `python-teleop/control_server.py`

```python
"""
TCP control server (default :8766) to configure the teleop at runtime.
Messages are newline-delimited JSON:
- {"type":"hello", "app":"LeRobotVP", "ver":"1.0"}
- {"type":"configure", "hand":"right", "ee_bounds":{...}, "max_ee_step_m":0.01,
   "smoothing":{"alpha":0.2}, "pinch":{"close_mm":25,"open_mm":55}, "pos_scale":1.0}
- {"type":"calibrate", "mode":"capture_wrist_as_origin"}
- {"type":"start"} | {"type":"stop"} | {"type":"estop"}
"""
from __future__ import annotations
import asyncio, json
from lerobot_teleoperator_vpro.config_vpro import VProTeleopConfig
from lerobot_teleoperator_vpro.vpro import VProTeleop

teleop = VProTeleop(VProTeleopConfig())
teleop.connect()

class ControlProto(asyncio.Protocol):
    def connection_made(self, transport):
        self.transport = transport

    def data_received(self, data: bytes):
        for line in data.splitlines():
            try:
                msg = json.loads(line.decode("utf-8"))
            except Exception:
                continue
            handle(msg, self.transport)

def handle(msg: dict, tx):
    t = msg.get("type")
    if t == "hello":
        tx.write(b'{"type":"caps","ok":true}\n')

    elif t == "configure":
        hand = msg.get("hand")
        if hand: teleop.set_hand(hand)
        pinch = msg.get("pinch", {})
        if "close_mm" in pinch: teleop.config.pinch_close_mm = float(pinch["close_mm"])
        if "open_mm"  in pinch: teleop.config.pinch_open_mm  = float(pinch["open_mm"])
        if "pos_scale" in msg:  teleop.config.pos_scale      = float(msg["pos_scale"])
        teleop.set_safety(
            ee_bounds=msg.get("ee_bounds"),
            max_ee_step_m=msg.get("max_ee_step_m"),
            smoothing=msg.get("smoothing")
        )
        tx.write(b'{"type":"ack","of":"configure"}\n')

    elif t == "calibrate":
        ok = teleop.capture_origin_from_current_wrist()
        tx.write(json.dumps({"type":"ack","of":"calibrate","ok":ok}).encode()+b"\n")

    elif t == "start":
        teleop.enable(True);  tx.write(b'{"type":"ack","of":"start"}\n')
    elif t == "stop":
        teleop.enable(False); tx.write(b'{"type":"ack","of":"stop"}\n')
    elif t == "estop":
        teleop.estop();       tx.write(b'{"type":"ack","of":"estop"}\n')

async def main():
    loop = asyncio.get_running_loop()
    server = await loop.create_server(ControlProto, "0.0.0.0", 8766)
    print("Control listening on :8766")
    await server.serve_forever()

if __name__ == "__main__":
    asyncio.run(main())
```

## (Optional) `python-teleop/mdns_advertise.py`

```python
"""
Advertise the control endpoint via Bonjour/mDNS so the visionOS app can auto-discover.
Publishes a TXT record with "host=<ip_or_dns>".
"""
from zeroconf import Zeroconf, ServiceInfo
import socket, time

def get_host_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    finally:
        s.close()
    return ip

if __name__ == "__main__":
    zc = Zeroconf()
    host = get_host_ip()
    service = ServiceInfo(
        type_="_lerobot._tcp.local.",
        name=f"LeRobot Control._lerobot._tcp.local.",
        addresses=[socket.inet_aton(host)],
        port=8766,
        properties={"host": host.encode("utf-8")},
    )
    zc.register_service(service)
    print(f"mDNS advertised at {host}:8766 (type _lerobot._tcp)")
    try:
        while True: time.sleep(3600)
    except KeyboardInterrupt:
        zc.unregister_service(service)
        zc.close()
```

## `python-teleop/run_with_lerobot_cli.sh`

```bash
#!/usr/bin/env bash
set -euo pipefail
# Example launcher using LeRobot's CLI (adjust to your environment).
# 1) Start control server (separate terminal):
#    python control_server.py
#
# 2) Then run LeRobot teleop pipeline pointing at our plugin 'vpro':
#    lerobot-teleoperate \
#      --robot.type=so101_follower \
#      --teleop.type=vpro \
#      --teleop.udp_port=8765 \
#      --processor.ee_to_joints=true \
#      --safety.ee_bounds='x:[-0.25,0.25],y:[0.00,0.40],z:[0.00,0.35]' \
#      --safety.max_ee_step_m=0.01
#
# If your CLI differs, adapt accordingly.
echo "See comments in this script for usage. Edit flags to match your setup."
```

---

# README (quick start)

## 0) Prereqs

* **visionOS**: Xcode 16+, visionOS SDK, Apple Vision Pro or Simulator (hand tracking requires device).
* **Python**: 3.9+, `pip`, LeRobot installed with SO-101 support (Feetech).

## 1) Python side (teleop + control)

```bash
cd python-teleop
python -m venv .venv && source .venv/bin/activate
pip install -e .
pip install zeroconf  # optional, for Bonjour discovery

# Terminal A: start control server
python control_server.py
# (Optional) Terminal B: advertise via Bonjour
python mdns_advertise.py
```

Run your LeRobot teleop with our plugin:

```bash
# Adjust ports/flags to your setup — this assumes LeRobot exposes a CLI like this:
lerobot-teleoperate \
  --robot.type=so101_follower \
  --teleop.type=vpro \
  --teleop.udp_port=8765 \
  --processor.ee_to_joints=true \
  --safety.ee_bounds='x:[-0.25,0.25],y:[0.00,0.40],z:[0.00,0.35]' \
  --safety.max_ee_step_m=0.01
```

> If your environment uses a different entrypoint, just ensure your pipeline pulls **teleop `vpro`** as the action source. The plugin exposes actions `{ee.x,ee.y,ee.z,ee.wx,ee.wy,ee.wz,gripper_pos}`.

## 2) visionOS app

1. Open **Xcode** → “Create a new App (visionOS)” named **LeRobotVP**.
2. Add the Swift files from `visionos-app/` into the project.
3. In your **Info.plist**, add:

   * `NSHandsTrackingUsageDescription` = “We use hand tracking to control the robot arm.”
4. Build & run on your Vision Pro device.

**Connect flow:**

* In the **Control** window (visionOS), enter your PC’s IP (or pick from the Bonjour list).
* Tap **Connect**, then **Start** in the immersive UI to begin streaming.
* Use **Calibrate origin** with your wrist where you want `(0,0,0)` to align.
* Switch **Left/Right hand**, adjust **pinch thresholds**, **pos scale**, **max step**, **smoothing**.
* Hit **E-STOP** anytime (teleop is disabled immediately on the Python side).

---

## Notes & extension points

* **Safety & smoothing**: We mirror configs over TCP; apply them in your LeRobot processor chain (EE bounds, max step, exponential smoothing). The teleop class stores them; wire those into the exact processors you use.
* **Coordinate frames**: We apply a simple **position offset**. If you want full 6-DoF base alignment, extend `_apply_origin` to pre-rotate the EE by the captured origin quaternion.
* **Transport**: UDP is used for data (low-latency). TCP (newline-delimited JSON) for control plane.
* **Discovery**: Optional Bonjour (mDNS) publisher/consumer included. If you skip it, just type the IP.
* **Rates**: 30–60 Hz is plenty. The app emits a timestamp and seq if you want client-side interpolation.
