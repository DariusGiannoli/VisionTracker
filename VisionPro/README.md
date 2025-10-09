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
