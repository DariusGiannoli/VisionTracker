# DabriusStreamer

DabriusStreamer is a teleoperation system for the LeRobot SO-101 arm using the Apple Vision Pro. It turns the headset into a WebSocket server that streams Head-Relative hand tracking data to control the robot in real-time.

**Authors:** Darius Giannoli & Gabriel Taieb

---

## 🛠 Hardware Requirements

* Apple Vision Pro (visionOS)
* LeRobot SO-101 (Feetech STS3215 motors)
* Mac/PC with Python environment

---

## 🚀 Installation & Setup

### 1. Vision Pro App (Server)

The visionOS app acts as the WebSocket server.

1. Open the project in Xcode and run it on your Vision Pro.
2. Grant Hand Tracking permissions.
3. Note the **IP Address** and **Port** (default: `8211`) displayed on the dashboard.
4. Configure your settings (see below) and press **Start**.

### 2. Python Client (Robot Control)

The Python script connects to the headset to drive the motors.

1. Install dependencies: `lerobot`, `numpy`, `websockets`.
2. Connect the robot via USB.
3. Run the CLI:
```bash
python -m dabrius.cli \
  --ws-host <VISION_PRO_IP> \
  --ws-port 8211 \
  --serial-port /dev/tty.usbmodem<YOUR_PORT>
```

---

## 🧠 Core Concept: Head-Relative Tracking

The system uses a **Head-Relative coordinate system**. The robot mimics the position of your **Hand relative to your Face** (Headset).

* **Benefit:** You can walk around the room or sit down; as long as your hand stays in the same position relative to your eyes, the robot remains stable.

---

## 🎮 Controls & Mapping

| Hand Motion (Relative to Face) | Robot Action | Motor ID |
|--------------------------------|--------------|----------|
| Left / Right | Shoulder Pan | ID 1 |
| Up / Down | Shoulder Lift | ID 2 |
| Forward / Back | Elbow Flex (Reach) | ID 3 |
| Wrist Pitch | Wrist Flex | ID 4 |
| Wrist Roll | Wrist Roll | ID 5 |
| Pinch (Thumb+Index) | Gripper Open/Close | ID 6 |

---

## 📱 App Interface & Features

### Configuration (Pre-Stream)

* **FPS Selection:** Choose 20, 30, or 60 FPS. Higher FPS is smoother but consumes more battery.
* **Active Motors:** Toggle specific motors ON/OFF (e.g., disable the Gripper or Elbow) before starting the stream.

### Stealth Mode (During Stream)

To maximize immersion, the UI disappears when you press **Start**. To bring the controls back (and Stop the robot):

* **Double-Tap** (Double Pinch) anywhere.
* **Long-Press** (Pinch and hold ~0.5s).
* **Drag** (Pinch and move slightly).

---

## ⚙️ Safety

* **Speed Gating:** Prevents sudden jumps if the tracking glitches.
* **Smoothing:** Applies an alpha smoothing factor (0.8) to movement.
* **Calibration:** Loads motor calibration from `~/.cache/huggingface/lerobot/...`. If missing, falls back to safe default ranges.

---

**Based on code version:** 0.1.0