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
