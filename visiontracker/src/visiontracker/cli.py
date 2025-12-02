import argparse
from .teleop import run_teleop

def main():
    p = argparse.ArgumentParser("visiontracker-teleop")
    p.add_argument("--ws-host", required=True, help="Vision Pro IP or hostname")
    p.add_argument("--ws-port", type=int, default=8211)
    p.add_argument("--ws-path", default="/stream")
    p.add_argument("--serial-port", required=True)
    args = p.parse_args()
    run_teleop(ws_host=args.ws_host, serial_port=args.serial_port, ws_port=args.ws_port, ws_path=args.ws_path)
