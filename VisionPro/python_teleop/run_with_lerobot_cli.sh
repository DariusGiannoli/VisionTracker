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
