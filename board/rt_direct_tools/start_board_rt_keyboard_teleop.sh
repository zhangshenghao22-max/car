#!/usr/bin/env bash
set -e
source /opt/ros/foxy/setup.bash
if [ -f /home/rock/Desktop/rock_ws/ros_ws/install/setup.bash ]; then
  source /home/rock/Desktop/rock_ws/ros_ws/install/setup.bash
fi
echo '[check] /cmd_vel endpoints:'
ros2 topic info /cmd_vel -v || true
echo '[start] keyboard teleop -> /cmd_vel'
exec python3 /home/rock/Desktop/rt_direct_tools/board_keyboard_teleop.py \
  --topic /cmd_vel \
  --linear-step 0.03 \
  --angular-step 0.05 \
  --max-linear 0.20 \
  --max-angular 0.30
