#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/foxy/setup.bash
source /home/rock/Desktop/rock_ws/ros_ws/install/setup.bash
set -u

export ROS_DOMAIN_ID=17
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rock/Desktop/rock_ws/fastdds_no_shm.xml
export PYTHONUNBUFFERED=1

log_dir="/home/rock/Desktop/rt_direct_tools/logs/nav_serialization_test/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$log_dir"
shadow_pid=""
nav_pid=""
cmd_pid=""

publish_zero() {
    timeout 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
        >/dev/null 2>&1 || true
    timeout 3 ros2 topic pub --once /cmd_vel_isolated geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
        >/dev/null 2>&1 || true
}

cleanup() {
    set +e
    publish_zero
    if [[ -n "$cmd_pid" ]]; then
        kill "$cmd_pid" 2>/dev/null || true
    fi
    if [[ -n "$nav_pid" ]]; then
        kill -INT "$nav_pid" 2>/dev/null || true
        for _ in $(seq 1 30); do
            kill -0 "$nav_pid" 2>/dev/null || break
            sleep 0.2
        done
        kill -TERM "$nav_pid" 2>/dev/null || true
    fi
    timeout 15 python3 /home/rock/Desktop/car_cloud_uploader/cleanup_navigation_processes.py \
        >/dev/null 2>&1 || true
    if [[ -n "$shadow_pid" ]]; then
        kill -INT "$shadow_pid" 2>/dev/null || true
        for _ in $(seq 1 15); do
            kill -0 "$shadow_pid" 2>/dev/null || break
            sleep 0.2
        done
        kill -TERM "$shadow_pid" 2>/dev/null || true
    fi
    sudo systemctl start f103-usb-chassis.service
    sudo systemctl start car-cloud-uploader.service
}
trap cleanup EXIT
trap 'exit 130' HUP INT TERM

publish_zero
sudo systemctl stop car-cloud-uploader.service
sudo systemctl stop f103-usb-chassis.service

python3 /home/rock/Desktop/rt_direct_tools/ros2_tools/f103_usb_ros2_bridge.py \
    --node-name f103_usb_ros2_bridge_isolated \
    --port /dev/serial/by-path/platform-fc8c0000.usb-usb-0:1:1.0-port0 \
    --baudrate 115200 \
    --cmd-topic /cmd_vel_isolated \
    --extra-cmd-topic /cmd_vel_cmd_isolated \
    --odom-topic /odom_raw \
    --state-topic /f103_state_shadow \
    --motion-fault-topic /f103_motion_fault_shadow \
    --max-angular-mrad-s 140 \
    --max-moving-angular-mrad-s 70 \
    --cmd-vx-sign 1 \
    --cmd-wz-sign -1 \
    --cmd-angular-scale 0.69 \
    --cmd-wz-trim-per-vx -0.10 \
    --cmd-output-mode dcr \
    --cmd-wheel-signs 1,1,1,1 \
    --inplace-wheel-min-mm-s 30,30,30,30 \
    --dcr-change-threshold-mm-s 10 \
    --dcr-max-update-hz 2 \
    --startup-feedback-mm-s 2 \
    --startup-feedback-frames 2 \
    --odom-vx-sign 1 \
    --odom-linear-scale 1.71 \
    --odom-angular-scale 2.356484 \
    --odom-wz-sign -1 \
    --odom-wheel-signs 1,1,-1,-1 \
    >"$log_dir/shadow_bridge.log" 2>&1 &
shadow_pid="$!"
sleep 3

cd /home/rock/Desktop/rock_ws/ros_ws
ros2 launch rt_robot_nav2 rt_robot_nav2_complete.launch.py \
    use_slam:=false \
    use_nav:=true \
    use_chassis_controller:=false \
    use_ekf:=true \
    use_auto_initialpose:=false \
    use_maze_turn_manager:=true \
    open_rviz:=false \
    urdf_file:=c26_double_layer_robot.urdf \
    map_file:=/home/rock/Desktop/car_cloud_uploader/lidar_maps/ceshi7.yaml \
    >"$log_dir/navigation.log" 2>&1 &
nav_pid="$!"

for _ in $(seq 1 45); do
    if ros2 action list 2>/dev/null | grep -qx /navigate_to_pose; then
        break
    fi
    sleep 1
done

ros2 action list >"$log_dir/actions.log" 2>&1
{
    ros2 param get /controller_server FollowPath.use_rotate_to_heading
    ros2 param get /controller_server FollowPath.lookahead_dist
    ros2 param get /controller_server FollowPath.regulated_linear_scaling_min_speed
} >"$log_dir/controller_params.log" 2>&1
timeout 18 stdbuf -oL ros2 topic echo /cmd_vel >"$log_dir/cmd_vel.log" 2>&1 &
cmd_pid="$!"

ros2 topic pub --times 3 -r 2 /initialpose \
    geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: -2.054, y: 0.923, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.944779323, w: 0.327707233}}}}" \
    >"$log_dir/initialpose.log" 2>&1
sleep 5

ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: -2.271, y: 0.779, z: 0.0}, orientation: {z: -0.944779323, w: 0.327707233}}}" \
    >"$log_dir/goal_1.log" 2>&1
sleep 1
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: -2.330, y: 0.740, z: 0.0}, orientation: {z: -0.944779323, w: 0.327707233}}}" \
    >"$log_dir/goal_2.log" 2>&1
sleep 1
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: -2.380, y: 0.700, z: 0.0}, orientation: {z: -0.944779323, w: 0.327707233}}}" \
    >"$log_dir/goal_3.log" 2>&1
sleep 3

state_json='{"target_wheels_mm_s":[60,60,60,60],"last_wheel_mm_s":[300,300,-300,-300],"odom_wheel_signs":[1,1,-1,-1],"last_body_twist":{"wz":0.0},"last_imu_wz":0.0}'
ros2 topic info /f103_state --verbose >"$log_dir/f103_state_topology.log" 2>&1 || true
timeout 3 ros2 topic pub -r 10 /f103_state std_msgs/msg/String \
    "{data: '$state_json'}" >"$log_dir/fault_injection.log" 2>&1 || true
sleep 3

pgrep -af 'bt_navigator|maze_turn_manager.py|controller_server|planner_server' \
    >"$log_dir/processes_after.log" || true
timeout 2 ros2 topic echo /f103_state_shadow >"$log_dir/shadow_state.log" 2>&1 || true
wait "$cmd_pid" 2>/dev/null || true
cmd_pid=""

if grep -q 'send_goal failed' "$log_dir/navigation.log"; then
    echo "send_goal failure remained" >&2
    exit 20
fi
if ! grep -q 'navigation safety stop: wheel_.*_overspeed' "$log_dir/navigation.log"; then
    echo "safety stop was not observed" >&2
    exit 21
fi
if ! grep -q 'managed route:' "$log_dir/navigation.log" || \
   ! grep -q 'Received a goal, begin computing control effort' "$log_dir/navigation.log"; then
    echo "manager did not send the planned path directly to FollowPath" >&2
    exit 22
fi
if ! grep -Eq 'x: -?0\.[0-9]*[1-9]|z: -?0\.[0-9]*[1-9]' "$log_dir/cmd_vel.log"; then
    echo "controller accepted FollowPath but emitted no non-zero cmd_vel" >&2
    exit 26
fi
if pgrep -f '[b]t_navigator' >/dev/null; then
    echo "bt_navigator unexpectedly started in managed mode" >&2
    exit 23
fi
for process in '[m]aze_turn_manager.py' '[c]ontroller_server' '[p]lanner_server'; do
    if ! pgrep -f "$process" >/dev/null; then
        echo "required navigation process missing: $process" >&2
        exit 24
    fi
done
if ! grep -qx '/follow_path' "$log_dir/actions.log" || \
   ! grep -qx '/navigate_to_pose' "$log_dir/actions.log" || \
   grep -qx '/nav2_navigate_to_pose' "$log_dir/actions.log"; then
    echo "managed action topology is incorrect" >&2
    exit 25
fi
if ! grep -q 'Boolean value is: False' "$log_dir/controller_params.log"; then
    echo "RPP rotate-to-heading was not disabled in managed mode" >&2
    exit 27
fi

echo "$log_dir"
