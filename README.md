# 基于视觉感知与路径规划的室内电力巡检机器人

本仓库保存当前实际使用的室内电力巡检机器人代码与资源，覆盖 ROCK5B 板端建图、导航、摄像头预览、F103 底盘桥接、六轴机械臂控制、视觉识别模型以及 8765 云平台代码。

## 目录结构

- `board/rt_direct_tools/`：板端运行脚本，包含 F103 bridge、键盘遥控、机械臂控制、摄像头预览和开机机械臂归位脚本。
- `board/ros_ws_src/`：板端 ROS2 工作空间源码包，包含建图、导航、底盘控制、雷达和 IMU 相关包。
- `board/systemd/`：板端 systemd 服务配置，包括 F103 底盘桥接和机械臂开机归位服务。
- `board/udev/`：板端 udev 规则，包括 IMU 设备别名规则。
- `cloud/8765/`：服务器 8765 云平台代码，用于数据展示、图像识别、建图/导航页面和板端数据上传接口。
- `vision/`：板端视觉识别脚本和表计读取相关代码。
- `assets/maps/`：当前使用和保留的建图/导航地图资源。
- `assets/yolo_models/`：当前视觉识别使用的 YOLO 模型和类别文件，大模型通过 Git LFS 管理。
- `arm_linux/`、`x86_linux/`、`tools/`、`code/`：底层 RT/HyperSDK/F103 相关源码、工具和历史保留代码。

## 板端 ROS2 运行约定

板端主链统一使用：

```bash
export ROS_DOMAIN_ID=17
export ROS_LOCALHOST_ONLY=1
source /opt/ros/foxy/setup.bash
source /home/rock/Desktop/rock_ws/ros_ws/install/setup.bash
```

F103 底盘桥接由 `f103-usb-chassis.service` 常驻运行，主要接口如下：

- `/cmd_vel`：导航和键盘控制底盘。
- `/cmd_vel_cmd`：建图避障等辅助底盘速度入口。
- `/odom`：F103 bridge 发布底盘里程计。
- `/f103_state`：F103 bridge 状态输出。
- `/arm_cmd`：六轴机械臂原始舵机协议入口。

## 常用启动命令

### 自动建图

```bash
export ROS_DOMAIN_ID=17
export ROS_LOCALHOST_ONLY=1
source /opt/ros/foxy/setup.bash
source /home/rock/Desktop/rock_ws/ros_ws/install/setup.bash

ros2 launch rt_robot_nav2 rt_robot_nav2_complete.launch.py \
  use_slam:=true \
  use_nav:=false \
  open_rviz:=true \
  use_auto_mapping:=true
```

### 使用 666 地图导航

```bash
export ROS_DOMAIN_ID=17
export ROS_LOCALHOST_ONLY=1
source /opt/ros/foxy/setup.bash
source /home/rock/Desktop/rock_ws/ros_ws/install/setup.bash

ros2 launch rt_robot_nav2 rt_robot_nav2_complete.launch.py \
  use_slam:=false \
  use_nav:=true \
  use_chassis_controller:=true \
  use_odom_fusion:=false \
  map_file:=/home/rock/Desktop/rock_ws/ros_ws/install/rt_robot_nav2/share/rt_robot_nav2/map/666.yaml \
  open_rviz:=true
```

### 摄像头预览

默认使用 `/dev/video0`、MJPG、`1280x720@15fps`，并旋转 180 度显示：

```bash
cd /home/rock/Desktop/rt_direct_tools
bash ./start_camera_preview_1280x720.sh
```

高分辨率采集并缩放显示示例：

```bash
cd /home/rock/Desktop/rt_direct_tools
bash ./start_camera_preview_1280x720.sh \
  --format mjpg \
  --width 2048 \
  --height 1536 \
  --fps 15 \
  --display-width 1280 \
  --display-height 720
```

### 六轴机械臂回初始姿态

默认初始姿态为：

```text
#000=2300, #001=1450, #002=1500, #003=2200, #004=1500, #005=1700
```

手动发送 home：

```bash
export ROS_DOMAIN_ID=17
export ROS_LOCALHOST_ONLY=1
source /opt/ros/foxy/setup.bash
source /home/rock/Desktop/rock_ws/ros_ws/install/setup.bash

cd /home/rock/Desktop/rt_direct_tools
python3 ros2_tools/arm_cmd_cli.py home --duration 2000
```

## 云平台

8765 云平台代码位于 `cloud/8765/`，服务器部署路径为 `/root/car/yun`。板端上传脚本和云平台接口代码已随当前版本同步到仓库。

启动云平台示例：

```bash
cd /root/car/yun
bash ./start_cloud_platform.sh
```

访问地址：

```text
http://115.159.33.216:8765
```

## Git LFS

仓库中的模型、镜像和大二进制文件使用 Git LFS 管理。首次拉取后建议执行：

```bash
git lfs install
git lfs pull
```

如果缺少 `.onnx`、`.pt`、`.rknn` 等模型文件，优先检查 Git LFS 是否已正确拉取。

## 注意事项

- 本仓库只保存当前代码与必要资源，不包含运行日志、虚拟环境、缓存和历史识别图片。
- 板端建图、导航、底盘、机械臂 ROS2 主链默认都在 Domain17 下运行。
- 8765 是当前稳定云平台；8764 展示测试平台不在本次同步范围内。
- 摄像头 `1280x720@30fps` 当前硬件能力不支持，默认使用 `1280x720@15fps`。
