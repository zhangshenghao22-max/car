# 室内电力巡检机器人 · 车载端工程（ROCK 5B / RK3588）

> 基于 RK3588 + ROS 2 Foxy 的室内配电柜巡检机器人完整车载工程：激光 SLAM 建图、Nav2 自主导航、
> 六轴机械臂 + 相机对配电柜做 AprilTag 对位与表计/状态灯/开关识别，并把数据实时上报云端平台。

---

## 一、项目简介

本项目是一台**室内电力巡检机器人**的车载（板端）软件集合，运行在 ROCK 5B（RK3588）上，
完成从「建图」到「巡检」的完整闭环：

1. **建图**：激光雷达 + gmapping 建图，支持前沿（frontier）自主探索与窄道自动建图；
2. **定位**：F103 底盘里程计 + IMU 经 EKF 融合，AMCL 在已保存地图上定位；
3. **导航**：Nav2（NavFn 全局规划 + DWB 局部控制）执行单点、巡航、巡检路线；
4. **巡检**：到达配电柜点后用 AprilTag 对位，机械臂伸至柜面，相机采集并识别电压/电流表、
   温控表、状态灯、选择开关、七段数码管，以及柜内烟雾/氢气/CO/温湿度/红外测温/声音；
5. **上云**：把地图、状态、识别结果、巡检报告、音频上传云端，同时接收云端的导航/巡检/遥控指令；
6. **可视**：本地 MJPEG 视频流、RViz 可视化、云端 Web 页面远程查看。

配套的云端平台（Flask，端口 8765）是独立工程，不包含在本仓库内。

---

## 二、硬件与运行环境

| 项目 | 配置 |
|---|---|
| 主控 | ROCK 5B（RK3588，8 核 ARM，NPU 6 TOPS） |
| 操作系统 | Ubuntu 20.04 LTS (Focal) |
| 内核 | 5.10.66-27-rockchip-gea60d388902d |
| ROS | ROS 2 **Foxy**，`rmw_fastrtps_cpp`，`ROS_DOMAIN_ID=17`，`ROS_LOCALHOST_ONLY=1`，`fastdds_no_shm.xml` |
| 底盘 | STM32F103 + 麦克纳姆轮，USB CDC 串口 **115200**（`/dev/serial/by-path/platform-fc8c0000.usb-usb-0:1:1.0-port0`） |
| 实时底盘（备用） | RT-Thread 侧，`/dev/ttyUSB0` @1.5 Mbaud，micro-ROS UDP `10.10.10.30:8888`（未启用） |
| 雷达 | LSLIDAR 系列（`lslidar_driver`）/ RPLIDAR A1（`rplidar_ros`） |
| IMU | 达妙 `dm_imu` / 雅宝 IMU（udev 规则 `71-yahboom-imu.rules`） |
| 机械臂 | 六轴，F103 直连或总线舵机（二选一） |
| 相机 | USB/MIPI 相机，支持高分辨率抓拍与实时流 |
| 远程运维 | NoMachine 远程桌面、SSH |

---

## 三、系统架构

```
                    ┌──────────────────────────── 云端平台 (Flask :8765) ───────────────────────────┐
                    │  状态看板 / 地图管理 / 导航指令 / 巡检报告 / 视频流 / 遥控 / 识别历史          │
                    └───────────────▲───────────────────────────────────────────▲──────────────────┘
                                    │ HTTPS/JSON 上报 + 指令轮询                 │ MJPEG
        ┌───────────────────────────┴───────────────┐                ┌──────────┴───────────┐
        │ board_linux_uploader.py（上云/指令/音频）  │                │ vision_stream_gateway │
        │ board_nav_command.py（巡航/巡检执行器）    │                └──────────▲───────────┘
        └───────────────▲───────────────────────────┘                           │
                        │ ROS2 话题/动作                                        │
   ┌────────────────────┴───────────────────────────────────────────────────────┴──────────────┐
   │                                   ROS 2 Foxy (DDS)                                        │
   │  /cmd_vel ──► f103_usb_ros2_bridge ──► 串口 ──► F103 ──► 电机                              │
   │  /odom_raw ◄──┘        ▲ /imu/data                                                         │
   │      └──── EKF(robot_localization) ──► /odom ──► AMCL ──► Nav2(NavFn+DWB) ──► /cmd_vel      │
   │  /scan ◄── 激光雷达                                                                        │
   │  相机 ──► recognition_worker (RKNN) ──► 结构化识别结果 ──► 巡检流程 / 云端                  │
   │  巡检：cabinet_apriltag_inspection ──► 机械臂对位 + 拍照 + 识别 + 报告                     │
   └────────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 四、目录结构

```
board_current/
├─ README.md                 本文档
├─ MANIFEST/                 板上文件哈希清单与本地工程比对结果
│   ├─ board_inventory.tsv
│   └─ board_vs_local.csv
├─ etc/
│   ├─ systemd/system/       自定义 systemd 单元（业务自启）
│   └─ udev/rules.d/         雷达 / IMU 设备规则
└─ home/rock/
    ├─ *.yaml                现场调参用的 Nav2 / AMCL / EKF 参数
    └─ Desktop/
        ├─ car_cloud_uploader/   上云、导航执行、柜体巡检、视觉识别（Python）
        ├─ rt_direct_tools/      F103 底盘桥、机械臂遥控、标定与启停脚本
        └─ rock_ws/ros_ws/src/   ROS 2 源码包
```

| 目录 | 内容 |
|---|---|
| `home/rock/Desktop/car_cloud_uploader/` | 业务主程序、巡检流程、视觉识别、视频流、云端协议 |
| `home/rock/Desktop/rt_direct_tools/` | 底盘串口桥、键盘遥控、机械臂、里程计标定、启停脚本 |
| `home/rock/Desktop/rock_ws/ros_ws/src/` | `rt_robot_nav2`、`chassis_controller`、`slam_gmapping`、`lslidar_driver` 等 13 个包 |
| `etc/systemd/system/` | 业务自启单元（底盘、云上传、识别 worker、视频流、机械臂归位等） |
| `etc/udev/rules.d/` | `70-rplidar-a1.rules`、`71-yahboom-imu.rules` |
| `MANIFEST/` | 与现场镜像的逐文件 SHA-256 清单及本地工程比对结果 |

---

## 五、功能模块详解

### 5.1 底盘运动控制（F103 USB 直链）

- **入口**：`rt_direct_tools/ros2_tools/f103_usb_ros2_bridge.py`（约 70 KB，当前实际使用的链路）
- **作用**：订阅 `/cmd_vel`、`/cmd_vel_cmd`、`/cmd_vel_mapping`，转换为 F103 的 DCR 串口协议下发；
  回读编码器/IMU 数据，发布 `/odom_raw`、`/f103_state`。
- **关键参数**（系统单元里固化，均为实车标定值）：
  `--cmd-vx-sign 1 --cmd-wz-sign -1 --cmd-angular-scale 1.0 --cmd-output-mode dcr`、
  `--odom-linear-scale 1.71 --odom-angular-scale 2.356484 --odom-wz-sign -1`、启动助推/原地转向确认等。
- **启动**：`f103-usb-chassis.service`（`Restart=always`，日志写 `rt_direct_tools/logs/f103_usb_bridge_systemd.log`）
- **辅助**：`f103_usb_twist_panel.py`（不经 ROS 直接调速）、`check_f103_usb_chassis.sh`、`start/stop_f103_usb_chassis.sh`

> 另一条 RT-Thread + micro-ROS 链路（`rt_autostart/`、`rt-microros-agent.service`）已在设备上保留但**未 enable**，
> 属于演进中的备用方案。

### 5.2 里程计与定位

- **EKF**：`robot_localization`，参数 `rt_robot_nav2/config/f103_ekf.yaml`，把 `/odom_raw` + `/imu/data` 融合为 `/odom`。
- **IMU**：`dm_imu` 驱动 + `imu_filter_madgwick` 姿态解算，`imu_z_calibrator.py` 做零偏标定。
- **TF**：`chassis_controller` 中 `odometry_to_tf.cpp`、`imu_to_tf.cpp` 维护 `odom→base_link→imu`。
- **AMCL**：在地图上定位，初始位姿可由地图 yaml 的 `initial_pose` 或 `auto_initialpose.py` 自动给定。

### 5.3 建图与自主探索

- **SLAM**：`slam_gmapping`（gmapping 分支）+ `openslam_gmapping`，激光 `/scan` 输入。
- **自主建图**：`chassis_controller/src/auto_mapping_driver.cpp`（窄道自动建图）+ `frontier_explorer.cpp`（前沿探索）。
- **保存地图**：`save_map.launch.py`、`ros2_tools/save_ros2_map.sh`、`save_ros2_map_fallback.py`、
  `stamp_map_initial_pose.py`（把初始位姿写回地图 yaml）。
- **地图库**：`car_cloud_uploader/lidar_maps/`（现场 22 份测试/正式地图，含 `ceshi*`、`cloud_map_*` 等）。

### 5.4 导航（Nav2）

- **总启动**：`rt_robot_nav2/launch/rt_robot_nav2_complete.launch.py`（带文件锁，防重复启动）
- **规划**：`nav2_navfn_planner/NavfnPlanner`（`expected_planner_frequency: 2.0`）
- **控制**：`dwb_core::DWBLocalPlanner`，`SimpleProgressChecker` / `SimpleGoalChecker`
- **参数**：`param/rt_robot_params.yaml`（约 14 KB）、`config/velocity_limits.yaml`
- **辅助脚本**：

| 脚本 | 作用 |
|---|---|
| `auto_initialpose.py` | 按地图默认位姿自动初始化 AMCL |
| `verify_localization.py` | 校验定位是否收敛 |
| `nav_motion_stabilizer.py` / `_core.py` | 速度平滑、防抖、起步/停止整形 |
| `costmap_scan_range_filter.py` | 雷达无效距离过滤，避免代价地图误判 |
| `save_initialpose_to_map.py` | 保存初始位姿到地图元数据 |

- **启动参数**：`use_slam / use_nav / use_ekf / use_auto_mapping / use_auto_initialpose / map_file / urdf_file`
  （现场常用 `use_slam:=false use_nav:=true`，即已知地图导航）。

### 5.5 机械臂

- `rt_direct_tools/ros2_tools/arm_cmd_cli.py`：发布 `/arm_cmd` 控制六轴；
- `board_keyboard_arm_teleop.py` / `arm_manual_control.py`：键盘手动遥控；
- `bus_servo_ros2_bridge.py` + `bus_servo_cmd_cli.py`：总线舵机链路；
- `start_f103_arm_startup_home.sh`：开机回六轴 home 位（`f103-arm-startup-home.service`）；
- `car_cloud_uploader/ros2_tools/arm_kinematics.py`：巡检用的机械臂运动学。

### 5.6 柜体巡检（核心业务）

| 文件 | 作用 |
|---|---|
| `ros2_tools/cabinet_apriltag_inspection.py`（约 214 KB） | 巡检主流程：导航到柜点 → AprilTag 对位 → 机械臂伸出 → 拍照 → 识别 → 结构化结果 |
| `ros2_tools/cabinet_inspection_runner.py` | 单次巡检执行器（子进程/阶段调度） |
| `ros2_tools/cabinet_nav_takeover.py` | 巡检期间接管导航（暂停/恢复/重定位） |
| `ros2_tools/cabinet_visual_core.py` / `cabinet_visual_mission.py` | 视觉任务编排与结构化输出 |
| `ros2_tools/cabinet_camera_overlay.py` | 相机去畸变 + AprilTag + 坐标叠加显示（`cabinet-camera-overlay.service`） |
| `ros2_tools/arm_apriltag_s0_search.py` | 机械臂多姿态搜索 AprilTag |
| `cabinet_inspection_job.py` | 异步巡检任务管理（启动/查询/停止，状态落盘 `runtime/`） |
| `ros2_tools/seven_segment_reader.py` | 七段数码管读数（温控表/显示面板） |

识别内容：电压表、电流表、温控表（PV/SV）、状态灯、选择开关、七段数码管，
以及柜内烟雾/氢气/CO 浓度、温湿度、红外测温、声音。

### 5.7 视觉识别（RKNN）

- `ros2_tools/recognition_worker.py`：常驻进程，加载 RKNN/ONNX 模型，通过 Unix socket 提供识别服务
  （`car-recognition-worker.service`）；
- `ros2_tools/camera_yolo_detect.py`：YOLO 目标检测（表计、状态灯、开关、数码管）；
- `ros2_tools/camera_highres_capture.py`：高分辨率抓拍；
- 模型：`yolo_models/*.onnx`（`digits_best`、`lightlabel_best`、`meter_digit_unified_best`、
  `power_cabinet_best`、`switch_yolo_best`）及对应 `*_classes.txt`。

### 5.8 视频流

- `vision_stream_gateway.py`：HTTP MJPEG 推流服务（多客户端、暂停/恢复、帧序号、检测元数据），
  由 `car-vision-stream.service` 托管；
- `ros2_tools/vision_stream_client.py`、`vision_stream_test_source.py`：客户端与测试源。

### 5.9 云端平台对接

| 文件 | 作用 |
|---|---|
| `board_linux_uploader.py`（约 184 KB） | 上云主进程：状态/地图/识别/巡检报告/音频上报，轮询云端指令，看门狗，遥控 |
| `board_nav_command.py`（约 115 KB） | 云端导航执行器：初始位姿、航点巡航、重试与到点确认、巡检 job 联动、返回起点 |
| `teleop_shared.py` | 遥控状态与速度档位归一化（服务端/板端共用） |
| `start_board_linux_uploader.sh` | 启动脚本（注入 `CAR_CLOUD_SERVER_URL` 等环境变量） |
| `start_board_f103_navigation.sh` / `start_board_f103_mapping.sh` | 现场一键导航/建图编排脚本 |

云端地址在仓库中统一为占位符 `http://<cloud-host>:8765`，部署时注入（见第八节）。

### 5.10 运维与调试

- `cleanup_navigation_processes.py`：清理残留导航进程；
- `run_direct_follow_path_test.sh`：直接下发路径跟踪测试；
- `odom_motion_probe.py`：里程计运动探针（直线/旋转标定）；
- `ros_probe.py`、`test_bridge_startup.py`：ROS 图与桥接自检；
- `record_navigation_diagnostics.sh`：导航诊断数据录制；
- `start_camera_preview_1280x720.sh`：本地相机预览。

---

## 六、关键话题与服务

| 名称 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | Topic (Twist) | 导航输出的速度指令 |
| `/cmd_vel_cmd` / `/cmd_vel_mapping` | Topic (Twist) | 遥控 / 建图专用速度通道 |
| `/odom_raw` | Topic (Odometry) | F103 原始里程计 |
| `/odom` | Topic (Odometry) | EKF 融合后里程计 |
| `/imu/data` | Topic (Imu) | IMU 姿态 |
| `/scan` | Topic (LaserScan) | 激光雷达 |
| `/f103_state` | Topic (String) | 底盘状态（电压、使能、故障等） |
| `/arm_cmd` | Topic (String) | 机械臂指令 |
| `/arm/move_pose` | Service | 机械臂位姿服务（`car2_arm_interfaces`，可选） |
| `/navigate_to_pose` | Action | Nav2 导航目标 |
| `/map` / `/amcl_pose` | Topic | 地图与定位结果 |

---

## 七、主要配置文件

| 文件 | 说明 |
|---|---|
| `rock_ws/ros_ws/src/rt_robot_nav2/param/rt_robot_params.yaml` | Nav2 全套参数（规划器、控制器、代价地图、恢复行为） |
| `rt_robot_nav2/config/f103_ekf.yaml` | EKF 融合参数 |
| `rt_robot_nav2/config/velocity_limits.yaml` | 速度限幅 |
| `chassis_controller/config/chassis_controller.yaml` | 底盘节点参数 |
| `chassis_controller/config/auto_mapping_narrow.yaml` | 窄道自动建图参数 |
| `home/rock/*.yaml` | 现场单点调参（amcl / controller_server / planner_server / frontier_explorer / maze_turn_manager 等） |
| `car_cloud_uploader/lidar_maps/*.yaml` | 地图元数据（分辨率、原点、初始位姿、航点） |

---

## 八、部署与运维

### 8.1 systemd 单元

| 单元 | 作用 | 开机自启 |
|---|---|---|
| `f103-usb-chassis.service` | F103 底盘桥（`/cmd_vel` ↔ `/odom_raw`） | 是 |
| `f103-arm-startup-home.service` | 开机机械臂回 home 位 | 是 |
| `car-cloud-uploader.service` | 云端上报 + 指令轮询 | 是 |
| `car-recognition-worker.service` | RKNN 识别常驻 worker | 是 |
| `car-vision-stream.service` | MJPEG 视频流网关 | 是 |
| `cabinet-camera-overlay.service` | 相机去畸变 / AprilTag / 叠加显示 | 否（未启用） |
| `rt-microros-agent.service` | RT 底盘 micro-ROS agent | 否（未启用） |
| `rt-chassis-autostart.service` | RT 底盘初始化 | 否（未启用） |
| `rockchip-ap.service` | 厂商 AP 初始化 | 是 |

### 8.2 还原到板子

```bash
# 1) 业务代码
rsync -a home/rock/Desktop/ /home/rock/Desktop/
# 2) systemd / udev
sudo cp etc/systemd/system/*.service /etc/systemd/system/
sudo cp etc/udev/rules.d/*.rules /etc/udev/rules.d/
sudo systemctl daemon-reload
sudo udevadm control --reload-rules && sudo udevadm trigger

# 3) 启用当前流程
sudo systemctl enable --now f103-usb-chassis.service f103-arm-startup-home.service \
  car-cloud-uploader.service car-recognition-worker.service car-vision-stream.service
```

- `rock_ws/ros_ws/src` 需在板上重新 `colcon build`（仓库不含 build/install 产物）；
- 地图文件需放到 `~/Desktop/car_cloud_uploader/lidar_maps/`。

### 8.3 云平台地址注入（必须）

仓库中所有云平台地址均为占位符 `http://<cloud-host>:8765`，**部署时必须注入真实地址**：

```ini
# /etc/systemd/system/car-cloud-uploader.service.d/override.conf
[Service]
Environment=CAR_CLOUD_SERVER_URL=http://<真实地址>:8765
```

```bash
sudo systemctl daemon-reload && sudo systemctl restart car-cloud-uploader
```

也可在启动脚本前 `export CAR_CLOUD_SERVER_URL=...`，或用 `board_linux_uploader.py --server-url` 覆盖。

### 8.4 常用环境变量

`ROS_DOMAIN_ID=17`、`ROS_LOCALHOST_ONLY=1`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、
`FASTRTPS_DEFAULT_PROFILES_FILE=~/Desktop/rock_ws/fastdds_no_shm.xml`。

---

## 九、典型工作流

```bash
# 建图
ros2 launch rt_robot_nav2 rt_robot_nav2_complete.launch.py use_slam:=true use_nav:=false use_auto_mapping:=true

# 导航（已知地图）
ros2 launch rt_robot_nav2 rt_robot_nav2_complete.launch.py \
  use_slam:=false use_nav:=true use_chassis_controller:=true use_ekf:=true \
  map_file:=/home/rock/Desktop/car_cloud_uploader/lidar_maps/ceshi17.yaml

# 键盘遥控
~/Desktop/rt_direct_tools/start_board_keyboard_teleop.sh

# 单柜巡检（AprilTag）
python3 ~/Desktop/car_cloud_uploader/ros2_tools/cabinet_apriltag_inspection.py \
  --search-config ~/Desktop/car_cloud_uploader/config/arm_apriltag_s0_search.yaml \
  --profile ~/Desktop/car_cloud_uploader/cabinet_inspection_profiles/fixed_scan.yaml
```

---

## 十、工程快照说明

本仓库是**车载板现场快照**，不是从零构建的源码树，用于完整留存与回滚现场版本。

| 项 | 值 |
|---|---|
| 来源镜像 | `sdcard_20261011_191620.img`（GPT：`boot` FAT32 + `rootfs` ext4 29.2 GiB） |
| 镜像 SHA-256 | `92179FA065D6F05EAE878A6F2FC04603C70E7C0871C221C127BC7C1BB3DB2293` |
| 提取方式 | WSL2 内 `losetup -r` + `mount -o ro,noload`，全程只读 |
| 规模 | 771 个文件 / 约 195 MB |
| 一致性 | 205 个核心文件与现场镜像逐字节一致（SHA-256 校验） |

**排除内容**：运行态 `runtime/`、日志、`backups/`、`.codex_backups/`、`__pycache__/`、
随板打包的 Python 运行环境 `yolo_runtime/`、boot 分区内核与 `HyperBoot.bin`。

**现场遗留**：目录中保留了大量 `.bak_*`、`.before_*`、`_candidate` 文件，是现场快速迭代的备份，
不影响运行，可按需清理。地图 `ceshi*` 为调试用测试地图。

---

## 十一、已知限制

- 仓库只含源码与配置，**不含编译产物**（`build/`、`install/`），新机器需自行 `colcon build`；
- 依赖版本以现场为准（Ubuntu 20.04 + ROS 2 Foxy + Python 3.8），未提供锁文件；
- 部分脚本内嵌了现场标定参数（轮径、角速度缩放、IMU 零偏），换车体需重新标定；
- RT-Thread / micro-ROS 链路与 `cabinet-camera-overlay` 服务在设备上未启用，属备用/实验路径。
