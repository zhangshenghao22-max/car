# board_current — ROCK 5B 车载板现场快照

本仓库是对 **E:\board_backups\sdcard_20261011_191620.img** 中「当前在用流程」相关文件的完整快照，
按板子上的绝对路径原样保存，便于检索、比对和回滚。

## 一、来源与校验

| 项 | 值 |
|---|---|
| 镜像文件 | `E:\board_backups\sdcard_20261011_191620.img` |
| 镜像大小 | 31,914,983,424 字节（29.72 GiB） |
| 镜像 SHA-256 | `92179FA065D6F05EAE878A6F2FC04603C70E7C0871C221C127BC7C1BB3DB2293` |
| 分区布局 | GPT：p0 `boot` FAT32 512 MiB；p1 `rootfs` ext4 29.21 GiB |
| 系统 | Ubuntu 20.04 LTS / kernel 5.10.66-27-rockchip / ROS 2 Foxy |
| 主机名 | rock-5b |
| 抓取方式 | WSL2 内 `losetup -r` + `mount -o ro,noload`，全程只读，未写回镜像 |
| 抓取时间 | 2026-10-18 |

镜像本身是「未干净卸载」的运行态备份（ext4 超级块 RECOVER 位仍置位），
因此挂载必须加 `noload`，不要对原镜像执行 `e2fsck -y`。

## 二、目录结构

```
home/rock/Desktop/car_cloud_uploader/   云端上传、导航指令、柜体巡检、视觉识别（Python）
home/rock/Desktop/rt_direct_tools/      F103 USB 底盘桥、机械臂遥控、标定与启停脚本
home/rock/Desktop/rock_ws/ros_ws/src/   ROS2 源码包（rt_robot_nav2、chassis_controller、
                                        lslidar_driver、slam_gmapping、robot_urdf 等）
home/rock/*.yaml                        现场调参用的 Nav2/AMCL/EKF 参数
etc/systemd/system/                     自定义 systemd 单元（f103-*、car-*、rt-*、cabinet-*）
etc/udev/rules.d/                       rplidar / IMU / 相机等设备规则
MANIFEST/                               板上文件哈希清单与本地比对结果
```

规模：约 790 个文件、195 MB。

## 三、当前流程（本仓库覆盖的范围）

- 底盘：`f103-usb-chassis.service` → `ros2_tools/f103_usb_ros2_bridge.py`（USB CDC 115200）
- 定位：`/odom_raw` + `/imu/data` → EKF → `/odom`；AMCL + 已保存地图导航
- 导航：`rt_robot_nav2`（NavfnPlanner + DWB），`rt_robot_nav2_complete.launch.py`
- 业务：`board_linux_uploader.py`（云端平台上报与指令轮询，服务地址见板端配置）、`board_nav_command.py`（巡航/巡检）
- 视觉：`recognition_worker.py`（RKNN 常驻）、`camera_yolo_detect.py`、`cabinet_apriltag_inspection.py`
- 机械臂：`arm_cmd_cli.py`、`board_keyboard_arm_teleop.py`、`f103-arm-startup-home.service`

> 板上 RT-Thread / micro-ROS 链路（`rt-microros-agent.service`、`rt-chassis-autostart.service`）
> 未 enable，属于备用方案；其单元文件仍收在本仓库 `etc/systemd/system/` 里。

## 四、包含与排除

**包含**：全部业务源码、启动脚本、配置 yaml、地图（`lidar_maps`）、YOLO/ONNX 模型、
URDF 与网格、systemd 单元、udev 规则、以及现场留存的历史备份文件（`*.bak_*` 等）。

**排除**（不属于源码，或体积过大）：

- `car_cloud_uploader/runtime/`：运行时状态与日志（1.4 GB）
- `car_cloud_uploader/backups/`、`rt_direct_tools/backups/`、`rt_direct_tools/logs/`
- `.codex_backups/`、`.codex_staging_*/`、`.deploy_*/`：AI 助手 staging 目录
- `__pycache__/`、`*.pyc`
- `car_cloud_uploader/yolo_runtime/`：随板打包的 Python 运行环境（139 MB）
- boot 分区内容（内核、dtb、`HyperBoot.bin` 等）、`ai_app.zip`、桌面固件 bin

需要时可从原镜像按同样方式再抽取。

## 五、与本地工程的比对结论

对本仓库覆盖的 205 个代码/配置/单元文件做了 SHA-256 比对：

- **196 个**在 `D:\project` 下存在字节级一致的原件（主要位于 `D:\project\car2.0\runtime\` 的各次部署暂存目录）
- **9 个**仅存在于板上：

| 文件 | 说明 |
|---|---|
| `lidar_maps/ceshi14~18.yaml` | 板上现场建的测试地图（5 个） |
| `yolo_models/lightlabel_stronglight_20260729.metrics.json` | 模型训练指标 |
| `etc/systemd/system/rockchip-ap.service` | 厂商 AP 初始化单元 |
| `etc/systemd/system/rt-chassis-autostart.service` | RT 底盘初始化（未启用） |
| `etc/systemd/system/rt-microros-agent.service` | micro-ROS agent（未启用） |

明细见 `MANIFEST/board_vs_local.csv`、`MANIFEST/board_inventory.tsv`。

## 六、如何还原到板子

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

注意：

- `rock_ws/ros_ws/src` 需在板上重新 `colcon build`（本仓库不含 build/install 产物）。
- 仓库里的云平台地址统一写成占位符 `http://<cloud-host>:8765`，**部署时必须通过环境变量注入真实地址**，例如创建 `/etc/systemd/system/car-cloud-uploader.service.d/override.conf`：

```ini
[Service]
Environment=CAR_CLOUD_SERVER_URL=http://<真实地址>:8765
```

  或在启动脚本前 `export CAR_CLOUD_SERVER_URL=...`；`board_linux_uploader.py --server-url` 亦可用参数覆盖。

## 七、备注

- 文件中保留了现场时间戳（部分为板子 RTC 时间，可能早于/晚于真实时间）。
- `.gitattributes` 已设为 `* -text`，避免 Windows 检出时改写行尾，保证与板上字节一致。
