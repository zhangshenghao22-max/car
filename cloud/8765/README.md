# RK3588 Cloud Platform Rebuild

## Goal

This server-side platform mirrors the current RK3588 board's mapping and navigation state in a browser.

It is responsible for:
- receiving board Linux uploads
- showing the latest realtime map image
- showing the latest scan image
- overlaying navigation path, robot pose, and goal pose on top of the map
- showing mapping/navigation status and recent logs

It does not run board serial control, ROS bringup, lidar, or navigation logic itself.

## Layout

- `app.py`
- `requirements.txt`
- `start_cloud_platform.sh`
- `stop_cloud_platform.sh`
- `templates/index.html`
- `static/app.js`
- `static/style.css`
- `board_linux_uploader.py`

## Runtime Ports

- Server local listen port: `5000`
- Public access entry: `8765`

Current server firewall forwards:
- `115.159.33.216:8765 -> 127.0.0.1:5000`

So:
- inside the server, test with `http://127.0.0.1:5000`
- outside the server, open `http://115.159.33.216:8765`

## Server API

- `GET /ping`
- `GET /api/cloud/state`
- `GET /api/cloud/frame/<kind>`
- `POST /api/upload/state`
- `POST /api/upload/frame/<kind>`
- `POST /api/upload/snapshot`

Upload authentication:
- header: `X-Upload-Token`

Allowed frame kinds:
- `map`
- `scan`
- `camera`
- `nav_overlay`

## Display Logic

- map panel shows the latest uploaded `map` frame
- scan panel shows the latest uploaded `scan` frame
- path overlay uses:
  - `map_meta.width`
  - `map_meta.height`
  - `map_meta.resolution`
  - `map_meta.origin_x`
  - `map_meta.origin_y`
  - `nav_path.points`
  - `robot_pose`
  - `goal_pose`

If only a map image is uploaded and `map_meta` is missing, the map image still shows, but path overlay cannot be drawn correctly.

## Deploy To Server

Target directory:
- `/root/car/yun`

### 1. Upload files

```bash
scp -r cloud_platform_server/* root@115.159.33.216:/root/car/yun/
```

### 2. Start

```bash
ssh root@115.159.33.216
cd /root/car/yun
bash ./start_cloud_platform.sh
```

The start script now auto-creates `.venv` and auto-installs `requirements.txt` if `flask` or `requests` is missing.

### 3. Stop

```bash
ssh root@115.159.33.216
cd /root/car/yun
bash ./stop_cloud_platform.sh
```

### 4. Verify

Inside the server:

```bash
curl http://127.0.0.1:5000/ping
curl http://127.0.0.1:5000/api/cloud/state
```

From outside:

```bash
curl http://115.159.33.216:8765/ping
curl http://115.159.33.216:8765/api/cloud/state
```

## Linux Board Upload Scheme

Recommended design:
- keep all ROS/serial/mapping/navigation logic on the board
- run one independent uploader process on board Linux
- let the uploader read local board status and images, then push them to the cloud server

### Local Board Data Sources

Board web APIs:
- `GET http://127.0.0.1:5000/api/status`
- `GET http://127.0.0.1:5000/api/ros/status`
- `GET http://127.0.0.1:5000/api/nav/status`
- `GET http://127.0.0.1:5000/api/logs`

Board MJPEG streams:
- `GET http://127.0.0.1:5000/stream/lidar_map.mjpg`
- `GET http://127.0.0.1:5000/stream/lidar_scan.mjpg`

Board ROS topics:
- `/map` -> map metadata
- `/plan` or `/global_plan` -> path points
- `/amcl_pose` -> robot pose
- `nav_status.current_goal` -> goal pose

### Recommended Upload Rates

- state JSON: `1 Hz`
- map frame: `0.5 Hz`
- scan frame: `0.5 Hz`
- path / pose: piggyback in state JSON

### Board Command Example

Run on board Linux:

```bash
python3 board_linux_uploader.py \
  --server-url http://115.159.33.216:8765 \
  --upload-token car-cloud-upload \
  --board-url http://127.0.0.1:5000 \
  --board-id rk3588-f103-board \
  --board-label "RK3588 F103 Board"
```

### Suggested Production Mode

Use a dedicated service on the board:
- start after board web app and ROS stack are up
- restart automatically if upload process exits
- do not merge uploader logic into the board mapping/navigation main script

Minimal `systemd` example on board Linux:

```ini
[Unit]
Description=Board Cloud Uploader
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
WorkingDirectory=/home/rock/car4.0/cloud_platform_server
ExecStart=/usr/bin/python3 /home/rock/car4.0/cloud_platform_server/board_linux_uploader.py --server-url http://115.159.33.216:8765 --upload-token car-cloud-upload --board-url http://127.0.0.1:5000 --board-id rk3588-f103-board --board-label RK3588-F103
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
```

## Notes

- Public entry is `8765`, not raw `5000`
- Server app stores latest state in `runtime/latest_state.json`
- Latest images are stored in `runtime/frames/`
- The current upload token default is `car-cloud-upload`
- If path overlay is missing, check `nav_path.points`, `map_meta`, and `robot_pose`
