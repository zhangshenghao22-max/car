from __future__ import annotations

import io
import json
import mimetypes
import time
from pathlib import Path
from socketserver import ThreadingMixIn
from wsgiref.simple_server import WSGIRequestHandler, WSGIServer, make_server

from flask import Flask, Response, jsonify, render_template, request, send_file
from PIL import Image
from werkzeug.utils import secure_filename

from board_backend import (
    BoardPlatform,
    MAP_EXPORT_DIR,
    SUPPORTED_VISION_MEDIA_EXTENSIONS,
    UPLOAD_MEDIA_DIR,
    VISION_EXPORT_DIR,
    draw_text_panel,
)

app = Flask(__name__, template_folder="board_templates", static_folder="board_static")
platform = BoardPlatform()


def _json_ok(**kwargs):
    payload = {"ok": True}
    payload.update(kwargs)
    return jsonify(payload)


def _json_error(message: str, status_code: int = 400, **kwargs):
    payload = {"ok": False, "message": message}
    payload.update(kwargs)
    response = jsonify(payload)
    response.status_code = status_code
    return response


def _ros_snapshot():
    return platform.ros_status()


def _ros_error_payload(message: str, status: dict | None = None):
    ros_status = dict(status or platform.refresh_ros_status(force=True))
    error_code = ros_status.get("error_code") or platform.ros.error_code_for(message)
    last_ready_error = ros_status.get("last_ready_error") or message
    ros_status["error_code"] = error_code
    ros_status["last_ready_error"] = last_ready_error
    ros_status = platform.store_ros_status(ros_status)
    return {
        "status": ros_status,
        "error_code": error_code,
        "last_ready_error": last_ready_error,
        "warning": "",
    }


def _stream_bytes(frame_func, fallback_lines):
    while True:
        payload = frame_func()
        if not payload:
            payload = draw_text_panel(fallback_lines)
        yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + payload + b"\r\n")
        time.sleep(0.08)


def _send_scoped_file(root_dir: Path, relative_path: str):
    root = root_dir.resolve()
    path = (root / relative_path).resolve()
    if root not in path.parents and path != root:
        return _json_error("非法路径", 403)
    if not path.exists():
        return _json_error("文件不存在", 404)
    guessed = mimetypes.guess_type(str(path))[0]
    return send_file(path, mimetype=guessed or "application/octet-stream")



def _send_browser_image(path: Path):
    if not path.exists():
        return _json_error("图片不存在", 404)
    if path.suffix.lower() == ".pgm":
        with Image.open(path) as image:
            buffer = io.BytesIO()
            image.save(buffer, format="PNG")
            buffer.seek(0)
        return send_file(buffer, mimetype="image/png", download_name=f"{path.stem}.png")
    guessed = mimetypes.guess_type(str(path))[0]
    return send_file(path, mimetype=guessed or "image/png")

@app.route("/ping")
def ping():
    return "pong", 200


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/status")
def api_status():
    return jsonify(platform.all_status())


@app.route("/api/ros/status")
def api_ros_status():
    return jsonify(_ros_snapshot())


@app.route("/api/control/connect", methods=["POST"])
def api_control_connect():
    payload = request.get_json(force=True, silent=True) or {}
    ok, message = platform.controller.connect(payload.get("port", ""), int(payload.get("baudrate", 115200)))
    if ok:
        return _json_ok(message=message, status=platform.all_status()["control"])
    return _json_error(message)


@app.route("/api/control/disconnect", methods=["POST"])
def api_control_disconnect():
    platform.controller.disconnect()
    return _json_ok(message="通信已断开")


@app.route("/api/control/ble/scan", methods=["POST"])
def api_control_ble_scan():
    payload = request.get_json(force=True, silent=True) or {}
    try:
        devices = platform.controller.scan_ble_devices(float(payload.get("timeout", 4.0)))
    except Exception as exc:
        return _json_error(str(exc))
    return _json_ok(message="蓝牙扫描完成", devices=devices, status=platform.all_status()["control"])


@app.route("/api/control/ble/connect", methods=["POST"])
def api_control_ble_connect():
    payload = request.get_json(force=True, silent=True) or {}
    ok, message = platform.controller.connect_ble(
        address=str(payload.get("address", "")).strip(),
        label=str(payload.get("label", "")).strip(),
        write_uuid=str(payload.get("write_uuid", "")).strip(),
        notify_uuid=str(payload.get("notify_uuid", "")).strip() or None,
        response=bool(payload.get("response", False)),
    )
    if ok:
        return _json_ok(message=message, status=platform.all_status()["control"])
    return _json_error(message)


@app.route("/api/control/ble/disconnect", methods=["POST"])
def api_control_ble_disconnect():
    platform.controller.disconnect_ble()
    return _json_ok(message="蓝牙已断开", status=platform.all_status()["control"])


@app.route("/api/control/motion", methods=["POST"])
def api_control_motion():
    payload = request.get_json(force=True, silent=True) or {}
    ok, message = platform.controller.send_motion(payload.get("action", ""))
    if ok:
        return _json_ok(message=message or "动作命令已发送", status=platform.all_status()["control"])
    return _json_error(message)


@app.route("/api/control/raw", methods=["POST"])
def api_control_raw():
    payload = request.get_json(force=True, silent=True) or {}
    ok, message = platform.controller.send_raw(payload.get("command", ""))
    if ok:
        return _json_ok(message="自定义命令已发送")
    return _json_error(message)


@app.route("/api/control/handshake", methods=["POST"])
def api_control_handshake():
    ok, message = platform.controller.handshake()
    if ok:
        return _json_ok(message=message, status=platform.all_status()["control"])
    return _json_error(message)


@app.route("/api/control/status/request", methods=["POST"])
def api_control_status_request():
    ok, message = platform.controller.request_status()
    if ok:
        return _json_ok(message="状态请求已发送", status=platform.all_status()["control"])
    return _json_error(message)


@app.route("/api/control/report", methods=["POST"])
def api_control_report():
    payload = request.get_json(force=True, silent=True) or {}
    ok, message = platform.controller.set_report_stream(bool(payload.get("enabled", True)))
    if ok:
        enabled = bool(payload.get("enabled", True))
        text = "已开启状态连续上报" if enabled else "已关闭状态连续上报"
        return _json_ok(message=text, status=platform.all_status()["control"])
    return _json_error(message or "状态上报命令发送失败")


@app.route("/api/control/mode", methods=["POST"])
def api_control_mode():
    payload = request.get_json(force=True, silent=True) or {}
    ok, message = platform.controller.set_mode(str(payload.get("mode", "")))
    if ok:
        return _json_ok(message=f"模式切换命令已发送: {payload.get('mode', '')}", status=platform.all_status()["control"])
    return _json_error(message)


@app.route("/api/control/estop", methods=["POST"])
def api_control_estop():
    ok, message = platform.controller.emergency_stop()
    if ok:
        return _json_ok(message=message or "急停命令已发送", status=platform.all_status()["control"])
    return _json_error(message)


@app.route("/api/control/estop/clear", methods=["POST"])
def api_control_estop_clear():
    ok, message = platform.controller.clear_emergency_stop()
    if ok:
        return _json_ok(message=message or "急停清除命令已发送", status=platform.all_status()["control"])
    return _json_error(message)


@app.route("/api/control/servo", methods=["POST"])
def api_control_servo():
    payload = request.get_json(force=True, silent=True) or {}
    targets = payload.get("targets", {})
    normalized = {int(key): int(value) for key, value in targets.items()}
    ok, message = platform.controller.send_servo_targets(normalized, duration=int(payload.get("duration", 120)))
    if ok:
        return _json_ok(message="关节命令已发送", servo_values=platform.controller.servo_values)
    return _json_error(message)


@app.route("/api/control/home", methods=["POST"])
def api_control_home():
    ok, message = platform.controller.home()
    if ok:
        return _json_ok(message="机械臂已回初始位", servo_values=platform.controller.servo_values)
    return _json_error(message)


@app.route("/api/control/servo/reset", methods=["POST"])
def api_control_servo_reset():
    ok, message = platform.controller.reset_arm()
    if ok:
        return _json_ok(message="机械臂复位命令已发送", servo_values=platform.controller.servo_values)
    return _json_error(message)


@app.route("/api/control/servo/stop", methods=["POST"])
def api_control_servo_stop():
    ok, message = platform.controller.stop_all_servos()
    if ok:
        return _json_ok(message="机械臂停止命令已发送")
    return _json_error(message)


@app.route("/api/vision/start", methods=["POST"])
def api_vision_start():
    payload = request.get_json(force=True, silent=True) or {}
    ok, message = platform.vision.start(int(payload.get("camera_index", 0)))
    if ok:
        return _json_ok(message=message)
    return _json_error(message)


@app.route("/api/vision/stop", methods=["POST"])
def api_vision_stop():
    platform.vision.stop()
    return _json_ok(message="摄像头已停止")


@app.route("/api/vision/cameras")
def api_vision_cameras():
    return _json_ok(cameras=platform.vision.list_cameras())


@app.route("/api/vision/load_models", methods=["POST"])
def api_vision_load_models():
    ok, message = platform.vision.load_models()
    if ok:
        return _json_ok(message=message, status=platform.vision.status())
    return _json_error(message)


@app.route("/api/vision/config", methods=["POST"])
def api_vision_config():
    payload = request.get_json(force=True, silent=True) or {}
    platform.vision.set_flags(
        yolo_enabled=payload.get("yolo_enabled"),
        meter_enabled=payload.get("meter_enabled"),
        tracking_enabled=payload.get("tracking_enabled"),
        yolo_confidence=payload.get("yolo_confidence"),
        meter_confidence=payload.get("meter_confidence"),
    )
    return _json_ok(message="视觉设置已更新", status=platform.vision.status())


@app.route("/api/vision/upload", methods=["POST"])
def api_vision_upload():
    upload = request.files.get("file")
    if upload is None or not upload.filename:
        return _json_error("请先选择图片文件")

    suffix = Path(upload.filename).suffix.lower()
    if suffix not in SUPPORTED_VISION_MEDIA_EXTENSIONS:
        allowed = "、".join(sorted(SUPPORTED_VISION_MEDIA_EXTENSIONS))
        return _json_error(f"暂仅支持这些格式: {allowed}")

    safe_name = secure_filename(upload.filename)
    safe_stem = Path(safe_name).stem[:60] if safe_name else ""
    safe_stem = safe_stem or "upload_media"
    saved_name = f"{time.strftime('%Y%m%d_%H%M%S')}_{int(time.time() * 1000) % 1000:03d}_{safe_stem}{suffix}"
    saved_path = UPLOAD_MEDIA_DIR / saved_name
    upload.save(saved_path)

    try:
        result = platform.vision.analyze_uploaded_media(saved_path)
        return _json_ok(message=result.get("message", "识别完成"), result=result)
    except Exception as exc:
        return _json_error(str(exc))


@app.route("/api/vision/result/<path:filename>")
def api_vision_result(filename: str):
    return _send_scoped_file(VISION_EXPORT_DIR, filename)


@app.route("/stream/camera.mjpg")
def stream_camera():
    return Response(
        _stream_bytes(
            platform.vision.frame_jpeg,
            [
                "摄像头尚未启动",
                "请先在网页中点击“启动摄像头”。",
                "如果模型首次加载较慢，请等待 10~60 秒。",
            ],
        ),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/api/lidar/start", methods=["POST"])
def api_lidar_start():
    payload = request.get_json(force=True, silent=True) or {}
    ok, message = platform.lidar.start(payload.get("port", ""))
    if ok:
        return _json_ok(message=message)
    return _json_error(message)


@app.route("/api/lidar/stop", methods=["POST"])
def api_lidar_stop():
    platform.lidar.stop()
    return _json_ok(message="雷达建图已停止")


@app.route("/api/lidar/reset", methods=["POST"])
def api_lidar_reset():
    platform.lidar.reset()
    return _json_ok(message="已请求重置地图")


@app.route("/api/lidar/save", methods=["POST"])
def api_lidar_save():
    try:
        saved = platform.lidar.save()
        return _json_ok(message="地图已保存", files=saved)
    except Exception as exc:
        return _json_error(str(exc))


@app.route("/stream/lidar_scan.mjpg")
def stream_lidar_scan():
    return Response(
        _stream_bytes(platform.ros.scan_jpeg, ["ROS 扫点预览等待数据", "请先启动 ROS 建图并确认雷达已上电。"]),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/stream/lidar_map.mjpg")
def stream_lidar_map():
    return Response(
        _stream_bytes(platform.ros.map_jpeg, ["ROS 地图预览等待数据", "启动后会持续刷新并显示 slam_toolbox 地图。"]),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/api/ros/mapping/start", methods=["POST"])
def api_ros_mapping_start():
    payload = request.get_json(force=True, silent=True) or {}
    try:
        result = platform.ros.start_mapping(payload.get("port", ""))
        status = platform.store_ros_status(result["status"])
        return _json_ok(
            message=result["message"],
            status=status,
            rviz_started=result["rviz_started"],
            rviz_reused=result["rviz_reused"],
            rviz_render_mode=result["rviz_render_mode"],
            warning=result["warning"],
            error_code=status.get("error_code", ""),
            last_ready_error=status.get("last_ready_error", ""),
        )
    except Exception as exc:
        return _json_error(str(exc), **_ros_error_payload(str(exc)))


@app.route("/api/ros/mapping/stop", methods=["POST"])
def api_ros_mapping_stop():
    try:
        message = platform.ros.stop_mapping()
        status = platform.refresh_ros_status(force=True)
        return _json_ok(
            message=message,
            status=status,
            warning="",
            error_code=status.get("error_code", ""),
            last_ready_error=status.get("last_ready_error", ""),
        )
    except Exception as exc:
        return _json_error(str(exc), **_ros_error_payload(str(exc)))


@app.route("/api/ros/mapping/reset", methods=["POST"])
def api_ros_mapping_reset():
    payload = request.get_json(force=True, silent=True) or {}
    try:
        result = platform.ros.reset_mapping(payload.get("port", ""))
        status = platform.store_ros_status(result["status"])
        return _json_ok(
            message=result["message"],
            status=status,
            rviz_started=result["rviz_started"],
            rviz_reused=result["rviz_reused"],
            rviz_render_mode=result["rviz_render_mode"],
            warning=result["warning"],
            error_code=status.get("error_code", ""),
            last_ready_error=status.get("last_ready_error", ""),
        )
    except Exception as exc:
        return _json_error(str(exc), **_ros_error_payload(str(exc)))


@app.route("/api/ros/mapping/save", methods=["POST"])
def api_ros_mapping_save():
    payload = request.get_json(force=True, silent=True) or {}
    try:
        saved = platform.ros.save_map(payload.get("name", ""))
        status = platform.refresh_ros_status(force=True)
        return _json_ok(
            message="ROS 地图已保存",
            files=saved,
            status=status,
            warning="",
            error_code=status.get("error_code", ""),
            last_ready_error=status.get("last_ready_error", ""),
        )
    except Exception as exc:
        return _json_error(str(exc), **_ros_error_payload(str(exc)))


@app.route("/api/ros/maps")
def api_ros_maps():
    maps = platform.ros.list_saved_maps()
    return _json_ok(maps=maps, latest=maps[0] if maps else None, total=platform.ros.saved_maps_count())


@app.route("/api/ros/maps/image/<map_name>")
def api_ros_maps_image(map_name: str):
    path = platform.ros.map_artifact_path(map_name, "image")
    if path is None or not path.exists():
        return _json_error("未找到地图预览图片", 404)
    return _send_browser_image(path)


@app.route("/api/ros/maps/download/<map_name>/<artifact>")
def api_ros_maps_download(map_name: str, artifact: str):
    if artifact not in {"yaml", "image"}:
        return _json_error("不支持的地图文件类型", 400)
    path = platform.ros.map_artifact_path(map_name, artifact)
    if path is None or not path.exists():
        return _json_error("未找到地图文件", 404)
    guessed = mimetypes.guess_type(str(path))[0]
    return send_file(path, mimetype=guessed or "application/octet-stream", as_attachment=True, download_name=path.name)


@app.route("/api/ros/rviz/open", methods=["POST"])
def api_ros_rviz_open():
    payload = request.get_json(force=True, silent=True) or {}
    try:
        result = platform.ros.open_rviz(str(payload.get("display", "")).strip())
        status = platform.store_ros_status(result["status"])
        return _json_ok(
            message=result["message"],
            status=status,
            rviz_started=not result["reused"],
            rviz_reused=result["reused"],
            rviz_render_mode=result["render_mode"],
            warning=result["warning"],
            error_code=status.get("error_code", ""),
            last_ready_error=status.get("last_ready_error", ""),
        )
    except Exception as exc:
        return _json_error(str(exc), **_ros_error_payload(str(exc)))


@app.route("/api/avoidance/lidar/start", methods=["POST"])
def api_avoid_lidar_start():
    payload = request.get_json(force=True, silent=True) or {}
    ok, message = platform.avoidance.start_lidar(payload.get("port", ""))
    if ok:
        return _json_ok(message=message)
    return _json_error(message)


@app.route("/api/avoidance/lidar/stop", methods=["POST"])
def api_avoid_lidar_stop():
    platform.avoidance.stop_lidar()
    return _json_ok(message="避障雷达已停止")


@app.route("/api/avoidance/start", methods=["POST"])
def api_avoid_start():
    ok, message = platform.avoidance.start()
    if ok:
        return _json_ok(message=message)
    return _json_error(message)


@app.route("/api/avoidance/stop", methods=["POST"])
def api_avoid_stop():
    ok, message = platform.avoidance.stop()
    if ok:
        return _json_ok(message=message)
    return _json_error(message)


@app.route("/api/avoidance/threshold", methods=["POST"])
def api_avoid_threshold():
    payload = request.get_json(force=True, silent=True) or {}
    value = int(payload.get("threshold_mm", 50))
    platform.avoidance.set_threshold(value)
    return _json_ok(message=f"避障阈值已更新为 {value} mm")


@app.route("/stream/avoidance_scan.mjpg")
def stream_avoidance_scan():
    return Response(
        _stream_bytes(platform.avoidance.scan_jpeg, ["避障雷达尚未启动", "先启动避障页中的雷达。"]),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/api/maps/list")
def api_maps_list():
    return jsonify({"maps": platform.maps.list_maps()})


@app.route("/api/maps/image/<map_name>")
def api_maps_image(map_name: str):
    path = platform.maps.map_image_path(map_name)
    if path is None or not path.exists():
        return _json_error("未找到地图图片", 404)
    return _send_browser_image(path)


@app.route("/api/maps/annotations/<map_name>", methods=["GET", "POST"])
def api_maps_annotations(map_name: str):
    if request.method == "GET":
        return jsonify(platform.maps.load_annotations(map_name))
    payload = request.get_json(force=True, silent=True) or {}
    saved = platform.maps.save_annotations(map_name, payload)
    return _json_ok(message="标注已保存", path=saved)


@app.route("/api/nav/status")
def api_nav_status():
    return jsonify(platform.nav.status(force=True))


@app.route("/api/nav/localization/start", methods=["POST"])
def api_nav_localization_start():
    payload = request.get_json(force=True, silent=True) or {}
    try:
        result = platform.nav.start_localization(
            str(payload.get("map_name", "")).strip(),
            str(payload.get("port", "")).strip(),
        )
        return _json_ok(message=result["message"], status=result["status"], warning=result.get("warning", ""))
    except Exception as exc:
        return _json_error(str(exc), status=platform.nav.status(force=True))


@app.route("/api/nav/localization/stop", methods=["POST"])
def api_nav_localization_stop():
    try:
        result = platform.nav.stop_localization()
        return _json_ok(message=result["message"], status=result["status"])
    except Exception as exc:
        return _json_error(str(exc), status=platform.nav.status(force=True))


@app.route("/api/nav/goal/start", methods=["POST"])
def api_nav_goal_start():
    payload = request.get_json(force=True, silent=True) or {}
    try:
        result = platform.nav.start_goal(payload)
        return _json_ok(message=result["message"], status=result["status"])
    except Exception as exc:
        return _json_error(str(exc), status=platform.nav.status(force=True))


@app.route("/api/nav/goal/cancel", methods=["POST"])
def api_nav_goal_cancel():
    try:
        result = platform.nav.cancel_goal()
        return _json_ok(message=result["message"], status=result["status"])
    except Exception as exc:
        return _json_error(str(exc), status=platform.nav.status(force=True))


@app.route("/api/nav/tasks/<map_name>", methods=["GET", "POST"])
def api_nav_tasks(map_name: str):
    if request.method == "GET":
        try:
            return jsonify(platform.nav.load_tasks(map_name))
        except Exception as exc:
            return _json_error(str(exc))
    payload = request.get_json(force=True, silent=True) or {}
    try:
        saved = platform.nav.save_tasks(map_name, payload)
        return _json_ok(message="任务配置已保存", tasks=saved)
    except Exception as exc:
        return _json_error(str(exc))


@app.route("/api/nav/task/start", methods=["POST"])
def api_nav_task_start():
    payload = request.get_json(force=True, silent=True) or {}
    try:
        result = platform.nav.start_task(payload)
        return _json_ok(message=result["message"], status=result["status"])
    except Exception as exc:
        return _json_error(str(exc), status=platform.nav.status(force=True))


@app.route("/api/nav/task/stop", methods=["POST"])
def api_nav_task_stop():
    try:
        result = platform.nav.stop_task()
        return _json_ok(message=result["message"], status=result["status"])
    except Exception as exc:
        return _json_error(str(exc), status=platform.nav.status(force=True))


@app.route("/api/logs")
def api_logs():
    return jsonify({"logs": platform.logger.tail(200)})


@app.route("/api/files/<path:relative_path>")
def api_files(relative_path: str):
    return _send_scoped_file(MAP_EXPORT_DIR, relative_path)


class ThreadingWSGIServer(ThreadingMixIn, WSGIServer):
    daemon_threads = True


if __name__ == "__main__":
    with make_server("0.0.0.0", 5000, app, server_class=ThreadingWSGIServer, handler_class=WSGIRequestHandler) as httpd:
        httpd.serve_forever()
