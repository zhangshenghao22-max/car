#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import queue
import re
import signal
import threading
import time
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String as RosString


SERVO_COUNT = 6
MIN_PWM = 500
MAX_PWM = 2500
MIN_DURATION_MS = 20
MAX_DURATION_MS = 9999
HOME_PWM = [1640, 1500, 1500, 2250, 1500, 1500]
STATE_PWM_RE = re.compile(r"(?:^|,)S([0-5])=(-?\d+)(?=,|!)")


HTML_PAGE = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <meta name="robots" content="noindex,nofollow">
  <title>机械臂手动控制</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #edf0f2;
      --panel: #ffffff;
      --ink: #1f262b;
      --muted: #68747c;
      --line: #cfd6da;
      --accent: #176b87;
      --accent-strong: #0d536b;
      --warn: #b5482d;
      --ok: #2d7d51;
      --focus: #f0a53a;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: linear-gradient(135deg, #f7f8f8 0%, var(--bg) 55%, #e4e9e6 100%);
      color: var(--ink);
      font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
      min-height: 100vh;
      letter-spacing: 0;
    }
    header {
      background: #20282d;
      color: #f7f9fa;
      border-bottom: 4px solid var(--focus);
    }
    .header-inner, main { width: min(1120px, calc(100% - 28px)); margin: 0 auto; }
    .header-inner {
      min-height: 74px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 18px;
      padding: 12px 0;
    }
    h1 { font-size: 22px; margin: 0; font-weight: 700; }
    .subtitle { color: #b9c5ca; font-size: 13px; margin-top: 3px; }
    .connection {
      display: flex;
      align-items: center;
      gap: 8px;
      font-size: 13px;
      white-space: nowrap;
    }
    .dot { width: 10px; height: 10px; border-radius: 50%; background: var(--warn); }
    .dot.ok { background: #51c983; box-shadow: 0 0 0 3px rgba(81, 201, 131, .18); }
    main { padding: 18px 0 30px; }
    .toolbar {
      display: grid;
      grid-template-columns: repeat(2, minmax(150px, 220px)) 1fr auto;
      align-items: end;
      gap: 12px;
      margin-bottom: 14px;
      padding: 14px;
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 6px;
    }
    label { display: block; color: var(--muted); font-size: 12px; font-weight: 700; }
    input[type="number"] {
      width: 100%;
      height: 38px;
      margin-top: 5px;
      border: 1px solid #aeb9bf;
      border-radius: 4px;
      padding: 0 10px;
      color: var(--ink);
      background: #fff;
      font: inherit;
    }
    input:focus, button:focus-visible { outline: 3px solid rgba(240,165,58,.35); outline-offset: 1px; }
    button {
      min-height: 38px;
      border: 1px solid transparent;
      border-radius: 4px;
      padding: 0 14px;
      font: inherit;
      font-weight: 700;
      cursor: pointer;
      background: var(--accent);
      color: #fff;
    }
    button:hover { background: var(--accent-strong); }
    button:disabled { opacity: .45; cursor: not-allowed; }
    button.secondary { color: var(--ink); background: #eef2f3; border-color: #b9c3c8; }
    button.secondary:hover { background: #e1e8ea; }
    button.stop { background: var(--warn); }
    button.stop:hover { background: #8f3420; }
    .toolbar-actions { display: flex; gap: 8px; justify-content: flex-end; flex-wrap: wrap; }
    .joint-list { display: grid; gap: 9px; }
    .joint {
      display: grid;
      grid-template-columns: 150px 90px minmax(180px, 1fr) 78px 46px 46px 82px;
      align-items: center;
      gap: 10px;
      padding: 11px 12px;
      background: var(--panel);
      border: 1px solid var(--line);
      border-left: 5px solid var(--accent);
      border-radius: 5px;
    }
    .joint-name strong { display: block; font-size: 16px; }
    .joint-name span { color: var(--muted); font-size: 12px; }
    .current { font-variant-numeric: tabular-nums; }
    .current b { display: block; color: var(--ok); font-size: 18px; }
    .current span { color: var(--muted); font-size: 11px; }
    input[type="range"] { width: 100%; accent-color: var(--accent); }
    .pwm-input { margin: 0 !important; text-align: center; padding: 0 4px !important; }
    .icon-button { min-width: 42px; padding: 0; font-size: 22px; line-height: 1; }
    .status-bar {
      margin-top: 14px;
      min-height: 42px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      padding: 9px 12px;
      background: #fff;
      border: 1px solid var(--line);
      border-radius: 5px;
      color: var(--muted);
      font-size: 13px;
    }
    .message { color: var(--ink); }
    .message.error { color: var(--warn); font-weight: 700; }
    @media (max-width: 900px) {
      .toolbar { grid-template-columns: 1fr 1fr; }
      .toolbar-actions { grid-column: 1 / -1; justify-content: stretch; }
      .toolbar-actions button { flex: 1; }
      .joint { grid-template-columns: 105px 70px 1fr 72px 42px 42px; }
      .joint .run { grid-column: 1 / -1; }
    }
    @media (max-width: 620px) {
      .header-inner { align-items: flex-start; flex-direction: column; }
      .toolbar { grid-template-columns: 1fr 1fr; padding: 10px; }
      .joint {
        grid-template-columns: 1fr 86px 42px 42px;
        gap: 8px;
      }
      .joint-name { grid-column: 1 / 3; }
      .current { grid-column: 3 / 5; text-align: right; }
      .joint input[type="range"] { grid-column: 1 / -1; }
      .joint .run { grid-column: 1 / -1; }
      .status-bar { align-items: flex-start; flex-direction: column; }
    }
  </style>
</head>
<body>
  <header>
    <div class="header-inner">
      <div>
        <h1>F103 六轴机械臂手动控制</h1>
        <div class="subtitle">仅控制 S0-S5，不占用串口，不发送底盘指令</div>
      </div>
      <div class="connection"><span id="dot" class="dot"></span><span id="connection">正在连接</span></div>
    </div>
  </header>
  <main>
    <section class="toolbar">
      <label>点动步长 PWM
        <input id="step" type="number" min="1" max="300" value="20">
      </label>
      <label>动作时间 ms
        <input id="duration" type="number" min="20" max="9999" value="800">
      </label>
      <div></div>
      <div class="toolbar-actions">
        <button id="sync" class="secondary">同步实际值</button>
        <button id="home" class="secondary">回 Home</button>
        <button id="all">六轴同步执行</button>
        <button id="stop" class="stop">急停机械臂</button>
      </div>
    </section>
    <section id="joints" class="joint-list"></section>
    <div class="status-bar">
      <span id="message" class="message">等待 F103 状态</span>
      <span id="detail">/arm_cmd · /f103_state</span>
    </div>
  </main>
  <script>
    const names = ["底座旋转", "肩关节", "肘关节", "腕部俯仰", "工具旋转", "夹爪"];
    const dirty = [false, false, false, false, false, false];
    const joints = document.getElementById("joints");

    function clamp(value, low, high) { return Math.max(low, Math.min(high, Number(value))); }
    function duration() { return clamp(document.getElementById("duration").value, 20, 9999); }
    function step() { return clamp(document.getElementById("step").value, 1, 300); }
    function setMessage(text, error=false) {
      const el = document.getElementById("message");
      el.textContent = text;
      el.className = error ? "message error" : "message";
    }

    async function post(path, body={}) {
      const response = await fetch(path, {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(body)
      });
      const data = await response.json();
      if (!response.ok || !data.ok) throw new Error(data.error || `HTTP ${response.status}`);
      return data;
    }

    function makeJoint(index) {
      const row = document.createElement("article");
      row.className = "joint";
      row.innerHTML = `
        <div class="joint-name"><strong>S${index}</strong><span>${names[index]}</span></div>
        <div class="current"><b id="current-${index}">--</b><span>当前 PWM</span></div>
        <input id="range-${index}" type="range" min="500" max="2500" step="1" value="1500" aria-label="S${index} PWM">
        <input id="pwm-${index}" class="pwm-input" type="number" min="500" max="2500" value="1500" aria-label="S${index} 目标 PWM">
        <button class="secondary icon-button" data-jog="-${index}" title="减小 PWM">−</button>
        <button class="secondary icon-button" data-jog="+${index}" title="增大 PWM">+</button>
        <button class="run" data-run="${index}">执行</button>`;
      const range = row.querySelector(`#range-${index}`);
      const input = row.querySelector(`#pwm-${index}`);
      range.addEventListener("input", () => { input.value = range.value; dirty[index] = true; });
      input.addEventListener("input", () => { range.value = clamp(input.value, 500, 2500); dirty[index] = true; });
      return row;
    }
    for (let i = 0; i < 6; i++) joints.appendChild(makeJoint(i));

    joints.addEventListener("click", async (event) => {
      const jog = event.target.dataset.jog;
      const run = event.target.dataset.run;
      try {
        if (jog) {
          const index = Number(jog.slice(1));
          const delta = (jog[0] === "+" ? 1 : -1) * step();
          const data = await post("/api/jog", {servo: index, delta, duration_ms: duration()});
          document.getElementById(`pwm-${index}`).value = data.target;
          document.getElementById(`range-${index}`).value = data.target;
          dirty[index] = false;
          setMessage(`S${index} -> ${data.target} PWM`);
        } else if (run !== undefined) {
          const index = Number(run);
          const pwm = clamp(document.getElementById(`pwm-${index}`).value, 500, 2500);
          await post("/api/move", {servo: index, pwm, duration_ms: duration()});
          dirty[index] = false;
          setMessage(`已发送 S${index} -> ${pwm} PWM`);
        }
      } catch (error) { setMessage(error.message, true); }
    });

    document.getElementById("all").addEventListener("click", async () => {
      try {
        const pwm = Array.from({length: 6}, (_, i) => clamp(document.getElementById(`pwm-${i}`).value, 500, 2500));
        await post("/api/move-all", {pwm, duration_ms: duration()});
        dirty.fill(false);
        setMessage(`已发送六轴目标 [${pwm.join(", ")}]`);
      } catch (error) { setMessage(error.message, true); }
    });
    document.getElementById("sync").addEventListener("click", async () => {
      try { await post("/api/sync"); dirty.fill(false); setMessage("目标已同步为 F103 实际 PWM"); }
      catch (error) { setMessage(error.message, true); }
    });
    document.getElementById("home").addEventListener("click", async () => {
      if (!confirm("机械臂将移动到 Home [1640,1500,1500,2250,1500,1500]，确认继续？")) return;
      try { await post("/api/home", {duration_ms: Math.max(1200, duration())}); dirty.fill(false); setMessage("正在回 Home"); }
      catch (error) { setMessage(error.message, true); }
    });
    document.getElementById("stop").addEventListener("click", async () => {
      try { await post("/api/stop"); setMessage("已发送机械臂急停 #255PDST!", true); }
      catch (error) { setMessage(error.message, true); }
    });

    async function poll() {
      try {
        const response = await fetch("/api/status", {cache: "no-store"});
        const data = await response.json();
        const online = data.bridge_subscribers > 0 && data.state_fresh;
        document.getElementById("dot").className = online ? "dot ok" : "dot";
        document.getElementById("connection").textContent = online ? "F103 在线" : "F103 状态不可用";
        for (let i = 0; i < 6; i++) {
          document.getElementById(`current-${i}`).textContent = data.current_pwm ? data.current_pwm[i] : "--";
          if (!dirty[i] && data.target_pwm) {
            document.getElementById(`pwm-${i}`).value = data.target_pwm[i];
            document.getElementById(`range-${i}`).value = data.target_pwm[i];
          }
        }
        document.getElementById("detail").textContent = `状态 ${data.state_age_s === null ? "--" : data.state_age_s.toFixed(2) + "s"} · 队列 ${data.queue_depth} · ${data.last_result || "等待命令"}`;
      } catch (error) {
        document.getElementById("dot").className = "dot";
        document.getElementById("connection").textContent = "控制脚本断开";
      }
    }
    poll();
    setInterval(poll, 300);
  </script>
</body>
</html>
"""


def clamp_int(value: Any, low: int, high: int, name: str) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be an integer") from exc
    if not low <= parsed <= high:
        raise ValueError(f"{name} must be between {low} and {high}")
    return parsed


def build_group_command(targets: dict[int, int], duration_ms: int) -> str:
    duration = clamp_int(duration_ms, MIN_DURATION_MS, MAX_DURATION_MS, "duration_ms")
    if not targets:
        raise ValueError("at least one servo target is required")
    parts = []
    for servo, pwm in sorted(targets.items()):
        servo_id = clamp_int(servo, 0, SERVO_COUNT - 1, "servo")
        pwm_value = clamp_int(pwm, MIN_PWM, MAX_PWM, "pwm")
        parts.append(f"#{servo_id:03d}P{pwm_value:04d}T{duration:04d}!")
    return parts[0] if len(parts) == 1 else "{" + "".join(parts) + "}"


@dataclass(frozen=True)
class PendingCommand:
    payload: str
    reason: str


class ArmManualNode(Node):
    def __init__(self, arm_topic: str, state_topic: str, min_publish_interval_s: float):
        super().__init__("arm_manual_control")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._arm_topic = arm_topic
        self._state_topic = state_topic
        self._publisher = self.create_publisher(RosString, arm_topic, qos)
        self._subscription = self.create_subscription(RosString, state_topic, self._on_state, qos)
        self._commands: queue.Queue[PendingCommand] = queue.Queue(maxsize=64)
        self._lock = threading.RLock()
        self._current_pwm: list[int] | None = None
        self._target_pwm: list[int] = list(HOME_PWM)
        self._last_state_at = 0.0
        self._last_publish_at = 0.0
        self._last_result = ""
        self._state_mode = ""
        self._state_estop = 0
        self._min_publish_interval_s = max(0.1, float(min_publish_interval_s))
        self._timer = self.create_timer(0.02, self._publish_next)
        self.get_logger().info(
            f"manual arm control ready: {arm_topic} -> bridge, feedback from {state_topic}"
        )

    def _on_state(self, msg: RosString) -> None:
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        raw_state = str(data.get("last_state", ""))
        state_fields: dict[str, str] = {}
        state_body = raw_state[len("@STATE:") :] if raw_state.startswith("@STATE:") else raw_state
        if state_body.endswith("!"):
            state_body = state_body[:-1]
        for part in state_body.split(","):
            if "=" in part:
                key, value = part.split("=", 1)
                state_fields[key.strip()] = value.strip()

        values = data.get("servo_pwm")
        if not isinstance(values, list) or len(values) < SERVO_COUNT:
            parsed = {int(index): int(value) for index, value in STATE_PWM_RE.findall(raw_state)}
            if len(parsed) == SERVO_COUNT:
                values = [parsed[index] for index in range(SERVO_COUNT)]
        if not isinstance(values, list) or len(values) < SERVO_COUNT:
            return
        try:
            current = [clamp_int(values[index], MIN_PWM, MAX_PWM, f"S{index}") for index in range(SERVO_COUNT)]
        except ValueError:
            return
        now = time.monotonic()
        with self._lock:
            first_state = self._current_pwm is None
            self._current_pwm = current
            self._last_state_at = now
            self._state_mode = str(data.get("mode") or state_fields.get("MODE", ""))
            self._state_estop = int(data.get("estop", state_fields.get("ESTOP", 0)) or 0)
            if first_state:
                self._target_pwm = list(current)

    def _require_motion_ready(self) -> None:
        with self._lock:
            state_age = None if self._last_state_at <= 0.0 else time.monotonic() - self._last_state_at
            estop = self._state_estop
        if self.count_subscribers(self._arm_topic) <= 0:
            raise RuntimeError("F103 bridge is not subscribed to /arm_cmd")
        if state_age is None or state_age > 1.0:
            raise RuntimeError("F103 servo state is unavailable or stale")
        if estop:
            raise RuntimeError("F103 ESTOP is active")

    def _clear_commands(self) -> None:
        while True:
            try:
                self._commands.get_nowait()
            except queue.Empty:
                return

    def enqueue(self, payload: str, reason: str, target_updates: dict[int, int] | None = None) -> None:
        if target_updates:
            with self._lock:
                for servo, pwm in target_updates.items():
                    self._target_pwm[int(servo)] = int(pwm)
        try:
            self._commands.put_nowait(PendingCommand(payload=payload, reason=reason))
        except queue.Full as exc:
            raise RuntimeError("command queue is full; wait for the current motion commands") from exc

    def emergency_stop(self) -> None:
        self._clear_commands()
        self.enqueue("#255PDST!", "arm emergency stop")

    def _publish_next(self) -> None:
        now = time.monotonic()
        if now - self._last_publish_at < self._min_publish_interval_s:
            return
        try:
            pending = self._commands.get_nowait()
        except queue.Empty:
            return
        message = RosString()
        message.data = pending.payload
        self._publisher.publish(message)
        self._last_publish_at = now
        with self._lock:
            self._last_result = f"{pending.reason}: {pending.payload}"
        self.get_logger().info(self._last_result)

    def move(self, servo: int, pwm: int, duration_ms: int) -> int:
        self._require_motion_ready()
        servo_id = clamp_int(servo, 0, SERVO_COUNT - 1, "servo")
        pwm_value = clamp_int(pwm, MIN_PWM, MAX_PWM, "pwm")
        payload = build_group_command({servo_id: pwm_value}, duration_ms)
        self.enqueue(payload, f"S{servo_id} move", {servo_id: pwm_value})
        return pwm_value

    def jog(self, servo: int, delta: int, duration_ms: int) -> int:
        servo_id = clamp_int(servo, 0, SERVO_COUNT - 1, "servo")
        delta_value = clamp_int(delta, -300, 300, "delta")
        if delta_value == 0:
            raise ValueError("delta must not be zero")
        with self._lock:
            base = self._target_pwm[servo_id]
        target = max(MIN_PWM, min(MAX_PWM, base + delta_value))
        return self.move(servo_id, target, duration_ms)

    def move_all(self, values: list[Any], duration_ms: int, reason: str = "six-axis move") -> list[int]:
        self._require_motion_ready()
        if not isinstance(values, list) or len(values) != SERVO_COUNT:
            raise ValueError("pwm must contain exactly six values")
        normalized = [clamp_int(value, MIN_PWM, MAX_PWM, f"S{index}") for index, value in enumerate(values)]
        targets = dict(enumerate(normalized))
        self.enqueue(build_group_command(targets, duration_ms), reason, targets)
        return normalized

    def sync_targets(self) -> list[int]:
        with self._lock:
            if self._current_pwm is None:
                raise RuntimeError("F103 servo state is not available")
            self._target_pwm = list(self._current_pwm)
            self._last_result = "targets synchronized from F103 state"
            return list(self._target_pwm)

    def status(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            age = None if self._last_state_at <= 0.0 else max(0.0, now - self._last_state_at)
            return {
                "ok": True,
                "current_pwm": None if self._current_pwm is None else list(self._current_pwm),
                "target_pwm": list(self._target_pwm),
                "home_pwm": list(HOME_PWM),
                "state_age_s": None if age is None else round(age, 3),
                "state_fresh": age is not None and age <= 1.0,
                "bridge_subscribers": self.count_subscribers(self._arm_topic),
                "queue_depth": self._commands.qsize(),
                "last_result": self._last_result,
                "mode": self._state_mode,
                "estop": self._state_estop,
            }


class ArmControlHandler(BaseHTTPRequestHandler):
    node: ArmManualNode
    server_version = "ArmManualControl/1.0"

    def log_message(self, fmt: str, *args: Any) -> None:
        self.node.get_logger().debug("http " + fmt % args)

    def _send_json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(int(status))
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict[str, Any]:
        length = clamp_int(self.headers.get("Content-Length", "0"), 0, 8192, "Content-Length")
        raw = self.rfile.read(length)
        if not raw:
            return {}
        data = json.loads(raw.decode("utf-8"))
        if not isinstance(data, dict):
            raise ValueError("request body must be a JSON object")
        return data

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path == "/":
            body = HTML_PAGE.encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)
            return
        if path == "/api/status":
            self._send_json(HTTPStatus.OK, self.node.status())
            return
        self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        try:
            data = self._read_json()
            if path == "/api/move":
                target = self.node.move(data.get("servo"), data.get("pwm"), data.get("duration_ms", 800))
                result = {"ok": True, "target": target}
            elif path == "/api/jog":
                target = self.node.jog(data.get("servo"), data.get("delta"), data.get("duration_ms", 800))
                result = {"ok": True, "target": target}
            elif path == "/api/move-all":
                targets = self.node.move_all(data.get("pwm"), data.get("duration_ms", 800))
                result = {"ok": True, "target_pwm": targets}
            elif path == "/api/home":
                targets = self.node.move_all(list(HOME_PWM), data.get("duration_ms", 2000), "home")
                result = {"ok": True, "target_pwm": targets}
            elif path == "/api/sync":
                result = {"ok": True, "target_pwm": self.node.sync_targets()}
            elif path == "/api/stop":
                self.node.emergency_stop()
                result = {"ok": True}
            else:
                self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})
                return
            self._send_json(HTTPStatus.OK, result)
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
        except RuntimeError as exc:
            self._send_json(HTTPStatus.CONFLICT, {"ok": False, "error": str(exc)})
        except Exception as exc:  # pragma: no cover - defensive HTTP boundary
            self.node.get_logger().error(f"manual arm HTTP request failed: {exc}")
            self._send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "error": str(exc)})


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Browser-based manual controller for the F103 six-servo arm")
    parser.add_argument("--bind", default="0.0.0.0", help="HTTP bind address")
    parser.add_argument("--port", type=int, default=8766, help="HTTP port")
    parser.add_argument("--arm-topic", default="/arm_cmd")
    parser.add_argument("--state-topic", default="/f103_state")
    parser.add_argument("--min-publish-interval", type=float, default=0.12)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    rclpy.init(args=None)
    node = ArmManualNode(args.arm_topic, args.state_topic, args.min_publish_interval)
    ArmControlHandler.node = node
    server = ThreadingHTTPServer((args.bind, args.port), ArmControlHandler)
    server_thread = threading.Thread(target=server.serve_forever, name="arm-manual-http", daemon=True)
    server_thread.start()
    node.get_logger().info(f"open http://{args.bind}:{args.port}/ in a browser")

    stopping = threading.Event()

    def stop_handler(_signum: int, _frame: Any) -> None:
        stopping.set()

    signal.signal(signal.SIGTERM, stop_handler)
    signal.signal(signal.SIGINT, stop_handler)
    try:
        while rclpy.ok() and not stopping.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
