#!/usr/bin/env python3
from __future__ import annotations

import argparse
import contextlib
import os
import select
import sys
import threading
import time

import serial

try:
    import termios
    import tty
except Exception:  # pragma: no cover - Windows fallback only
    termios = None
    tty = None

try:
    import msvcrt
except Exception:  # pragma: no cover - Linux runtime path
    msvcrt = None


DEFAULT_PORT = "/dev/serial/by-path/platform-fc8c0000.usb-usb-0:1:1.0-port0"
DEFAULT_BAUDRATE = 115200


HELP_LINES = [
    "========================================",
    "F103 USB Twist Panel",
    "========================================",
    "w/s : Increase/Decrease VX (forward/backward)",
    "a/d : Increase/Decrease VY (left/right)",
    "q/e : Increase/Decrease WZ (rotate left/right)",
    "x/r : Stop all movement",
    "p   : Send $PING!",
    "t   : Send $STATUS!",
    "o   : Enable @ODOM report",
    "c   : Disable @ODOM/@STATE report",
    "i   : Print current velocity",
    "h   : Print help",
    "Ctrl+C : Stop and exit",
    "========================================",
]


class KeyReader:
    def __enter__(self):
        self._fd = None
        self._old_termios = None
        if os.name == "nt" and msvcrt is not None:
            return self
        if termios is None or tty is None:
            raise RuntimeError("raw terminal input is not available")
        if not sys.stdin.isatty():
            raise RuntimeError("stdin is not a TTY")
        self._fd = sys.stdin.fileno()
        self._old_termios = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._fd is not None and self._old_termios is not None:
            with contextlib.suppress(Exception):
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_termios)

    def read_key(self, timeout_s: float = 0.05) -> str:
        if os.name == "nt" and msvcrt is not None:
            deadline = time.time() + timeout_s
            while time.time() < deadline:
                if msvcrt.kbhit():
                    return msvcrt.getwch()
                time.sleep(0.01)
            return ""
        if self._fd is None:
            return ""
        ready, _, _ = select.select([self._fd], [], [], timeout_s)
        if not ready:
            return ""
        return os.read(self._fd, 8).decode("utf-8", errors="ignore")


class F103TwistPanel:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.serial = serial.Serial(
            port=args.port,
            baudrate=args.baudrate,
            timeout=0.05,
            write_timeout=0.5,
        )
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.seq = int(time.time()) % 900000
        self.vx = 0
        self.vy = 0
        self.wz = 0
        self.last_sent_zero = False
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)

    def start(self, *, tx_loop: bool = True):
        with contextlib.suppress(Exception):
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
        self.rx_thread.start()
        if tx_loop:
            self.tx_thread.start()

    def close(self):
        self.stop_event.set()
        self.stop_motion()
        if self.tx_thread.is_alive():
            with contextlib.suppress(Exception):
                self.tx_thread.join(timeout=0.5)
        with contextlib.suppress(Exception):
            self.rx_thread.join(timeout=0.5)
        with contextlib.suppress(Exception):
            self.serial.close()

    def _rx_loop(self):
        buffer = b""
        while not self.stop_event.is_set():
            try:
                chunk = self.serial.read(256)
            except Exception as exc:
                print(f"\n[RX ERROR] {exc}", flush=True)
                time.sleep(0.2)
                continue
            if not chunk:
                continue
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                text = line.decode("utf-8", errors="replace").rstrip("\r")
                if text:
                    print(f"\nRX: {text}", flush=True)
            if len(buffer) > 512:
                text = buffer.decode("utf-8", errors="replace")
                print(f"\nRX_RAW: {text}", flush=True)
                buffer = b""

    def _tx_loop(self):
        interval = 1.0 / max(1.0, float(self.args.rate_hz))
        while not self.stop_event.is_set():
            with self.lock:
                vx, vy, wz = self.vx, self.vy, self.wz
            if vx != 0 or vy != 0 or wz != 0 or not self.last_sent_zero:
                self.send_twist(vx, vy, wz, show=False)
                self.last_sent_zero = vx == 0 and vy == 0 and wz == 0
            time.sleep(interval)

    def _write(self, command: str, *, show: bool = True):
        command = command.strip()
        if not command:
            return
        payload = command.encode("ascii", errors="ignore")
        with self.lock:
            self.serial.write(payload)
            self.serial.flush()
        if show:
            print(f"TX: {command}", flush=True)

    def send_raw(self, command: str):
        self._write(command)

    def send_twist(self, vx: int, vy: int, wz: int, *, show: bool = True):
        self.seq = (self.seq % 999999) + 1
        self._write(f"$TWIST:SEQ={self.seq},VX={vx},VY={vy},WZ={wz}!", show=show)

    def set_velocity(self, vx: int | None = None, vy: int | None = None, wz: int | None = None):
        with self.lock:
            if vx is not None:
                self.vx = self._clamp(vx, -self.args.max_linear_mm_s, self.args.max_linear_mm_s)
            if vy is not None:
                self.vy = self._clamp(vy, -self.args.max_lateral_mm_s, self.args.max_lateral_mm_s)
            if wz is not None:
                self.wz = self._clamp(wz, -self.args.max_angular_mrad_s, self.args.max_angular_mrad_s)
            self.last_sent_zero = False
            current = (self.vx, self.vy, self.wz)
        print(f"VEL: VX={current[0]} mm/s, VY={current[1]} mm/s, WZ={current[2]} mrad/s", flush=True)

    def stop_motion(self):
        self.set_velocity(0, 0, 0)
        self.send_twist(0, 0, 0)

    @staticmethod
    def _clamp(value: int, low: int, high: int) -> int:
        return max(low, min(high, int(value)))

    def process_key(self, key: str):
        if not key:
            return
        key = key[0]
        if key in ("h", "H"):
            print_help()
        elif key in ("i", "I"):
            with self.lock:
                print(f"VEL: VX={self.vx} mm/s, VY={self.vy} mm/s, WZ={self.wz} mrad/s", flush=True)
        elif key in ("w", "W"):
            self.set_velocity(vx=self.vx + self.args.linear_step_mm_s)
        elif key in ("s", "S"):
            self.set_velocity(vx=self.vx - self.args.linear_step_mm_s)
        elif key in ("a", "A"):
            self.set_velocity(vy=self.vy + self.args.linear_step_mm_s)
        elif key in ("d", "D"):
            self.set_velocity(vy=self.vy - self.args.linear_step_mm_s)
        elif key in ("q", "Q"):
            self.set_velocity(wz=self.wz + self.args.angular_step_mrad_s)
        elif key in ("e", "E"):
            self.set_velocity(wz=self.wz - self.args.angular_step_mrad_s)
        elif key in ("x", "X", "r", "R"):
            self.stop_motion()
        elif key in ("p", "P"):
            self.send_raw("$PING!")
        elif key in ("t", "T"):
            self.send_raw("$STATUS!")
        elif key in ("o", "O"):
            self.send_raw("$REPORT:ON!")
            self.send_raw("$ODOM:ON!")
        elif key in ("c", "C"):
            self.send_raw("$REPORT:OFF!")
            self.send_raw("$ODOM:OFF!")


def print_help():
    print("")
    for line in HELP_LINES:
        print(line)
    print("")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Direct USB serial speed controller for F103 firmware.")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--linear-step-mm-s", type=int, default=100)
    parser.add_argument("--angular-step-mrad-s", type=int, default=100)
    parser.add_argument("--max-linear-mm-s", type=int, default=900)
    parser.add_argument("--max-lateral-mm-s", type=int, default=900)
    parser.add_argument("--max-angular-mrad-s", type=int, default=900)
    parser.add_argument("--ping", action="store_true", help="send $PING! and exit")
    parser.add_argument("--status", action="store_true", help="send $STATUS! and exit")
    parser.add_argument("--raw", action="append", default=[], help="send raw command and exit; can be repeated")
    parser.add_argument("--twist", nargs=3, type=int, metavar=("VX", "VY", "WZ"), help="send twist in mm/s, mm/s, mrad/s")
    parser.add_argument("--duration", type=float, default=0.0, help="repeat --twist for this many seconds, then stop")
    return parser


def run_one_shot(panel: F103TwistPanel, args: argparse.Namespace) -> bool:
    used = False
    if args.ping:
        panel.send_raw("$PING!")
        used = True
    if args.status:
        panel.send_raw("$STATUS!")
        used = True
    for command in args.raw:
        panel.send_raw(command)
        used = True
    if args.twist is not None:
        vx, vy, wz = args.twist
        duration = max(0.0, float(args.duration))
        if duration <= 0.0:
            panel.send_twist(vx, vy, wz)
        else:
            deadline = time.time() + duration
            interval = 1.0 / max(1.0, float(args.rate_hz))
            while time.time() < deadline:
                panel.send_twist(vx, vy, wz)
                time.sleep(interval)
            panel.send_twist(0, 0, 0)
        used = True
    if used:
        time.sleep(1.0)
    return used


def has_one_shot_args(args: argparse.Namespace) -> bool:
    return bool(args.ping or args.status or args.raw or args.twist is not None)


def main() -> int:
    args = build_arg_parser().parse_args()
    panel = F103TwistPanel(args)
    panel.start(tx_loop=not has_one_shot_args(args))
    try:
        if run_one_shot(panel, args):
            return 0
        print_help()
        panel.send_raw("$PING!")
        panel.send_raw("$STATUS!")
        with KeyReader() as key_reader:
            while True:
                panel.process_key(key_reader.read_key(timeout_s=0.05))
                time.sleep(0.01)
    except KeyboardInterrupt:
        return 0
    finally:
        panel.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
