#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT_DIR"

if [ -f cloud_platform.pid ]; then
  PID="$(cat cloud_platform.pid || true)"
  if [ -n "$PID" ] && kill -0 "$PID" 2>/dev/null; then
    kill "$PID"
    sleep 1
  fi
  rm -f cloud_platform.pid
fi

pkill -f "$ROOT_DIR/.venv/bin/python $ROOT_DIR/app.py" 2>/dev/null || true
