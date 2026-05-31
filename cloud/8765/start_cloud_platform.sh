#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT_DIR"

if [ -f cloud_platform.pid ] && kill -0 "$(cat cloud_platform.pid)" 2>/dev/null; then
  echo "cloud platform already running: $(cat cloud_platform.pid)"
  exit 0
fi

export CAR_CLOUD_HOST="${CAR_CLOUD_HOST:-0.0.0.0}"
export CAR_CLOUD_PORT="${CAR_CLOUD_PORT:-8765}"
export CAR_CLOUD_UPLOAD_TOKEN="${CAR_CLOUD_UPLOAD_TOKEN:-car-cloud-upload}"
export CAR_CLOUD_COMMAND_TOKEN="${CAR_CLOUD_COMMAND_TOKEN:-${CAR_CLOUD_UPLOAD_TOKEN}}"

if [ ! -x "$ROOT_DIR/.venv/bin/python" ]; then
  python3 -m venv "$ROOT_DIR/.venv"
fi

if ! "$ROOT_DIR/.venv/bin/python" -c "import flask, requests" >/dev/null 2>&1; then
  "$ROOT_DIR/.venv/bin/python" -m pip install --upgrade pip
  "$ROOT_DIR/.venv/bin/python" -m pip install -r "$ROOT_DIR/requirements.txt"
fi

nohup "$ROOT_DIR/.venv/bin/python" "$ROOT_DIR/app.py" > "$ROOT_DIR/cloud_platform.log" 2>&1 &
echo $! > "$ROOT_DIR/cloud_platform.pid"
sleep 2
cat "$ROOT_DIR/cloud_platform.pid"
