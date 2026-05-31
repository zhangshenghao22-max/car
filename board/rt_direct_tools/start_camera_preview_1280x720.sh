#!/usr/bin/env bash
set -euo pipefail

DEVICE="${DEVICE:-/dev/video0}"
FORMAT="${FORMAT:-mjpg}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
FPS="${FPS:-15}"
SINK="${SINK:-autovideosink}"
ROTATE="${ROTATE:-180}"
DISPLAY_WIDTH="${DISPLAY_WIDTH:-}"
DISPLAY_HEIGHT="${DISPLAY_HEIGHT:-}"
SCALE_ENABLED=1
SHOW_FPS_OVERLAY=1
DRY_RUN=0
LIST_CAPS=0

usage() {
  cat <<'EOF'
Usage:
  bash ./start_camera_preview_1280x720.sh [options]

Options:
  --device DEV          Video device, default: /dev/video0
  --format FORMAT      mjpg or yuyv, default: mjpg
  --width PX           Frame width, default: 1280
  --height PX          Frame height, default: 720
  --fps FPS            Requested frame rate, default: 15
  --sink SINK          GStreamer video sink, default: autovideosink
  --rotate MODE        none, 180, 90, 270, hflip, or vflip, default: 180
  --display-width PX   Scale preview output width before display
  --display-height PX  Scale preview output height before display
  --no-scale           Disable preview scaling even if display size is set
  --no-fps-overlay     Disable FPS text overlay
  --dry-run            Print the GStreamer command and exit
  --list-caps          List camera capabilities and exit
  -h, --help           Show this help

Environment defaults are also supported:
  DEVICE=/dev/video0 FORMAT=mjpg WIDTH=1280 HEIGHT=720 FPS=15 SINK=autovideosink ROTATE=180
  DISPLAY_WIDTH=1280 DISPLAY_HEIGHT=720

Notes:
  The current icSpring camera high-resolution modes are MJPG 15fps.
  For high-resolution capture, scale before display, for example:
    --width 2048 --height 1536 --display-width 1280 --display-height 720
  If startup fails with not-negotiated, the requested caps may be unsupported by the camera or sink.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --device)
      DEVICE="${2:?missing value for --device}"
      shift 2
      ;;
    --format)
      FORMAT="${2:?missing value for --format}"
      shift 2
      ;;
    --width)
      WIDTH="${2:?missing value for --width}"
      shift 2
      ;;
    --height)
      HEIGHT="${2:?missing value for --height}"
      shift 2
      ;;
    --fps)
      FPS="${2:?missing value for --fps}"
      shift 2
      ;;
    --sink)
      SINK="${2:?missing value for --sink}"
      shift 2
      ;;
    --rotate)
      ROTATE="${2:?missing value for --rotate}"
      shift 2
      ;;
    --display-width)
      DISPLAY_WIDTH="${2:?missing value for --display-width}"
      shift 2
      ;;
    --display-height)
      DISPLAY_HEIGHT="${2:?missing value for --display-height}"
      shift 2
      ;;
    --no-scale)
      SCALE_ENABLED=0
      shift
      ;;
    --no-fps-overlay)
      SHOW_FPS_OVERLAY=0
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    --list-caps)
      LIST_CAPS=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${FORMAT,,}" in
  mjpg|mjpeg)
    CAPS="image/jpeg,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1"
    DECODE=(jpegdec)
    ;;
  yuyv|yuy2)
    CAPS="video/x-raw,format=YUY2,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1"
    DECODE=()
    ;;
  *)
    echo "ERROR: --format must be mjpg or yuyv, got: ${FORMAT}" >&2
    exit 2
    ;;
esac

case "${ROTATE,,}" in
  none|0)
    FLIP_METHOD=""
    ;;
  180|rotate-180)
    FLIP_METHOD="rotate-180"
    ;;
  90|clockwise)
    FLIP_METHOD="clockwise"
    ;;
  270|counterclockwise|counter-clockwise)
    FLIP_METHOD="counterclockwise"
    ;;
  hflip|horizontal|horizontal-flip)
    FLIP_METHOD="horizontal-flip"
    ;;
  vflip|vertical|vertical-flip)
    FLIP_METHOD="vertical-flip"
    ;;
  *)
    echo "ERROR: --rotate must be none, 180, 90, 270, hflip, or vflip; got: ${ROTATE}" >&2
    exit 2
    ;;
esac

if [[ "${SCALE_ENABLED}" -eq 1 ]]; then
  if [[ -n "${DISPLAY_WIDTH}" && -z "${DISPLAY_HEIGHT}" ]]; then
    echo "ERROR: --display-height is required when --display-width is set" >&2
    exit 2
  fi
  if [[ -z "${DISPLAY_WIDTH}" && -n "${DISPLAY_HEIGHT}" ]]; then
    echo "ERROR: --display-width is required when --display-height is set" >&2
    exit 2
  fi
fi

if [[ "${LIST_CAPS}" -eq 1 ]]; then
  if command -v gst-device-monitor-1.0 >/dev/null 2>&1; then
    gst-device-monitor-1.0 Video/Source
  else
    echo "ERROR: gst-device-monitor-1.0 not found" >&2
    exit 127
  fi
  exit 0
fi

PIPELINE=(gst-launch-1.0 v4l2src "device=${DEVICE}" ! "${CAPS}" !)
if [[ "${#DECODE[@]}" -gt 0 ]]; then
  PIPELINE+=("${DECODE[@]}" !)
fi
PIPELINE+=(videoconvert !)
if [[ -n "${FLIP_METHOD}" ]]; then
  PIPELINE+=(videoflip "method=${FLIP_METHOD}" !)
fi
if [[ "${SCALE_ENABLED}" -eq 1 && -n "${DISPLAY_WIDTH}" && -n "${DISPLAY_HEIGHT}" ]]; then
  PIPELINE+=(videoscale ! "video/x-raw,width=${DISPLAY_WIDTH},height=${DISPLAY_HEIGHT}" !)
fi

if [[ "${SHOW_FPS_OVERLAY}" -eq 1 ]]; then
  PIPELINE+=(fpsdisplaysink "video-sink=${SINK}" text-overlay=true sync=false)
else
  PIPELINE+=("${SINK}" sync=false)
fi

if [[ "${SCALE_ENABLED}" -eq 1 && -n "${DISPLAY_WIDTH}" && -n "${DISPLAY_HEIGHT}" ]]; then
  DISPLAY_DESC="${DISPLAY_WIDTH}x${DISPLAY_HEIGHT}"
else
  DISPLAY_DESC="source-size"
fi

echo "Camera preview: device=${DEVICE} format=${FORMAT,,} capture=${WIDTH}x${HEIGHT}@${FPS}fps display=${DISPLAY_DESC} sink=${SINK} rotate=${ROTATE,,}" >&2
echo "If startup fails with not-negotiated, the requested caps or direct display size may be unsupported." >&2

if [[ "${DRY_RUN}" -eq 1 ]]; then
  printf '%q ' "${PIPELINE[@]}"
  printf '\n'
  exit 0
fi

if [[ ! -e "${DEVICE}" ]]; then
  echo "ERROR: video device not found: ${DEVICE}" >&2
  exit 2
fi

exec "${PIPELINE[@]}"
