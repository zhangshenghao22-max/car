#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_PROBE="${ROOT_DIR}/ros2_tools/ros_probe.py"
RT_HELPER="${ROOT_DIR}/board_tools/rt_shell_run_commands.py"
STOP_SCRIPT="${ROOT_DIR}/stop_board_f103_arm_only.sh"

ROS_SETUP="/opt/ros/foxy/setup.bash"
ROS_WS_DIR="${HOME}/Desktop/rock_ws/ros_ws"
ROS_WS_SETUP="${ROS_WS_DIR}/install/setup.bash"
MICROROS_WS_DIR="${HOME}/Desktop/rock_ws/microros_ws"
MICROROS_WS_SETUP="${MICROROS_WS_DIR}/install/setup.bash"
MICROROS_AGENT_BIN="${MICROROS_WS_DIR}/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent"

RT_PORT="${CAR2_RT_PORT:-/dev/serial/by-path/platform-fc8c0000.usb-usb-0:1:1.0-port0}"
RT_BAUD="${CAR2_RT_BAUD:-1500000}"
F103_UART="${CAR2_F103_UART:-uart7}"
F103_BAUD="${CAR2_F103_BAUD:-115200}"
AGENT_IP="${CAR2_AGENT_IP:-10.10.20.1}"
AGENT_PORT="${CAR2_AGENT_PORT:-8888}"
ARM_SUB_TIMEOUT="12"
ODOM_TIMEOUT="8"
LOG_DIR=""
RT_PORT_STATE_FILE=""

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --rt-port PATH            RT shell serial port candidate.
  --rt-baud NUM             RT shell baudrate. Default: ${RT_BAUD}
  --f103-uart NAME          RT-side F103 UART. Default: ${F103_UART}
  --f103-baud NUM           F103 UART baudrate. Default: ${F103_BAUD}
  --agent-ip IP             Linux-side micro-ROS agent IP. Default: ${AGENT_IP}
  --agent-port NUM          micro-ROS agent UDP port. Default: ${AGENT_PORT}
  --arm-timeout SEC         Wait time for /arm_cmd subscribers. Default: ${ARM_SUB_TIMEOUT}
  --odom-timeout SEC        Wait time for /odom_raw sanity probe. Default: ${ODOM_TIMEOUT}
  -h, --help                Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --rt-port)
      RT_PORT="${2:?missing value for --rt-port}"
      shift 2
      ;;
    --rt-baud)
      RT_BAUD="${2:?missing value for --rt-baud}"
      shift 2
      ;;
    --f103-uart)
      F103_UART="${2:?missing value for --f103-uart}"
      shift 2
      ;;
    --f103-baud)
      F103_BAUD="${2:?missing value for --f103-baud}"
      shift 2
      ;;
    --agent-ip)
      AGENT_IP="${2:?missing value for --agent-ip}"
      shift 2
      ;;
    --agent-port)
      AGENT_PORT="${2:?missing value for --agent-port}"
      shift 2
      ;;
    --arm-timeout)
      ARM_SUB_TIMEOUT="${2:?missing value for --arm-timeout}"
      shift 2
      ;;
    --odom-timeout)
      ODOM_TIMEOUT="${2:?missing value for --odom-timeout}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

log() {
  if [[ -n "${LOG_DIR}" ]]; then
    printf '%s %s\n' "$(date -Is)" "$*" | tee -a "${LOG_DIR}/session.log"
  else
    printf '%s %s\n' "$(date -Is)" "$*"
  fi
}

fail() {
  log "ERROR: $*"
  exit 1
}

resolve_log_dir() {
  local candidates=()
  if [[ -n "${CAR2_F103_ARM_ONLY_LOG_DIR:-}" ]]; then
    candidates+=("${CAR2_F103_ARM_ONLY_LOG_DIR}")
  fi
  candidates+=(
    "${ROOT_DIR}/runtime/f103_arm_only"
    "${ROOT_DIR}/logs/f103_arm_only"
    "${HOME}/.car2_runtime_logs/f103_arm_only"
  )

  local candidate
  for candidate in "${candidates[@]}"; do
    if mkdir -p "${candidate}" 2>/dev/null; then
      LOG_DIR="${candidate}"
      return 0
    fi
  done

  echo "cannot create a writable log directory" >&2
  return 1
}

source_ros_env() {
  export ROS_LOCALHOST_ONLY=0
  unset LD_PRELOAD
  set +u
  source "${ROS_SETUP}" >/dev/null 2>&1
  source "${ROS_WS_SETUP}" >/dev/null 2>&1
  set -u
}

run_with_rt_port_access() {
  if [[ -r "${RT_PORT}" && -w "${RT_PORT}" ]]; then
    "$@"
    return $?
  fi

  if sudo -n true >/dev/null 2>&1; then
    sudo "$@"
    return $?
  fi

  if [[ -n "${CAR2_SUDO_PASSWORD:-}" ]]; then
    printf '%s\n' "${CAR2_SUDO_PASSWORD}" | sudo -S "$@"
    return $?
  fi

  if ! command -v sudo >/dev/null 2>&1; then
    fail "cannot access ${RT_PORT} and sudo is unavailable"
  fi

  sudo "$@"
}

kill_pid_file() {
  local pid_file="$1"
  [[ -f "${pid_file}" ]] || return 0
  local pid
  pid="$(cat "${pid_file}" 2>/dev/null || true)"
  if [[ -n "${pid}" ]]; then
    kill -- "-${pid}" 2>/dev/null || kill "${pid}" 2>/dev/null || true
    sleep 0.5
    kill -9 -- "-${pid}" 2>/dev/null || kill -9 "${pid}" 2>/dev/null || true
  fi
  rm -f "${pid_file}"
}

stop_processes_matching() {
  local pattern="$1"
  pkill -f -- "${pattern}" >/dev/null 2>&1 || true
  sleep 0.5
  pkill -9 -f -- "${pattern}" >/dev/null 2>&1 || true
}

append_unique_rt_port() {
  local candidate="${1:-}"
  [[ -n "${candidate}" ]] || return 0
  [[ -e "${candidate}" ]] || return 0
  case " ${_RT_PORT_CANDIDATES_SEEN:-} " in
    *" ${candidate} "*) return 0 ;;
  esac
  _RT_PORT_CANDIDATES_SEEN="${_RT_PORT_CANDIDATES_SEEN:-} ${candidate}"
  printf '%s\n' "${candidate}"
}

candidate_rt_ports() {
  _RT_PORT_CANDIDATES_SEEN=""
  local candidate=""

  if [[ -n "${RT_PORT_STATE_FILE}" && -f "${RT_PORT_STATE_FILE}" ]]; then
    candidate="$(head -n 1 "${RT_PORT_STATE_FILE}" 2>/dev/null || true)"
    append_unique_rt_port "${candidate}"
  fi

  append_unique_rt_port "${RT_PORT}"
  append_unique_rt_port "/dev/rt_shell"

  for candidate in \
    /dev/serial/by-id/* \
    /dev/serial/by-path/platform-fc8c0000.usb-usb-0:1:1.0-port0 \
    /dev/ttyUSB*; do
    append_unique_rt_port "${candidate}"
  done
}

probe_rt_shell_port() {
  local port="$1"
  local previous_rt_port="${RT_PORT}"
  RT_PORT="${port}"
  local helper_args=(
    "${RT_HELPER}"
    --port "${port}"
    --baud "${RT_BAUD}"
    --check-shell
    --wait-for-prompt
  )
  local rc=0
  if run_with_rt_port_access python3 "${helper_args[@]}"; then
    rc=0
  else
    rc=$?
  fi
  RT_PORT="${previous_rt_port}"
  return "${rc}"
}

refresh_rt_port() {
  local timeout_s="${1:-0}"
  local log_file="${2:-${LOG_DIR}/rt_shell_probe.log}"
  local start_ts now_ts candidate
  start_ts="$(date +%s)"
  while true; do
    while IFS= read -r candidate; do
      [[ -n "${candidate}" ]] || continue
      if probe_rt_shell_port "${candidate}" >"${log_file}" 2>&1; then
        RT_PORT="${candidate}"
        if [[ -n "${RT_PORT_STATE_FILE}" ]]; then
          printf '%s\n' "${RT_PORT}" >"${RT_PORT_STATE_FILE}"
        fi
        return 0
      fi
    done < <(candidate_rt_ports)

    now_ts="$(date +%s)"
    if (( now_ts - start_ts >= timeout_s )); then
      return 1
    fi
    sleep 1
  done
}

wait_for_rt_shell_ready() {
  local timeout_s="${1:?missing timeout}"
  if refresh_rt_port "${timeout_s}" "${LOG_DIR}/rt_shell_probe.log"; then
    log "RT shell ready on ${RT_PORT}"
    return 0
  fi
  return 1
}

pid_alive() {
  local pid_file="$1"
  [[ -f "${pid_file}" ]] || return 1
  local pid
  pid="$(cat "${pid_file}" 2>/dev/null || true)"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null
}

start_bg() {
  local name="$1"
  shift
  local log_file="${LOG_DIR}/${name}.log"
  local pid_file="${LOG_DIR}/${name}.pid"
  : >"${log_file}"
  if command -v setsid >/dev/null 2>&1; then
    nohup setsid bash -lc "$*" </dev/null >"${log_file}" 2>&1 &
  else
    nohup bash -lc "$*" </dev/null >"${log_file}" 2>&1 &
  fi
  printf '%s\n' "$!" >"${pid_file}"
}

ensure_bg_alive() {
  local name="$1"
  local pid_file="${LOG_DIR}/${name}.pid"
  if pid_alive "${pid_file}"; then
    return 0
  fi
  if [[ -f "${LOG_DIR}/${name}.log" ]]; then
    tail -n 80 "${LOG_DIR}/${name}.log" >&2 || true
  fi
  fail "${name} exited early"
}

wait_for_probe() {
  local label="$1"
  shift
  local timeout_s="${1:?missing timeout}"
  shift
  local start_ts
  start_ts="$(date +%s)"
  while true; do
    if "$@" >/dev/null 2>&1; then
      log "${label} ready"
      return 0
    fi
    local now_ts
    now_ts="$(date +%s)"
    if (( now_ts - start_ts >= timeout_s )); then
      return 1
    fi
    sleep 1
  done
}

start_micro_ros_agent_fresh() {
  kill_pid_file "${LOG_DIR}/micro_ros_agent.pid"
  stop_processes_matching "micro_ros_agent.*udp4 --port ${AGENT_PORT}"
  log "starting fresh micro_ros_agent on udp/${AGENT_PORT}"
  start_bg "micro_ros_agent" "source '${ROS_SETUP}' >/dev/null 2>&1; source '${MICROROS_WS_SETUP}' >/dev/null 2>&1; exec '${MICROROS_AGENT_BIN}' udp4 --port ${AGENT_PORT}"
  sleep 2
  ensure_bg_alive "micro_ros_agent"
}

run_rt_helper() {
  local helper_args=(
    "${RT_HELPER}"
    --port "${RT_PORT}"
    --baud "${RT_BAUD}"
    --check-shell
    --wait-for-prompt
    --command "chassis_car_app_f103"
    --wait "4.0"
    --command "microros_chassis_f103 udp ${AGENT_IP} ${AGENT_PORT} --f103-uart ${F103_UART} --f103-baud ${F103_BAUD}"
    --wait "8.0"
    --fail-on-unknown
  )
  run_with_rt_port_access python3 "${helper_args[@]}"
}

full_ros_session_active() {
  pgrep -af "ros2 launch rt_robot_nav2 f103_mapping.launch.py" >/dev/null 2>&1 || \
  pgrep -af "ros2 launch rt_robot_nav2 f103_navigation.launch.py" >/dev/null 2>&1
}

[[ -f "${ROS_SETUP}" ]] || fail "ROS 2 Foxy environment not found: ${ROS_SETUP}"
[[ -f "${ROS_WS_SETUP}" ]] || fail "ROS workspace not found: ${ROS_WS_SETUP}"
[[ -f "${MICROROS_WS_SETUP}" ]] || fail "micro-ROS workspace not found: ${MICROROS_WS_SETUP}"
[[ -x "${MICROROS_AGENT_BIN}" ]] || fail "micro_ros_agent not found: ${MICROROS_AGENT_BIN}"
[[ -f "${RT_HELPER}" ]] || fail "RT shell helper not found: ${RT_HELPER}"
[[ -f "${ROS_PROBE}" ]] || fail "ROS probe not found: ${ROS_PROBE}"

resolve_log_dir
: >"${LOG_DIR}/session.log"
RT_PORT_STATE_FILE="${LOG_DIR}/rt_shell_port.last"

log "starting F103 arm-only bridge"
log "rt_port=${RT_PORT} agent=${AGENT_IP}:${AGENT_PORT} f103_uart=${F103_UART}:${F103_BAUD}"

source_ros_env

if wait_for_probe "/arm_cmd subscriptions" 2 python3 "${ROS_PROBE}" topic-endpoints --topic /arm_cmd --kind subscriptions --timeout 0.8; then
  log "reusing existing /arm_cmd subscriber chain"
  cat <<EOF

Arm command path is already live.

Useful commands on the board:
  bash ${ROOT_DIR}/start_board_keyboard_arm_teleop.sh --wait-subscriber 5 --no-stop-on-exit
  python3 ${ROOT_DIR}/ros2_tools/arm_cmd_cli.py --wait-subscriber 5 servo --set 1=2200 --duration 400
  python3 ${ROOT_DIR}/ros2_tools/arm_cmd_cli.py --wait-subscriber 5 servo --set 1=1500 --duration 600
  python3 ${ROOT_DIR}/ros2_tools/arm_cmd_cli.py --wait-subscriber 5 ik6 120 120 120 0 -45 0 --duration 1500

EOF
  exit 0
fi

if full_ros_session_active; then
  fail "mapping/navigation is already running but /arm_cmd is not ready; do not start arm-only until the full session is fixed or stopped"
fi

if pid_alive "${LOG_DIR}/micro_ros_agent.pid"; then
  log "existing arm-only runtime detected; running pre-stop cleanup"
  bash "${STOP_SCRIPT}" >/dev/null 2>&1 || true
fi

if ! wait_for_rt_shell_ready 25; then
  cat "${LOG_DIR}/rt_shell_probe.log" >&2 || true
  fail "RT shell did not become ready; power-cycle the board and retry"
fi

start_micro_ros_agent_fresh

log "starting RT F103 bridge over ${RT_PORT}"
if ! run_rt_helper >"${LOG_DIR}/rt_shell_start.log" 2>&1; then
  cat "${LOG_DIR}/rt_shell_start.log" >&2 || true
  if grep -q "RT shell has no echo" "${LOG_DIR}/rt_shell_start.log" 2>/dev/null; then
    fail "RT shell lost echo during startup; stop here and power-cycle the board"
  fi
  fail "failed to start RT shell F103 chain"
fi

if ! wait_for_probe "/arm_cmd subscriptions" "${ARM_SUB_TIMEOUT}" python3 "${ROS_PROBE}" topic-endpoints --topic /arm_cmd --kind subscriptions --timeout 2.5; then
  cat "${LOG_DIR}/rt_shell_start.log" >&2 || true
  fail "/arm_cmd has no subscribers after RT startup; F103 arm bridge is not connected"
fi

if wait_for_probe "/odom_raw" "${ODOM_TIMEOUT}" python3 "${ROS_PROBE}" odom --topic /odom_raw --timeout 2.5; then
  log "/odom_raw sanity check passed"
else
  log "WARN: /odom_raw is not visible yet; arm control is up but odom was not confirmed"
fi

if wait_for_probe "/arm/move_pose service" 3 python3 "${ROS_PROBE}" service --name /arm/move_pose --timeout 1.0; then
  log "/arm/move_pose service is ready"
else
  log "WARN: /arm/move_pose service is not visible; /arm_cmd raw topic can still be used"
fi

cat <<EOF

Arm-only bridge is up.

Logs:
  ${LOG_DIR}/session.log
  ${LOG_DIR}/micro_ros_agent.log
  ${LOG_DIR}/rt_shell_start.log

Useful commands on the board:
  bash ${ROOT_DIR}/start_board_keyboard_arm_teleop.sh --wait-subscriber 5 --no-stop-on-exit
  python3 ${ROOT_DIR}/ros2_tools/arm_cmd_cli.py --wait-subscriber 5 servo --set 1=2200 --duration 400
  python3 ${ROOT_DIR}/ros2_tools/arm_cmd_cli.py --wait-subscriber 5 servo --set 1=1500 --duration 600
  python3 ${ROOT_DIR}/ros2_tools/arm_cmd_cli.py --wait-subscriber 5 ik6 120 120 120 0 -45 0 --duration 1500
  ros2 service call /arm/move_pose car2_arm_interfaces/srv/MoveArmPose "{x: 120.0, y: 120.0, z: 120.0, rx: 0.0, ry: -45.0, rz: 0.0, duration: 1.5, wait_done: true}"
  bash ${ROOT_DIR}/stop_board_f103_arm_only.sh

EOF
