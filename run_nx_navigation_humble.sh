#!/usr/bin/env bash
# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAVIGATION_ENV_FILE="${NAVIGATION_ENV_FILE:-${ROOT_DIR}/config/navigation.env}"
if [[ -r "${NAVIGATION_ENV_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${NAVIGATION_ENV_FILE}"
fi
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ -f "${ROOT_DIR}/build/permissive_scan/install/setup.bash" ]]; then
  DEFAULT_SCAN_ROOT="${ROOT_DIR}/build/permissive_scan"
else
  DEFAULT_SCAN_ROOT="${ROOT_DIR}/SCAN-Planner-ros2-community"
fi
SCAN_ROOT="${SCAN_ROOT:-${DEFAULT_SCAN_ROOT}}"
PCT_ROOT="${PCT_ROOT:-${ROOT_DIR}/PCT_planner-RC2026_Map_Planner/planner}"
PYTHON_EXECUTABLE="${PYTHON_EXECUTABLE:-/usr/bin/python3}"
PCT_TOMOGRAM="${PCT_TOMOGRAM:-${PCT_ROOT}/../rsc/tomogram/map.pickle}"
PCT_TOMOGRAM_PUBLISHER="${PCT_TOMOGRAM_PUBLISHER:-${PCT_ROOT}/../tomography/scripts/publish_tomogram_from_pickle.py}"
PUBLISH_PCT_TOMOGRAM="${PUBLISH_PCT_TOMOGRAM:-true}"
PCT_TOMOGRAM_STRIDE="${PCT_TOMOGRAM_STRIDE:-1}"
PCT_TOMOGRAM_CHUNK_TOPIC="${PCT_TOMOGRAM_CHUNK_TOPIC:-/pct_tomogram_chunks}"
PCT_TOMOGRAM_CHUNK_POINTS="${PCT_TOMOGRAM_CHUNK_POINTS:-512}"
PCT_VISUALIZATION_COST_THRESHOLD="${PCT_VISUALIZATION_COST_THRESHOLD:-}"
PCT_VISUALIZATION_MIN_COMPONENT_CELLS="${PCT_VISUALIZATION_MIN_COMPONENT_CELLS:-1}"
PCT_HIDE_TERMINAL_ENVELOPE="${PCT_HIDE_TERMINAL_ENVELOPE:-false}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-26}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

FRAME_ID="${FRAME_ID:-map}"
POSE_TOPIC="${POSE_TOPIC:-/pose_stamped}"
BODY_ODOM_TOPIC="${BODY_ODOM_TOPIC:-/scan/body_odom}"
CLOUD_TOPIC="${CLOUD_TOPIC:-/corrected_current_pcd}"
CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/sdk_cmd_vel}"
GOAL_TOPIC="${GOAL_TOPIC:-/goal_pose}"
REFERENCE_PATH_TOPIC="${REFERENCE_PATH_TOPIC:-/pct_path}"
START_PAUSED="${START_PAUSED:-true}"
CHASSIS_OUTPUT_MODE="${CHASSIS_OUTPUT_MODE:-off}"
WEBSOCKET_URL="${WEBSOCKET_URL:-}"
WEBSOCKET_PROTOCOL="${WEBSOCKET_PROTOCOL:-legacy}"
WEBSOCKET_ROBOT_ACCID="${WEBSOCKET_ROBOT_ACCID:-}"
WEBSOCKET_SEND_RATE="${WEBSOCKET_SEND_RATE:-10.0}"
WEBSOCKET_COMMAND_TIMEOUT="${WEBSOCKET_COMMAND_TIMEOUT:-0.3}"
WEBSOCKET_MAX_LINEAR_SPEED="${WEBSOCKET_MAX_LINEAR_SPEED:-0.60}"
WEBSOCKET_MAX_LATERAL_SPEED="${WEBSOCKET_MAX_LATERAL_SPEED:-0.60}"
WEBSOCKET_MAX_ANGULAR_SPEED="${WEBSOCKET_MAX_ANGULAR_SPEED:-0.20}"
CONTROLLER_MAX_LINEAR_SPEED="${CONTROLLER_MAX_LINEAR_SPEED:-}"
CONTROLLER_MAX_ANGULAR_SPEED="${CONTROLLER_MAX_ANGULAR_SPEED:-}"
ENABLE_LOCAL_OBSTACLE_AVOIDANCE="${ENABLE_LOCAL_OBSTACLE_AVOIDANCE:-true}"
ENABLE_PCT_STAIR_CENTERLINE="${ENABLE_PCT_STAIR_CENTERLINE:-false}"

is_true() {
  [[ "${1,,}" == "true" || "${1}" == "1" || "${1,,}" == "yes" ]]
}

normalize_boolean() {
  case "${1,,}" in
    true|1|yes) echo true ;;
    false|0|no) echo false ;;
    *) return 1 ;;
  esac
}

is_positive_number_at_most() {
  local value="$1"
  local maximum="$2"
  [[ "${value}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] &&
    awk -v value="${value}" -v maximum="${maximum}" \
      'BEGIN { exit !(value > 0.0 && value <= maximum) }'
}

is_valid_robot_accid() {
  local value="$1"
  [[ ${#value} -le 64 && "${value}" =~ ^WF_[A-Za-z0-9]+_[A-Za-z0-9]+$ ]]
}

is_valid_websocket_url() {
  local value="$1"
  [[ "${value}" =~ ^wss?://[^[:space:]]+$ ]]
}

has_live_process() {
  local process_name="$1"
  local pid state
  while read -r pid; do
    [[ -n "${pid}" ]] || continue
    state="$(ps -o stat= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
    if [[ -n "${state}" && "${state}" != Z* ]]; then
      return 0
    fi
  done < <(pgrep -x "${process_name}" 2>/dev/null || true)
  return 1
}

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS 2 setup not found: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${SCAN_ROOT}/install/setup.bash" ]]; then
  echo "SCAN is not built: ${SCAN_ROOT}/install/setup.bash" >&2
  exit 1
fi
if [[ ! -f "${PCT_ROOT}/run_navigation_humble.sh" ]]; then
  echo "PCT launcher not found: ${PCT_ROOT}/run_navigation_humble.sh" >&2
  exit 1
fi
if is_true "${PUBLISH_PCT_TOMOGRAM}"; then
  if [[ ! -x "${PYTHON_EXECUTABLE}" ]]; then
    echo "Python executable not found: ${PYTHON_EXECUTABLE}" >&2
    exit 1
  fi
  if [[ ! -f "${PCT_TOMOGRAM}" ]]; then
    echo "PCT tomogram not found: ${PCT_TOMOGRAM}" >&2
    exit 1
  fi
  if [[ ! -f "${PCT_TOMOGRAM_PUBLISHER}" ]]; then
    echo "PCT tomogram publisher not found: ${PCT_TOMOGRAM_PUBLISHER}" >&2
    exit 1
  fi
fi

effective_chassis_mode="${CHASSIS_OUTPUT_MODE,,}"

case "${effective_chassis_mode}" in
  off|websocket) ;;
  *)
    echo "CHASSIS_OUTPUT_MODE must be one of: off, websocket." >&2
    exit 2
    ;;
esac

WEBSOCKET_PROTOCOL="${WEBSOCKET_PROTOCOL,,}"
if [[ "${WEBSOCKET_PROTOCOL}" != "legacy" ]]; then
  echo "Only WEBSOCKET_PROTOCOL=legacy is supported by the navigation launcher." >&2
  exit 2
fi

if [[ "${effective_chassis_mode}" == "websocket" ]]; then
  if ! is_valid_websocket_url "${WEBSOCKET_URL}"; then
    echo "WEBSOCKET_URL must be configured as ws://... or wss://... in config/navigation.env or the environment." >&2
    exit 2
  fi
  if ! is_valid_robot_accid "${WEBSOCKET_ROBOT_ACCID}"; then
    echo "WEBSOCKET_ROBOT_ACCID must match WF_<MODEL>_<ID> and be at most 64 characters." >&2
    exit 2
  fi
fi

if [[ "${effective_chassis_mode}" == "websocket" ]]; then
  ENABLE_WEBSOCKET_COMMANDS=true
else
  ENABLE_WEBSOCKET_COMMANDS=false
fi

if [[ -z "${CONTROLLER_MAX_LINEAR_SPEED}" ]]; then
  case "${effective_chassis_mode}" in
    websocket) CONTROLLER_MAX_LINEAR_SPEED="${WEBSOCKET_MAX_LINEAR_SPEED}" ;;
    *) CONTROLLER_MAX_LINEAR_SPEED=0.60 ;;
  esac
fi
if [[ -z "${CONTROLLER_MAX_ANGULAR_SPEED}" ]]; then
  case "${effective_chassis_mode}" in
    websocket) CONTROLLER_MAX_ANGULAR_SPEED="${WEBSOCKET_MAX_ANGULAR_SPEED}" ;;
    *) CONTROLLER_MAX_ANGULAR_SPEED=0.20 ;;
  esac
fi

if ! ENABLE_LOCAL_OBSTACLE_AVOIDANCE="$(
  normalize_boolean "${ENABLE_LOCAL_OBSTACLE_AVOIDANCE}"
)"; then
  echo "ENABLE_LOCAL_OBSTACLE_AVOIDANCE must be true or false." >&2
  exit 2
fi
if ! ENABLE_PCT_STAIR_CENTERLINE="$(
  normalize_boolean "${ENABLE_PCT_STAIR_CENTERLINE}"
)"; then
  echo "ENABLE_PCT_STAIR_CENTERLINE must be true or false." >&2
  exit 2
fi
if [[ "${ENABLE_PCT_STAIR_CENTERLINE}" == true && \
      "${ENABLE_LOCAL_OBSTACLE_AVOIDANCE}" != false ]]; then
  echo "PCT stair centerline mode requires local obstacle avoidance to be disabled." >&2
  exit 2
fi
if [[ "${ENABLE_LOCAL_OBSTACLE_AVOIDANCE}" == false ]]; then
  is_true "${START_PAUSED}" || {
    echo "Disabling local obstacle avoidance requires START_PAUSED=true." >&2
    exit 2
  }
  is_positive_number_at_most "${CONTROLLER_MAX_LINEAR_SPEED}" 0.60 || {
    echo "Without local obstacle avoidance, controller linear speed must be in (0, 0.60] m/s." >&2
    exit 2
  }
  if [[ "${effective_chassis_mode}" == websocket ]]; then
    is_positive_number_at_most "${WEBSOCKET_MAX_LINEAR_SPEED}" 0.60 &&
      is_positive_number_at_most "${WEBSOCKET_MAX_LATERAL_SPEED}" 0.60 || {
        echo "Without local obstacle avoidance, WebSocket linear/lateral limits must be in (0, 0.60] m/s." >&2
        exit 2
      }
  fi
fi

set +u
source "${ROS_SETUP}"
source "${SCAN_ROOT}/install/setup.bash"
set -u

if is_true "${ENABLE_WEBSOCKET_COMMANDS}" &&
  { has_live_process sdk_walk_bridge ||
    pgrep -f '[m]ros_ros2_bridge' >/dev/null 2>&1 ||
    pgrep -f '[l]imx_websocket_cmd_bridge' >/dev/null 2>&1; }
then
  echo "Refusing chassis control: another chassis command bridge is already running." >&2
  echo "Stop the old bridge so the robot has exactly one velocity source." >&2
  exit 1
fi
if is_true "${ENABLE_WEBSOCKET_COMMANDS}"; then
  WEBSOCKET_BRIDGE_BIN="${SCAN_ROOT}/install/scan_planner/lib/scan_planner/limx_websocket_cmd_bridge"
  if [[ ! -x "${PYTHON_EXECUTABLE}" ]]; then
    echo "Python executable not found: ${PYTHON_EXECUTABLE}" >&2
    exit 1
  fi
  if [[ ! -x "${WEBSOCKET_BRIDGE_BIN}" ]]; then
    echo "ROS 2 WebSocket bridge is not installed: ${WEBSOCKET_BRIDGE_BIN}" >&2
    echo "Rebuild SCAN after adding the direct chassis bridge." >&2
    exit 1
  fi
  if ! "${PYTHON_EXECUTABLE}" -c 'import rclpy, websocket' >/dev/null 2>&1; then
    echo "WebSocket bridge dependencies are incomplete." >&2
    echo "Install ROS 2 rclpy and Ubuntu package python3-websocket." >&2
    exit 1
  fi
fi
if is_true "${ENABLE_WEBSOCKET_COMMANDS}" &&
  { has_live_process move_base_node || has_live_process move_base; }
then
  echo "Refusing chassis command forwarding: move_base_node is still running." >&2
  echo "Stop the old controller first; do not create two /sdk_cmd_vel publishers." >&2
  exit 1
fi

pids=()
cleaned_up=false
process_groups_alive() {
  local pid
  for pid in "${pids[@]:-}"; do
    if kill -0 -- "-${pid}" 2>/dev/null; then
      return 0
    fi
  done
  return 1
}

signal_process_groups() {
  local signal="$1"
  local pid
  for pid in "${pids[@]:-}"; do
    kill "-${signal}" -- "-${pid}" 2>/dev/null || true
  done
}

wait_for_process_groups() {
  local attempts="${1:-50}"
  local attempt
  for ((attempt = 0; attempt < attempts; ++attempt)); do
    process_groups_alive || return 0
    sleep 0.1
  done
  return 1
}

publish_safe_stop() {
  timeout 2s ros2 topic pub -r 20 -t 5 -w 1 \
    /scan_planner/pause std_msgs/msg/Bool '{data: true}' \
    >/dev/null 2>&1 || true
  timeout 2s ros2 topic pub -r 20 -t 5 -w 1 \
    "${CMD_VEL_TOPIC}" geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
    >/dev/null 2>&1 || true
}

cleanup() {
  if [[ "${cleaned_up}" == true ]]; then
    return
  fi
  cleaned_up=true
  trap - HUP INT TERM EXIT
  # Stop command production and force a zero through the live chassis bridge
  # before any ROS process is terminated.
  publish_safe_stop
  signal_process_groups TERM
  if ! wait_for_process_groups 100; then
    signal_process_groups KILL
    wait_for_process_groups 20 || true
  fi
  for pid in "${pids[@]:-}"; do
    wait "${pid}" 2>/dev/null || true
  done
}
trap cleanup HUP INT TERM EXIT

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, frame=${FRAME_ID}"
echo "Inputs: pose=${POSE_TOPIC}, cloud=${CLOUD_TOPIC}, goal=${GOAL_TOPIC}"
echo "Outputs: path=${REFERENCE_PATH_TOPIC}, cmd=${CMD_VEL_TOPIC}"
echo "Controller starts paused: ${START_PAUSED}"
echo "Local obstacle avoidance: ${ENABLE_LOCAL_OBSTACLE_AVOIDANCE}"
echo "PCT stair centerline: ${ENABLE_PCT_STAIR_CENTERLINE}"
if [[ "${ENABLE_LOCAL_OBSTACLE_AVOIDANCE}" == false ]]; then
  echo "WARNING: obstacle cloud subscription is disabled; PCT path-only test mode is active."
fi
echo "Chassis output mode: ${effective_chassis_mode}"
echo "Controller limits: linear=${CONTROLLER_MAX_LINEAR_SPEED}, angular=${CONTROLLER_MAX_ANGULAR_SPEED}"
echo "LimX WebSocket chassis output: ${ENABLE_WEBSOCKET_COMMANDS}; protocol=${WEBSOCKET_PROTOCOL}; url=${WEBSOCKET_URL}; accid=${WEBSOCKET_ROBOT_ACCID}; safety limits=(${WEBSOCKET_MAX_LINEAR_SPEED}, ${WEBSOCKET_MAX_LATERAL_SPEED}, ${WEBSOCKET_MAX_ANGULAR_SPEED})"
echo "NX PCT map publisher: ${PUBLISH_PCT_TOMOGRAM}; chunk_topic=${PCT_TOMOGRAM_CHUNK_TOPIC}; stride=${PCT_TOMOGRAM_STRIDE}"
echo "PCT display filters: cost=${PCT_VISUALIZATION_COST_THRESHOLD:-off}; component>=${PCT_VISUALIZATION_MIN_COMPONENT_CELLS}; hide_terminal_envelope=${PCT_HIDE_TERMINAL_ENVELOPE}"

# Use node-scoped overrides: ROS 2 gives the node-specific YAML entries higher
# precedence than unscoped command-line rules.
PCT_TOMOGRAM="${PCT_TOMOGRAM}" setsid bash "${PCT_ROOT}/run_navigation_humble.sh" --ros-args \
  -p pct_planner:frame_id:="${FRAME_ID}" \
  -p pct_planner:pose_topic:="${POSE_TOPIC}" \
  -p pct_planner:goal_topic:="${GOAL_TOPIC}" \
  -p pct_planner:path_topic:="${REFERENCE_PATH_TOPIC}" \
  -p pct_planner:stair_centerline.enabled:="${ENABLE_PCT_STAIR_CENTERLINE}" &
pids+=("$!")

scan_launch_args=(
  "frame_id:=${FRAME_ID}"
  "pose_topic:=${POSE_TOPIC}"
  "body_odom_topic:=${BODY_ODOM_TOPIC}"
  "cloud_topic:=${CLOUD_TOPIC}"
  "enable_cloud_subscription:=${ENABLE_LOCAL_OBSTACLE_AVOIDANCE}"
  "cmd_vel_topic:=${CMD_VEL_TOPIC}"
  "reference_path_topic:=${REFERENCE_PATH_TOPIC}"
  "start_paused:=${START_PAUSED}"
  "start_websocket_bridge:=${ENABLE_WEBSOCKET_COMMANDS}"
  "websocket_url:=${WEBSOCKET_URL}"
  "websocket_protocol:=${WEBSOCKET_PROTOCOL}"
  "websocket_send_rate:=${WEBSOCKET_SEND_RATE}"
  "websocket_command_timeout:=${WEBSOCKET_COMMAND_TIMEOUT}"
  "websocket_max_linear_speed:=${WEBSOCKET_MAX_LINEAR_SPEED}"
  "websocket_max_lateral_speed:=${WEBSOCKET_MAX_LATERAL_SPEED}"
  "websocket_max_angular_speed:=${WEBSOCKET_MAX_ANGULAR_SPEED}"
  "controller_max_linear_speed:=${CONTROLLER_MAX_LINEAR_SPEED}"
  "controller_max_angular_speed:=${CONTROLLER_MAX_ANGULAR_SPEED}"
)
if [[ -n "${WEBSOCKET_ROBOT_ACCID}" ]]; then
  scan_launch_args+=("websocket_robot_accid:=${WEBSOCKET_ROBOT_ACCID}")
fi

setsid ros2 launch scan_planner nx_pct_navigation.launch.py \
  "${scan_launch_args[@]}" &
pids+=("$!")

if is_true "${PUBLISH_PCT_TOMOGRAM}"; then
  pct_visualization_args=(
    --minimum-component-cells "${PCT_VISUALIZATION_MIN_COMPONENT_CELLS}"
  )
  if [[ -n "${PCT_VISUALIZATION_COST_THRESHOLD}" ]]; then
    pct_visualization_args+=(
      --visualization-cost-threshold "${PCT_VISUALIZATION_COST_THRESHOLD}"
    )
  fi
  if is_true "${PCT_HIDE_TERMINAL_ENVELOPE}"; then
    pct_visualization_args+=(--hide-terminal-envelope)
  fi
  setsid "${PYTHON_EXECUTABLE}" "${PCT_TOMOGRAM_PUBLISHER}" \
    --tomogram "${PCT_TOMOGRAM}" \
    --topic "" \
    --chunk-topic "${PCT_TOMOGRAM_CHUNK_TOPIC}" \
    --chunk-points "${PCT_TOMOGRAM_CHUNK_POINTS}" \
    --pose-topic "${POSE_TOPIC}" \
    --frame-id "${FRAME_ID}" \
    --grid-stride "${PCT_TOMOGRAM_STRIDE}" \
    "${pct_visualization_args[@]}" \
    --publish-before-localization \
    --disable-goal-editor &
  pids+=("$!")
fi

set +e
wait -n "${pids[@]}"
status=$?
set -e
cleanup
exit "${status}"
