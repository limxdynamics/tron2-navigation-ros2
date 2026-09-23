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
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-26}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
PYTHON_EXECUTABLE="${PYTHON_EXECUTABLE:-/usr/bin/python3}"
FAST_LIO_PYTHON_SITE="${FAST_LIO_PYTHON_SITE:-${ROOT_DIR}/.fast_lio_python_site}"
SYSTEM_PYTHON_SITE="${SYSTEM_PYTHON_SITE:-/usr/lib/python3/dist-packages}"
MAP_TOOLS_SITE="${PCT_MAP_TOOLS_SITE:-${ROOT_DIR}/.map-tools/site}"
MAP_SELECTOR="${MAP_SELECTOR:-${ROOT_DIR}/select_navigation_map.py}"
NAVIGATION_MAP_SELECTION="${NAVIGATION_MAP_SELECTION:-prompt}"
NAVIGATION_MAP_NAME="${NAVIGATION_MAP_NAME:-}"
FAST_LIO_ROS2_SETUP="${FAST_LIO_ROS2_SETUP:-${ROOT_DIR}/build/native_fast_lio/install/setup.bash}"
RSLIDAR_CONFIG="${RSLIDAR_CONFIG:-${ROOT_DIR}/RSLIDAR_SDK_ROS2/config/config.yaml}"
LOCALIZATION_MAP="${LOCALIZATION_MAP:-${ROOT_DIR}/PCT_planner-RC2026_Map_Planner/rsc/pcd/map.pcd}"
LOCALIZATION_CONFIG_PATH="${LOCALIZATION_CONFIG_PATH:-${ROOT_DIR}/FAST_LIO_LOCALIZATION2/config}"
LOCALIZATION_CONFIG_FILE="${LOCALIZATION_CONFIG_FILE:-rs_fairy.yaml}"
LOCALIZATION_Z_OFFSET="${LOCALIZATION_Z_OFFSET:-0.0}"
BASE_YAW_OFFSET_DEG="${BASE_YAW_OFFSET_DEG:-60.0}"
LOCALIZATION_MAP_TOPIC="${LOCALIZATION_MAP_TOPIC:-/fast_lio_map}"
POSE_TOPIC="${POSE_TOPIC:-/pose_stamped}"
CLOUD_TOPIC="${CLOUD_TOPIC:-/corrected_current_pcd}"
PCT_TOMOGRAM="${PCT_TOMOGRAM:-${ROOT_DIR}/PCT_planner-RC2026_Map_Planner/rsc/tomogram/map.pickle}"
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

is_valid_robot_accid() {
  local value="$1"
  [[ ${#value} -le 64 && "${value}" =~ ^WF_[A-Za-z0-9]+_[A-Za-z0-9]+$ ]]
}

is_valid_websocket_url() {
  local value="$1"
  [[ "${value}" =~ ^wss?://[^[:space:]]+$ ]]
}

case "${CHASSIS_OUTPUT_MODE,,}" in
  off) ENABLE_WEBSOCKET_COMMANDS=false ;;
  websocket) ENABLE_WEBSOCKET_COMMANDS=true ;;
  *)
    echo "CHASSIS_OUTPUT_MODE must be one of: off, websocket." >&2
    exit 2
    ;;
esac
if [[ "${WEBSOCKET_PROTOCOL,,}" != "legacy" ]]; then
  echo "Only WEBSOCKET_PROTOCOL=legacy is supported by the navigation launcher." >&2
  exit 2
fi
if [[ "${ENABLE_WEBSOCKET_COMMANDS}" == true ]]; then
  if ! is_valid_websocket_url "${WEBSOCKET_URL}"; then
    echo "WEBSOCKET_URL must be configured as ws://... or wss://... in config/navigation.env or the environment." >&2
    exit 2
  fi
  if ! is_valid_robot_accid "${WEBSOCKET_ROBOT_ACCID}"; then
    echo "WEBSOCKET_ROBOT_ACCID must match WF_<MODEL>_<ID> and be at most 64 characters." >&2
    exit 2
  fi
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS 2 Humble setup not found: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${FAST_LIO_ROS2_SETUP}" ]]; then
  echo "Native FAST-LIO overlay not found: ${FAST_LIO_ROS2_SETUP}" >&2
  echo "Run build_fast_lio_ros2_humble.sh first." >&2
  exit 1
fi
if [[ ! -f "${RSLIDAR_CONFIG}" ]]; then
  echo "Native RoboSense config not found: ${RSLIDAR_CONFIG}" >&2
  exit 1
fi
if [[ ! -x "${PYTHON_EXECUTABLE}" ]]; then
  echo "Python executable not found: ${PYTHON_EXECUTABLE}" >&2
  exit 1
fi
export PATH="$(dirname "${PYTHON_EXECUTABLE}"):${PATH}"
if [[ ! -f "${MAP_SELECTOR}" ]]; then
  echo "Navigation map selector not found: ${MAP_SELECTOR}" >&2
  exit 1
fi
if pgrep -f '[r]slidar_sdk_node' >/dev/null 2>&1; then
  echo "An rslidar_sdk_node is already running." >&2
  echo "Stop it before selecting a map or starting navigation." >&2
  exit 1
fi

default_localization_map="${ROOT_DIR}/PCT_planner-RC2026_Map_Planner/rsc/pcd/map.pcd"
default_pct_tomogram="${ROOT_DIR}/PCT_planner-RC2026_Map_Planner/rsc/tomogram/map.pickle"
if [[ "${LOCALIZATION_MAP}" == "${default_localization_map}" && \
      "${PCT_TOMOGRAM}" == "${default_pct_tomogram}" ]]; then
  selector_args=(--root "${ROOT_DIR}")
  if [[ -n "${NAVIGATION_MAP_NAME}" ]]; then
    selector_args+=(--name "${NAVIGATION_MAP_NAME}")
  else
    case "${NAVIGATION_MAP_SELECTION,,}" in
      prompt)
        if [[ -t 0 && -t 1 ]]; then
          selector_args+=(--mode prompt)
        else
          echo "No interactive terminal; keeping and verifying the active map."
          selector_args+=(--mode current)
        fi
        ;;
      current|latest)
        selector_args+=(--mode "${NAVIGATION_MAP_SELECTION,,}")
        ;;
      *)
        echo "NAVIGATION_MAP_SELECTION must be prompt, current, or latest." >&2
        exit 2
        ;;
    esac
  fi
  PYTHONPATH="${MAP_TOOLS_SITE}${PYTHONPATH:+:${PYTHONPATH}}" \
    "${PYTHON_EXECUTABLE}" "${MAP_SELECTOR}" "${selector_args[@]}"
else
  echo "Explicit map paths selected; interactive map selection is skipped."
  echo "  localization=${LOCALIZATION_MAP}"
  echo "  PCT=${PCT_TOMOGRAM}"
fi
if [[ ! -f "${LOCALIZATION_MAP}" ]]; then
  echo "Localization PCD not found: ${LOCALIZATION_MAP}" >&2
  exit 1
fi
if [[ ! -f "${LOCALIZATION_CONFIG_PATH}/${LOCALIZATION_CONFIG_FILE}" ]]; then
  echo "Localization config not found: ${LOCALIZATION_CONFIG_PATH}/${LOCALIZATION_CONFIG_FILE}" >&2
  exit 1
fi
if [[ ! -f "${PCT_TOMOGRAM}" ]]; then
  echo "PCT tomogram not found: ${PCT_TOMOGRAM}" >&2
  exit 1
fi
set +u
source "${ROS_SETUP}"
source "${FAST_LIO_ROS2_SETUP}"
set -u

if ! ros2 pkg prefix fast_lio_localization >/dev/null 2>&1; then
  echo "fast_lio_localization is not visible after sourcing ${FAST_LIO_ROS2_SETUP}" >&2
  exit 1
fi
if ! ros2 pkg prefix rslidar_sdk >/dev/null 2>&1; then
  echo "Native ROS 2 rslidar_sdk is not visible after sourcing ${FAST_LIO_ROS2_SETUP}" >&2
  exit 1
fi
FAST_LIO_RUNTIME_PYTHONPATH="${SYSTEM_PYTHON_SITE}${PYTHONPATH:+:${PYTHONPATH}}"
if [[ -d "${FAST_LIO_PYTHON_SITE}" ]]; then
  FAST_LIO_RUNTIME_PYTHONPATH="${FAST_LIO_PYTHON_SITE}:${FAST_LIO_RUNTIME_PYTHONPATH}"
fi
if ! PYTHONNOUSERSITE=1 PYTHONPATH="${FAST_LIO_RUNTIME_PYTHONPATH}" \
  "${PYTHON_EXECUTABLE}" -c \
  'import open3d, rclpy, transforms3d; from sensor_msgs.msg import PointCloud2' \
  >/dev/null 2>&1; then
  echo "FAST-LIO localization Python dependencies are incomplete." >&2
  echo "Required in the ROS 2 Humble Python environment: open3d and transforms3d." >&2
  echo "Checked isolated path: ${FAST_LIO_RUNTIME_PYTHONPATH}" >&2
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

cleanup() {
  if [[ "${cleaned_up}" == true ]]; then
    return
  fi
  cleaned_up=true
  trap - HUP INT TERM EXIT
  # A Bash process started as an asynchronous job may inherit SIGINT as
  # ignored. SIGTERM is trapped by both launchers and still gives ROS nodes a
  # graceful shutdown path, including the bridge's final zero command.
  signal_process_groups TERM
  # Give the nested navigation launcher enough time to clean its own groups.
  if ! wait_for_process_groups 200; then
    signal_process_groups KILL
    wait_for_process_groups 20 || true
  fi
  for pid in "${pids[@]:-}"; do
    wait "${pid}" 2>/dev/null || true
  done
}
trap cleanup HUP INT TERM EXIT

echo "Starting native ROS 2 RoboSense driver and FAST-LIO localization"
echo "  map=${LOCALIZATION_MAP}"
echo "  driver config=${RSLIDAR_CONFIG}"
echo "  ROS 2 sensors: /rslidar_points + /rslidar_imu_data"
echo "  outputs: pose=${POSE_TOPIC}, cloud=${CLOUD_TOPIC}, frame=map"
echo "  FAST-LIO to robot-base yaw offset=${BASE_YAW_OFFSET_DEG} deg"
echo "  navigation starts paused=${START_PAUSED}"
echo "  PCT stair centerline=${ENABLE_PCT_STAIR_CENTERLINE}"
echo "  requested chassis output mode=${CHASSIS_OUTPUT_MODE}"
echo "  LimX WebSocket chassis output=${ENABLE_WEBSOCKET_COMMANDS}; protocol=${WEBSOCKET_PROTOCOL} (${WEBSOCKET_URL})"

setsid ros2 run rslidar_sdk rslidar_sdk_node --ros-args \
  -p config_path:="${RSLIDAR_CONFIG}" &
pids+=("$!")

PYTHONNOUSERSITE=1 PYTHONPATH="${FAST_LIO_RUNTIME_PYTHONPATH}" \
setsid ros2 launch fast_lio_localization localization.launch.py \
  config_path:="${LOCALIZATION_CONFIG_PATH}" \
  config_file:="${LOCALIZATION_CONFIG_FILE}" \
  map:="${LOCALIZATION_MAP}" \
  pcd_map_topic:="${LOCALIZATION_MAP_TOPIC}" \
  frame_id:=map \
  pose_topic:="${POSE_TOPIC}" \
  corrected_cloud_topic:="${CLOUD_TOPIC}" \
  publish_before_localization:=false \
  z_offset:="${LOCALIZATION_Z_OFFSET}" \
  base_yaw_offset_deg:="${BASE_YAW_OFFSET_DEG}" \
  rviz:=false &
pids+=("$!")

POSE_TOPIC="${POSE_TOPIC}" \
CLOUD_TOPIC="${CLOUD_TOPIC}" \
PCT_TOMOGRAM="${PCT_TOMOGRAM}" \
START_PAUSED="${START_PAUSED}" \
CHASSIS_OUTPUT_MODE="${CHASSIS_OUTPUT_MODE}" \
WEBSOCKET_URL="${WEBSOCKET_URL}" \
WEBSOCKET_PROTOCOL="${WEBSOCKET_PROTOCOL}" \
WEBSOCKET_ROBOT_ACCID="${WEBSOCKET_ROBOT_ACCID}" \
WEBSOCKET_SEND_RATE="${WEBSOCKET_SEND_RATE}" \
WEBSOCKET_COMMAND_TIMEOUT="${WEBSOCKET_COMMAND_TIMEOUT}" \
WEBSOCKET_MAX_LINEAR_SPEED="${WEBSOCKET_MAX_LINEAR_SPEED}" \
WEBSOCKET_MAX_LATERAL_SPEED="${WEBSOCKET_MAX_LATERAL_SPEED}" \
WEBSOCKET_MAX_ANGULAR_SPEED="${WEBSOCKET_MAX_ANGULAR_SPEED}" \
CONTROLLER_MAX_LINEAR_SPEED="${CONTROLLER_MAX_LINEAR_SPEED}" \
CONTROLLER_MAX_ANGULAR_SPEED="${CONTROLLER_MAX_ANGULAR_SPEED}" \
ENABLE_LOCAL_OBSTACLE_AVOIDANCE="${ENABLE_LOCAL_OBSTACLE_AVOIDANCE}" \
ENABLE_PCT_STAIR_CENTERLINE="${ENABLE_PCT_STAIR_CENTERLINE}" \
  setsid bash "${ROOT_DIR}/run_nx_navigation_humble.sh" &
pids+=("$!")

set +e
wait -n "${pids[@]}"
status=$?
set -e
cleanup
exit "${status}"
