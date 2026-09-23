#!/usr/bin/env bash
# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FOXY_SETUP="${FOXY_SETUP:-/opt/ros/foxy/setup.bash}"
RVIZ_CONFIG="${RVIZ_CONFIG:-${ROOT_DIR}/SCAN-Planner-ros2-community/src/planner/plan_manage/config/nx_pct_navigation.rviz}"
PYTHON_EXECUTABLE="${PYTHON_EXECUTABLE:-/usr/bin/python3}"
TOMOGRAM_PUBLISHER="${TOMOGRAM_PUBLISHER:-${ROOT_DIR}/PCT_planner-RC2026_Map_Planner/tomography/scripts/publish_tomogram_from_pickle.py}"
TOMOGRAM_RECEIVER="${TOMOGRAM_RECEIVER:-${ROOT_DIR}/PCT_planner-RC2026_Map_Planner/tomography/scripts/receive_tomogram_chunks.py}"
PCT_TOMOGRAM="${PCT_TOMOGRAM:-${ROOT_DIR}/PCT_planner-RC2026_Map_Planner/rsc/tomogram/map.pickle}"
PCT_GOAL_EDITOR_TOMOGRAM="${PCT_GOAL_EDITOR_TOMOGRAM:-}"
PCT_TOMOGRAM_TOPIC="${PCT_TOMOGRAM_TOPIC:-/pct_tomogram_aligned}"
PCT_TOMOGRAM_CHUNK_TOPIC="${PCT_TOMOGRAM_CHUNK_TOPIC:-/pct_tomogram_chunks}"
PCT_TOMOGRAM_STRIDE="${PCT_TOMOGRAM_STRIDE:-2}"
PCT_VISUALIZATION_COST_THRESHOLD="${PCT_VISUALIZATION_COST_THRESHOLD:-}"
PCT_VISUALIZATION_MIN_COMPONENT_CELLS="${PCT_VISUALIZATION_MIN_COMPONENT_CELLS:-1}"
PCT_DISPLAY_MIN_Z_OFFSET="${PCT_DISPLAY_MIN_Z_OFFSET:-}"
PCT_DISPLAY_MAX_Z_OFFSET="${PCT_DISPLAY_MAX_Z_OFFSET:-}"
PCT_HIDE_TERMINAL_ENVELOPE="${PCT_HIDE_TERMINAL_ENVELOPE:-false}"
PCT_PREVIEW_POSE="${PCT_PREVIEW_POSE:-}"
START_PCT_TOMOGRAM="${START_PCT_TOMOGRAM:-false}"
LOCK_FILE="${XDG_RUNTIME_DIR:-/tmp}/navigation-local-rviz-foxy-${UID}.lock"

if ! command -v flock >/dev/null 2>&1; then
  echo "Required command not found: flock" >&2
  exit 1
fi
if ! command -v setsid >/dev/null 2>&1; then
  echo "Required command not found: setsid" >&2
  exit 1
fi
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  echo "Local PCT RViz is already running; close the existing window first." >&2
  exit 1
fi
existing_receivers="$(pgrep -af '[r]eceive_tomogram_chunks[.]py' || true)"
if [[ -n "${existing_receivers}" ]]; then
  echo "A stale PCT map receiver is already running; refusing a duplicate:" >&2
  printf '%s\n' "${existing_receivers}" >&2
  exit 1
fi

is_true() {
  [[ "${1,,}" == "true" || "${1}" == "1" || "${1,,}" == "yes" ]]
}

if [[ ! -f "${FOXY_SETUP}" ]]; then
  echo "ROS 2 Foxy is not installed: ${FOXY_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${RVIZ_CONFIG}" ]]; then
  echo "RViz configuration not found: ${RVIZ_CONFIG}" >&2
  exit 1
fi
if [[ ! -x "${PYTHON_EXECUTABLE}" ]]; then
  echo "Python executable not found: ${PYTHON_EXECUTABLE}" >&2
  exit 1
fi
if is_true "${START_PCT_TOMOGRAM}"; then
  if [[ ! -f "${TOMOGRAM_PUBLISHER}" ]]; then
    echo "Tomogram publisher not found: ${TOMOGRAM_PUBLISHER}" >&2
    exit 1
  fi
  if [[ ! -f "${PCT_TOMOGRAM}" ]]; then
    echo "PCT tomogram not found: ${PCT_TOMOGRAM}" >&2
    exit 1
  fi
  if [[ -n "${PCT_GOAL_EDITOR_TOMOGRAM}" && ! -f "${PCT_GOAL_EDITOR_TOMOGRAM}" ]]; then
    echo "PCT goal-editor tomogram not found: ${PCT_GOAL_EDITOR_TOMOGRAM}" >&2
    exit 1
  fi
elif [[ ! -f "${TOMOGRAM_RECEIVER}" ]]; then
  echo "PCT tomogram chunk receiver not found: ${TOMOGRAM_RECEIVER}" >&2
  exit 1
fi

# Do not mix an inherited ROS 1 Noetic environment with ROS 2 Foxy.
unset ROS_MASTER_URI ROS_ROOT ROS_PACKAGE_PATH ROS_ETC_DIR
unset ROS_VERSION ROS_PYTHON_VERSION ROS_DISTRO
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH

set +u
source "${FOXY_SETUP}"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-26}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if ! is_true "${START_PCT_TOMOGRAM}"; then
  if ! "${PYTHON_EXECUTABLE}" "${TOMOGRAM_RECEIVER}" --help >/dev/null; then
    echo "PCT map receiver import check failed; RViz was not started." >&2
    exit 1
  fi
fi

tomogram_pid=""
rviz_pid=""
stop_process_group() {
  local pid="$1"
  local signal attempt

  for signal in TERM KILL; do
    kill "-${signal}" -- "-${pid}" 2>/dev/null || true
    for attempt in {1..20}; do
      if ! kill -0 -- "-${pid}" 2>/dev/null; then
        wait "${pid}" 2>/dev/null || true
        return
      fi
      sleep 0.1
    done
  done
  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  trap - EXIT HUP INT TERM
  if [[ -n "${tomogram_pid}" ]]; then
    stop_process_group "${tomogram_pid}"
  fi
  if [[ -n "${rviz_pid}" ]]; then
    stop_process_group "${rviz_pid}"
  fi
}
trap cleanup EXIT HUP INT TERM

if is_true "${START_PCT_TOMOGRAM}"; then
  preview_pose_args=()
  goal_editor_map_args=()
  visualization_filter_args=(
    --minimum-component-cells "${PCT_VISUALIZATION_MIN_COMPONENT_CELLS}"
  )
  if [[ -n "${PCT_VISUALIZATION_COST_THRESHOLD}" ]]; then
    visualization_filter_args+=(
      --visualization-cost-threshold "${PCT_VISUALIZATION_COST_THRESHOLD}"
    )
  fi
  if [[ -n "${PCT_DISPLAY_MIN_Z_OFFSET}" ]]; then
    visualization_filter_args+=(
      --display-min-z-offset "${PCT_DISPLAY_MIN_Z_OFFSET}"
    )
  fi
  if [[ -n "${PCT_DISPLAY_MAX_Z_OFFSET}" ]]; then
    visualization_filter_args+=(
      --display-max-z-offset "${PCT_DISPLAY_MAX_Z_OFFSET}"
    )
  fi
  if is_true "${PCT_HIDE_TERMINAL_ENVELOPE}"; then
    visualization_filter_args+=(--hide-terminal-envelope)
  fi
  if [[ -n "${PCT_GOAL_EDITOR_TOMOGRAM}" ]]; then
    goal_editor_map_args=(
      --goal-editor-tomogram "${PCT_GOAL_EDITOR_TOMOGRAM}"
    )
  fi
  if [[ -n "${PCT_PREVIEW_POSE}" ]]; then
    read -r -a preview_pose_values <<<"${PCT_PREVIEW_POSE}"
    if [[ ${#preview_pose_values[@]} -ne 3 ]]; then
      echo "PCT_PREVIEW_POSE must contain exactly: X Y Z" >&2
      exit 2
    fi
    preview_pose_args=(--initial-pose "${preview_pose_values[@]}")
  fi
  echo "PCT static map: ${PCT_TOMOGRAM} -> ${PCT_TOMOGRAM_TOPIC}"
  if [[ -n "${PCT_GOAL_EDITOR_TOMOGRAM}" ]]; then
    echo "PCT goal-editor map: ${PCT_GOAL_EDITOR_TOMOGRAM}"
  fi
  echo "NX live cloud display is disabled."
  echo "Display cell deletion filters are disabled unless explicitly configured."
  echo "Goal: Publish Point -> click map -> Interact -> right-click orange ball."
  setsid "${PYTHON_EXECUTABLE}" "${TOMOGRAM_PUBLISHER}" \
    --tomogram "${PCT_TOMOGRAM}" \
    --topic "${PCT_TOMOGRAM_TOPIC}" \
    --pose-topic /pose_stamped \
    --frame-id map \
    --grid-stride "${PCT_TOMOGRAM_STRIDE}" \
    "${visualization_filter_args[@]}" \
    "${goal_editor_map_args[@]}" \
    "${preview_pose_args[@]}" 9>&- &
  tomogram_pid=$!
else
  echo "PCT static map: receiving ${PCT_TOMOGRAM_CHUNK_TOPIC} from NX."
  echo "Local reassembled map topic: ${PCT_TOMOGRAM_TOPIC}."
  echo "PCT goal editor: local Foxy process (no cross-version InteractiveMarker)."
  echo "Goal: Publish Point -> click map -> Interact -> right-click orange ball."
  echo "Local pickle loading is disabled; start NX with PUBLISH_PCT_TOMOGRAM=true."
  setsid "${PYTHON_EXECUTABLE}" "${TOMOGRAM_RECEIVER}" \
    --chunk-topic "${PCT_TOMOGRAM_CHUNK_TOPIC}" \
    --output-topic "${PCT_TOMOGRAM_TOPIC}" 9>&- &
  tomogram_pid=$!
fi

echo "Relocalization: enable 'FAST-LIO Map (enable for relocalization)', then use '2D Pose Estimate' (/initialpose)."
echo "Yellow arrow: IMU/body reference pose; green arrow: visual-only ground projection."
echo "ICP check: gray local map and red corrected live scan should overlap."

setsid rviz2 -d "${RVIZ_CONFIG}" 9>&- &
rviz_pid=$!

set +e
wait -n "${tomogram_pid}" "${rviz_pid}"
status=$?
set -e
if ! kill -0 "${tomogram_pid}" 2>/dev/null; then
  echo "Local PCT map receiver exited; closing RViz (status=${status})." >&2
fi
exit "${status}"
