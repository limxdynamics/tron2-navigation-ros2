#!/usr/bin/env bash
# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ -f "${ROOT_DIR}/build/permissive_scan/install/setup.bash" ]]; then
  DEFAULT_SCAN_SETUP="${ROOT_DIR}/build/permissive_scan/install/setup.bash"
else
  DEFAULT_SCAN_SETUP="${ROOT_DIR}/SCAN-Planner-ros2-community/install/setup.bash"
fi
SCAN_SETUP="${SCAN_SETUP:-${DEFAULT_SCAN_SETUP}}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-26}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

launcher_patterns=(
  '[r]un_fast_lio_navigation_humble[.]sh'
  '[r]un_nx_navigation_humble[.]sh'
)
runtime_patterns=(
  # Match only the native ROS 2 driver from this deployment. Never signal the
  # root-owned /opt/limx driver managed by camera-mros.service.
  "${ROOT_DIR}/build/native_fast_lio/install/lib/rslidar_sdk/[r]slidar_sdk_node"
  '[f]astlio_mapping'
  '[l]aser_mapping'
  '[l]ocalization[.]launch[.]py'
  '[g]lobal_localization'
  '[t]ransform_fusion'
  '[p]cd_to_pointcloud'
  '[n]avigation_plan[.]py'
  '[p]ct_planner'
  '[p]ublish_tomogram_from_pickle[.]py'
  '[s]can_planner_node'
  '[p]ose_stamped_to_odometry'
  '[c]losed_loop_controller'
  '[m]ros_ros2_bridge'
  '[l]imx_websocket_cmd_bridge'
  '[n]x_pct_navigation[.]launch[.]py'
)

protected_pids=()
ancestor_pid="$$"
while [[ "${ancestor_pid}" =~ ^[0-9]+$ && "${ancestor_pid}" -gt 1 ]]; do
  protected_pids+=("${ancestor_pid}")
  ancestor_pid="$(ps -o ppid= -p "${ancestor_pid}" 2>/dev/null | tr -d '[:space:]')"
done

is_protected_pid() {
  local candidate="$1"
  local protected_pid
  for protected_pid in "${protected_pids[@]}"; do
    [[ "${candidate}" == "${protected_pid}" ]] && return 0
  done
  return 1
}

collect_pids() {
  local pattern
  local pid
  while read -r pid; do
    [[ -n "${pid}" ]] || continue
    is_protected_pid "${pid}" || printf '%s\n' "${pid}"
  done < <(
    for pattern in "$@"; do
      pgrep -f -- "${pattern}" 2>/dev/null || true
    done | sort -nu
  )
}

signal_matches() {
  local signal="$1"
  shift
  local pid
  while read -r pid; do
    [[ -n "${pid}" ]] || continue
    is_protected_pid "${pid}" && continue
    kill "-${signal}" "${pid}" 2>/dev/null || true
  done < <(collect_pids "$@")
}

wait_until_stopped() {
  local attempts="$1"
  shift
  local attempt
  for ((attempt = 0; attempt < attempts; ++attempt)); do
    [[ -z "$(collect_pids "$@")" ]] && return 0
    sleep 0.1
  done
  return 1
}

publish_safe_stop() {
  [[ -f "${ROS_SETUP}" ]] || return 1
  set +u
  source "${ROS_SETUP}"
  if [[ -f "${SCAN_SETUP}" ]]; then
    source "${SCAN_SETUP}"
  fi
  set -u

  timeout 2s ros2 topic pub -r 20 \
    /scan_planner/pause std_msgs/msg/Bool '{data: true}' \
    >/dev/null 2>&1 || true
  timeout 2s ros2 topic pub -r 20 \
    /sdk_cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
    >/dev/null 2>&1 || true
}

if publish_safe_stop; then
  echo "Pause and repeated zero velocity sent on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}."
else
  echo "ROS setup unavailable; proceeding with process cleanup." >&2
fi

# Ask the launcher shells to run their own process-group cleanup first.
signal_matches TERM "${launcher_patterns[@]}"
wait_until_stopped 120 "${launcher_patterns[@]}" || true

# Clean up an orphaned partial stack left by an interrupted SSH session.
signal_matches TERM "${runtime_patterns[@]}"
wait_until_stopped 80 "${runtime_patterns[@]}" || true

remaining="$(collect_pids "${launcher_patterns[@]}" "${runtime_patterns[@]}")"
if [[ -n "${remaining}" ]]; then
  echo "Force-killing remaining navigation processes: ${remaining//$'\n'/ }" >&2
  signal_matches KILL "${launcher_patterns[@]}" "${runtime_patterns[@]}"
  wait_until_stopped 20 "${launcher_patterns[@]}" "${runtime_patterns[@]}" || true
fi

remaining="$(collect_pids "${launcher_patterns[@]}" "${runtime_patterns[@]}")"
if [[ -n "${remaining}" ]]; then
  echo "Navigation cleanup failed; remaining PIDs: ${remaining//$'\n'/ }" >&2
  exit 1
fi

echo "NX native navigation stopped; no matching launcher/controller/bridge remains."
