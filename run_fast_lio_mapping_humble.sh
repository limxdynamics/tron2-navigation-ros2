#!/usr/bin/env bash
# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
FAST_LIO_ROS2_SETUP="${FAST_LIO_ROS2_SETUP:-${ROOT_DIR}/build/native_fast_lio/install/setup.bash}"
RSLIDAR_CONFIG="${RSLIDAR_CONFIG:-${ROOT_DIR}/RSLIDAR_SDK_ROS2/config/config.yaml}"
MAPPING_LOCALIZATION_MAX_DISTANCE="${MAPPING_LOCALIZATION_MAX_DISTANCE:-100}"
MAPPING_REGISTRATION_MAX_DISTANCE="${MAPPING_REGISTRATION_MAX_DISTANCE:-15}"
MAPPING_PCT_MAX_DISTANCE="${MAPPING_PCT_MAX_DISTANCE:-15}"
MAPPING_CONFIG_PATH="${MAPPING_CONFIG_PATH:-${ROOT_DIR}/FAST_LIO/config}"
MAPPING_CONFIG_FILE="${MAPPING_CONFIG_FILE:-rs_fairy.yaml}"
MAP_OUTPUT="${MAP_OUTPUT:-${ROOT_DIR}/FAST_LIO/PCD/rs_fairy_map.pcd}"
PCT_MAP_OUTPUT="${PCT_MAP_OUTPUT:-${MAP_OUTPUT%.pcd}.pct-source.pcd}"
RVIZ="${RVIZ:-false}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-26}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

for required_file in \
  "${ROS_SETUP}" \
  "${FAST_LIO_ROS2_SETUP}" \
  "${RSLIDAR_CONFIG}" \
  "${MAPPING_CONFIG_PATH}/${MAPPING_CONFIG_FILE}"
do
  if [[ ! -f "${required_file}" ]]; then
    echo "Required file not found: ${required_file}" >&2
    exit 1
  fi
done

for distance_name in \
  MAPPING_LOCALIZATION_MAX_DISTANCE \
  MAPPING_REGISTRATION_MAX_DISTANCE \
  MAPPING_PCT_MAX_DISTANCE
do
  distance_value="${!distance_name}"
  [[ "${distance_value}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] &&
    awk -v value="${distance_value}" 'BEGIN { exit !(value > 0.2 && value <= 200.0) }' || {
      echo "${distance_name} must be greater than 0.2 and no more than 200 m." >&2
      exit 2
    }
done
awk -v registration="${MAPPING_REGISTRATION_MAX_DISTANCE}" \
    -v localization="${MAPPING_LOCALIZATION_MAX_DISTANCE}" \
    'BEGIN { exit !(registration <= localization) }' || {
  echo "MAPPING_REGISTRATION_MAX_DISTANCE must not exceed MAPPING_LOCALIZATION_MAX_DISTANCE." >&2
  exit 2
}
awk -v pct="${MAPPING_PCT_MAX_DISTANCE}" \
    -v localization="${MAPPING_LOCALIZATION_MAX_DISTANCE}" \
    'BEGIN { exit !(pct <= localization) }' || {
  echo "MAPPING_PCT_MAX_DISTANCE must not exceed MAPPING_LOCALIZATION_MAX_DISTANCE." >&2
  exit 2
}
[[ "$(realpath -m "${MAP_OUTPUT}")" != "$(realpath -m "${PCT_MAP_OUTPUT}")" ]] || {
  echo "MAP_OUTPUT and PCT_MAP_OUTPUT must be different files." >&2
  exit 2
}
if [[ -n "${MAPPING_LIDAR_MAX_DISTANCE:-}" ]]; then
  echo "Warning: MAPPING_LIDAR_MAX_DISTANCE is deprecated and ignored; use MAPPING_LOCALIZATION_MAX_DISTANCE." >&2
fi

runtime_config="${TMPDIR:-/tmp}/tron2-navigation-rslidar-mapping-${UID}.yaml"
temporary_config="${runtime_config}.new.$$"
if ! awk -v limit="${MAPPING_LOCALIZATION_MAX_DISTANCE}" '
  !replaced && /^[[:space:]]*max_distance:[[:space:]]*/ {
    indent=$0
    sub(/[^[:space:]].*$/, "", indent)
    print indent "max_distance: " limit "  # Full localization-map range (m)"
    replaced=1
    next
  }
  { print }
  END { if (!replaced) exit 42 }
' "${RSLIDAR_CONFIG}" > "${temporary_config}"; then
  rm -f "${temporary_config}"
  echo "Could not create mapping-only RoboSense config." >&2
  exit 1
fi
chmod 600 "${temporary_config}"
mv -f "${temporary_config}" "${runtime_config}"
RSLIDAR_CONFIG="${runtime_config}"

if pgrep -f '[r]slidar_sdk_node' >/dev/null 2>&1; then
  echo "An rslidar_sdk_node is already running." >&2
  echo "Stop the old MROS driver before starting the native ROS 2 driver; both use the same UDP ports." >&2
  exit 1
fi
mkdir -p "$(dirname "${MAP_OUTPUT}")" "$(dirname "${PCT_MAP_OUTPUT}")"

set +u
source "${ROS_SETUP}"
source "${FAST_LIO_ROS2_SETUP}"
set -u

if ! ros2 pkg prefix fast_lio >/dev/null 2>&1; then
  echo "fast_lio is not visible after sourcing ${FAST_LIO_ROS2_SETUP}" >&2
  exit 1
fi
if ! ros2 pkg prefix rslidar_sdk >/dev/null 2>&1; then
  echo "Native ROS 2 rslidar_sdk is not visible after sourcing ${FAST_LIO_ROS2_SETUP}" >&2
  exit 1
fi

pids=()
cleaned_up=false
cleanup() {
  if [[ "${cleaned_up}" == true ]]; then
    return
  fi
  cleaned_up=true
  trap - INT TERM EXIT
  for pid in "${pids[@]:-}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  for pid in "${pids[@]:-}"; do
    wait "${pid}" 2>/dev/null || true
  done
}
trap cleanup INT TERM EXIT

echo "Starting RoboSense FAST-LIO mapping"
echo "  native ROS 2 driver config: ${RSLIDAR_CONFIG}"
echo "  localization observation range: ${MAPPING_LOCALIZATION_MAX_DISTANCE} m"
echo "  scan-to-map registration range: ${MAPPING_REGISTRATION_MAX_DISTANCE} m"
echo "  PCT source per-frame range: ${MAPPING_PCT_MAX_DISTANCE} m"
echo "  ROS 2 input: /rslidar_points + /rslidar_imu_data"
echo "  config: ${MAPPING_CONFIG_PATH}/${MAPPING_CONFIG_FILE}"
echo "  localization map output: ${MAP_OUTPUT}"
echo "  PCT source map output: ${PCT_MAP_OUTPUT}"
echo "Call /map_save (std_srvs/srv/Trigger) before stopping to write both map outputs."

ros2 run rslidar_sdk rslidar_sdk_node --ros-args \
  -p config_path:="${RSLIDAR_CONFIG}" &
pids+=("$!")

ros2 launch fast_lio mapping.launch.py \
  config_path:="${MAPPING_CONFIG_PATH}" \
  config_file:="${MAPPING_CONFIG_FILE}" \
  map_file_path:="${MAP_OUTPUT}" \
  pct_map_file_path:="${PCT_MAP_OUTPUT}" \
  pct_max_range:="${MAPPING_PCT_MAX_DISTANCE}" \
  registration_max_range:="${MAPPING_REGISTRATION_MAX_DISTANCE}" \
  rviz:="${RVIZ}" &
pids+=("$!")

set +e
wait -n "${pids[@]}"
status=$?
set -e
cleanup
exit "${status}"
