#!/usr/bin/env bash
# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FOXY_SETUP="${FOXY_SETUP:-/opt/ros/foxy/setup.bash}"
RVIZ_CONFIG="${RVIZ_CONFIG:-${ROOT_DIR}/FAST_LIO/rviz/fastlio.rviz}"
LOCK_FILE="${XDG_RUNTIME_DIR:-/tmp}/navigation-fast-lio-mapping-rviz-foxy-${UID}.lock"

if [[ ! -f "${FOXY_SETUP}" ]]; then
  echo "ROS 2 Foxy setup not found: ${FOXY_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${RVIZ_CONFIG}" ]]; then
  echo "FAST-LIO RViz configuration not found: ${RVIZ_CONFIG}" >&2
  exit 1
fi
if ! command -v flock >/dev/null 2>&1; then
  echo "Required command not found: flock" >&2
  exit 1
fi

exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  echo "FAST-LIO mapping RViz is already running; close it first." >&2
  exit 1
fi
if pgrep -x rviz2 >/dev/null 2>&1; then
  echo "Another rviz2 process is already running; close it first." >&2
  exit 1
fi

# Avoid mixing the user's default ROS 1 Noetic shell with ROS 2 Foxy.
unset ROS_MASTER_URI ROS_ROOT ROS_PACKAGE_PATH ROS_ETC_DIR
unset ROS_VERSION ROS_PYTHON_VERSION ROS_DISTRO
unset AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH COLCON_CURRENT_PREFIX
unset PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH
unset FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI

set +u
source "${FOXY_SETUP}"
set -u

export ROS_VERSION=2
export ROS_PYTHON_VERSION=3
export ROS_DISTRO=foxy
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-26}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

echo "Starting low-bandwidth FAST-LIO mapping RViz"
echo "  config=${RVIZ_CONFIG}"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "  Point clouds use Best Effort/depth 1; the large /Laser_map display is disabled by default."
echo "  CloudRegistered accumulates 30 seconds of registered scans for live mapping."

exec rviz2 -d "${RVIZ_CONFIG}"
