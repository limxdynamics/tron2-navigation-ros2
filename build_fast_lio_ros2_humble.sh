#!/usr/bin/env bash
# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
OUTPUT_ROOT="${FAST_LIO_BUILD_ROOT:-${ROOT_DIR}/build/native_fast_lio}"
RSLIDAR_SDK_DIR="${RSLIDAR_SDK_DIR:-${ROOT_DIR}/RSLIDAR_SDK_ROS2}"
RSLIDAR_MSG_DIR="${RSLIDAR_MSG_DIR:-${ROOT_DIR}/RSLIDAR_MSG_ROS2}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS 2 Humble setup not found: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${ROOT_DIR}/FAST_LIO/package.xml" ]]; then
  echo "FAST_LIO source not found: ${ROOT_DIR}/FAST_LIO" >&2
  exit 1
fi
if [[ ! -f "${ROOT_DIR}/FAST_LIO_LOCALIZATION2/package.xml" ]]; then
  echo "FAST_LIO_LOCALIZATION2 source not found: ${ROOT_DIR}/FAST_LIO_LOCALIZATION2" >&2
  exit 1
fi
if [[ ! -f "${RSLIDAR_SDK_DIR}/package.xml" || ! -f "${RSLIDAR_SDK_DIR}/src/rs_driver/CMakeLists.txt" ]]; then
  echo "Official rslidar_sdk (including rs_driver submodule) not found: ${RSLIDAR_SDK_DIR}" >&2
  exit 1
fi
if [[ ! -f "${RSLIDAR_MSG_DIR}/package.xml" ]]; then
  echo "Official rslidar_msg source not found: ${RSLIDAR_MSG_DIR}" >&2
  exit 1
fi

# The old jerett/0826 overlay contains an MROS fast_lio package with a colliding
# name. Keep it out of this native ROS 2 overlay to avoid a mixed install tree.
case ":${AMENT_PREFIX_PATH:-}:${CMAKE_PREFIX_PATH:-}:" in
  *jerett/0826/install*)
    echo "The old MROS jerett/0826 environment is active." >&2
    echo "Build native FAST-LIO in a clean shell; do not source the MROS setup first." >&2
    exit 1
    ;;
esac

set +u
source "${ROS_SETUP}"
set -u
if [[ "${ROS_VERSION:-}" != "2" || "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "Expected ROS 2 Humble after sourcing ${ROS_SETUP}; got ROS_VERSION=${ROS_VERSION:-unset}, ROS_DISTRO=${ROS_DISTRO:-unset}." >&2
  exit 1
fi

mkdir -p "${OUTPUT_ROOT}"
PYTHONDONTWRITEBYTECODE=1 colcon --log-base "${OUTPUT_ROOT}/log" build \
  --base-paths \
    "${RSLIDAR_MSG_DIR}" \
    "${RSLIDAR_SDK_DIR}" \
    "${ROOT_DIR}/FAST_LIO" \
    "${ROOT_DIR}/FAST_LIO_LOCALIZATION2" \
  --packages-select rslidar_msg rslidar_sdk fast_lio fast_lio_localization \
  --build-base "${OUTPUT_ROOT}/build" \
  --install-base "${OUTPUT_ROOT}/install" \
  --merge-install \
  --symlink-install \
  --cmake-force-configure \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Native FAST-LIO overlay built: ${OUTPUT_ROOT}/install/setup.bash"
echo "Native ROS 2 RoboSense driver: rslidar_sdk v1.5.20, RSFAIRY, XYZIRT + IMU."
echo "Default sensor topics: /rslidar_points + /rslidar_imu_data."
echo "Runtime Python dependencies still required: open3d, transforms3d, numpy<1.24"
