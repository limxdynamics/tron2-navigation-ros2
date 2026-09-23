#!/usr/bin/env bash
# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
JOBS="${BUILD_JOBS:-2}"
BOOTSTRAP=false
RUN_TESTS=false
PREFLIGHT_ONLY=false

usage() {
  cat <<'EOF'
用法: ./install.sh nx [选项]

  --bootstrap       在全新 Ubuntu 22.04 aarch64 NX 上安装 ROS 2/系统依赖
  --jobs N          编译并行数，默认 2
  --with-tests      构建后运行 RoboSense 与 SCAN 测试
  --preflight-only  只读检查源码和环境，不安装、不构建
  -h, --help        显示帮助

本安装器只构建仓库内的 BSD/Apache 源码。它不会下载或构建外部 GPL
组件，不会启动导航、连接 LimX 底盘或发送速度命令。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bootstrap) BOOTSTRAP=true; shift ;;
    --jobs)
      [[ $# -ge 2 ]] || { echo "--jobs 缺少数值" >&2; exit 2; }
      JOBS="$2"
      shift 2
      ;;
    --with-tests) RUN_TESTS=true; shift ;;
    --preflight-only) PREFLIGHT_ONLY=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "${JOBS}" =~ ^[1-9][0-9]*$ ]] || {
  echo "--jobs 必须是正整数" >&2
  exit 2
}
if [[ "${PREFLIGHT_ONLY}" == true && "${BOOTSTRAP}" == true ]]; then
  echo "--preflight-only 不能与 --bootstrap 组合使用。" >&2
  exit 2
fi

required_source_files=(
  LICENSE
  LICENSING.md
  THIRD_PARTY_NOTICES.md
  SOURCE_INVENTORY.sha256
  config/navigation.env.example
  docs/GPL_EXTERNAL_DEPENDENCIES.md
  RSLIDAR_MSG_ROS2/package.xml
  RSLIDAR_SDK_ROS2/package.xml
  RSLIDAR_SDK_ROS2/src/rs_driver/CMakeLists.txt
  SCAN-Planner-ros2-community/src/planner/plan_manage/package.xml
)
for relative_path in "${required_source_files[@]}"; do
  [[ -f "${ROOT_DIR}/${relative_path}" ]] || {
    echo "宽松源码文件缺失: ${relative_path}" >&2
    exit 1
  }
done

for forbidden_root in FAST_LIO FAST_LIO_LOCALIZATION2 PCT_planner-RC2026_Map_Planner PCT_planner; do
  if [[ -e "${ROOT_DIR}/${forbidden_root}" || -L "${ROOT_DIR}/${forbidden_root}" ]]; then
    echo "宽松源码树中禁止包含 GPL 目录: ${forbidden_root}" >&2
    exit 1
  fi
done

if ! (cd "${ROOT_DIR}" && sha256sum --quiet -c SOURCE_INVENTORY.sha256); then
  echo "源码清单校验失败；拒绝在被修改或不完整的源码上安装。" >&2
  exit 1
fi
echo "PERMISSIVE_SOURCE_INVENTORY=PASS"

[[ -f /etc/os-release ]] || { echo "无法识别操作系统。" >&2; exit 1; }
source /etc/os-release
target_arch="$(uname -m)"
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 22.04 || "${target_arch}" != aarch64 ]]; then
  echo "仅支持 Ubuntu 22.04 aarch64；当前 ${ID:-unknown} ${VERSION_ID:-unknown} ${target_arch}。" >&2
  [[ "${ALLOW_UNSUPPORTED_TARGET:-0}" == 1 ]] || exit 1
  echo "ALLOW_UNSUPPORTED_TARGET=1：仅供安装器测试，继续执行。" >&2
fi

bootstrap_system_dependencies() {
  echo "[初始化] 安装 ROS 2 Humble 和宽松子集依赖"
  sudo -v
  sudo apt-get update
  sudo apt-get install -y \
    ca-certificates curl file gnupg2 locales lsb-release software-properties-common
  sudo add-apt-repository -y universe

  if [[ ! -f "${ROS_SETUP}" ]]; then
    sudo install -d -m 0755 /usr/share/keyrings
    sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" | \
      sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    sudo apt-get update
    sudo apt-get install -y ros-humble-ros-base
  fi

  sudo apt-get install -y \
    build-essential cmake git pkg-config rsync file \
    python3-pytest python3-websocket python3-yaml python3-rosdep \
    python3-colcon-common-extensions libeigen3-dev libarmadillo-dev \
    libpcap-dev libyaml-cpp-dev libpcl-dev \
    ros-humble-pcl-ros ros-humble-rmw-fastrtps-cpp

  set +u
  source "${ROS_SETUP}"
  set -u
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  rosdep update
  rosdep install --rosdistro humble --from-paths \
    "${ROOT_DIR}/RSLIDAR_MSG_ROS2" \
    "${ROOT_DIR}/RSLIDAR_SDK_ROS2" \
    "${ROOT_DIR}/SCAN-Planner-ros2-community/src/planner" \
    --ignore-src -r -y
}

preflight_environment() {
  local missing=()
  local executable
  for executable in cmake ctest gcc g++ git colcon pkg-config rsync sha256sum file; do
    command -v "${executable}" >/dev/null 2>&1 || missing+=("command:${executable}")
  done
  for required_file in \
    "${ROS_SETUP}" \
    /usr/include/eigen3/Eigen/Core \
    /usr/include/yaml-cpp/yaml.h; do
    [[ -e "${required_file}" ]] || missing+=("file:${required_file}")
  done
  if ! PYTHONNOUSERSITE=1 /usr/bin/python3 - "${RUN_TESTS}" <<'PY'
import importlib
import sys

modules = ["yaml", "websocket"]
if sys.argv[1] == "true":
    modules.append("pytest")
missing = []
for name in modules:
    try:
        importlib.import_module(name)
    except Exception as exc:
        missing.append(f"{name} ({exc})")
if missing:
    print("Python 模块不可用: " + ", ".join(missing), file=sys.stderr)
    raise SystemExit(1)
PY
  then
    if [[ "${RUN_TESTS}" == true ]]; then
      missing+=("python:yaml websocket pytest")
    else
      missing+=("python:yaml websocket")
    fi
  fi
  if (( ${#missing[@]} > 0 )); then
    echo "NX 环境预检失败：" >&2
    printf '  - %s\n' "${missing[@]}" >&2
    exit 1
  fi

  set +u
  source "${ROS_SETUP}"
  set -u
  [[ "${ROS_VERSION:-}" == 2 && "${ROS_DISTRO:-}" == humble ]] || {
    echo "期望 ROS 2 Humble，实际 ROS_VERSION=${ROS_VERSION:-unset} ROS_DISTRO=${ROS_DISTRO:-unset}。" >&2
    exit 1
  }
  echo "NX_PERMISSIVE_ENVIRONMENT_PREFLIGHT=PASS"
}

run_ctest_if_present() {
  local directory="$1"
  local label="$2"
  [[ -d "${directory}" ]] || { echo "测试目录不存在: ${directory}" >&2; return 1; }
  ctest --test-dir "${directory}" --output-on-failure
  echo "${label}_TESTS=PASS"
}

verify_dynamic_dependencies() {
  local executable="$1"
  local output
  [[ -x "${executable}" ]] || { echo "关键程序未生成: ${executable}" >&2; return 1; }
  output="$(ldd "${executable}")"
  if grep -q 'not found' <<<"${output}"; then
    printf '%s\n' "${output}" >&2
    return 1
  fi
}

if [[ "${BOOTSTRAP}" == true ]]; then
  echo "INSTALL_MODE=bootstrap-permissive-subset"
  bootstrap_system_dependencies
else
  echo "INSTALL_MODE=prepared-permissive-subset"
fi

[[ -f "${ROS_SETUP}" ]] || {
  echo "ROS 2 Humble 不存在；全新 NX 请运行 ./install.sh nx --bootstrap。" >&2
  exit 1
}
preflight_environment

if [[ "${PREFLIGHT_ONLY}" == true ]]; then
  echo "PERMISSIVE_SOURCE_PREFLIGHT_ONLY=PASS"
  exit 0
fi

set +u
source "${ROS_SETUP}"
set -u

rslidar_output="${ROOT_DIR}/build/permissive_rslidar"
scan_output="${ROOT_DIR}/build/permissive_scan"
rm -rf "${rslidar_output}" "${scan_output}"

printf '[1/2] 构建 RoboSense BSD-3-Clause 包\n'
PYTHONDONTWRITEBYTECODE=1 colcon --log-base "${rslidar_output}/log" build \
  --base-paths "${ROOT_DIR}/RSLIDAR_MSG_ROS2" "${ROOT_DIR}/RSLIDAR_SDK_ROS2" \
  --packages-select rslidar_msg rslidar_sdk \
  --build-base "${rslidar_output}/build" \
  --install-base "${rslidar_output}/install" \
  --merge-install --symlink-install --cmake-force-configure \
  --parallel-workers "${JOBS}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

printf '[2/2] 构建 SCAN Apache-2.0 生产子集\n'
PYTHONDONTWRITEBYTECODE=1 colcon --log-base "${scan_output}/log" build \
  --base-paths "${ROOT_DIR}/SCAN-Planner-ros2-community/src/planner" \
  --packages-up-to scan_planner \
  --build-base "${scan_output}/build" \
  --install-base "${scan_output}/install" \
  --symlink-install --parallel-workers "${JOBS}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

verify_dynamic_dependencies "${rslidar_output}/install/lib/rslidar_sdk/rslidar_sdk_node"
scan_bin="${scan_output}/install/scan_planner/lib/scan_planner"
for executable in scan_planner_node closed_loop_controller pose_stamped_to_odometry; do
  verify_dynamic_dependencies "${scan_bin}/${executable}"
done
[[ -x "${scan_bin}/limx_websocket_cmd_bridge" ]] || {
  echo "LimX legacy WebSocket gateway 未安装。" >&2
  exit 1
}
if find "${scan_output}/install" -type f \
  \( -name mros_ros2_bridge -o -name limx_websocket_mode_probe \) \
  -print -quit | grep -q .; then
  echo "检测到已停用的 MROS 桥或协议探针。" >&2
  exit 1
fi
echo "PERMISSIVE_RUNTIME_BUILD_VERIFIED=PASS"

if [[ "${RUN_TESTS}" == true ]]; then
  set +u
  source "${rslidar_output}/install/setup.bash"
  source "${scan_output}/install/setup.bash"
  set -u
  run_ctest_if_present "${rslidar_output}/build/rslidar_msg" RSLIDAR_MSG
  run_ctest_if_present "${rslidar_output}/build/rslidar_sdk" RSLIDAR_SDK
  for package in scan_planner_msgs plan_env path_searching bspline_opt traj_utils scan_planner; do
    run_ctest_if_present "${scan_output}/build/${package}" "SCAN_${package}"
  done
  echo "PERMISSIVE_SOURCE_TESTS=PASS"
fi

if ! (cd "${ROOT_DIR}" && sha256sum --quiet -c SOURCE_INVENTORY.sha256); then
  echo "构建过程修改了受保护源码。" >&2
  exit 1
fi
echo "POST_BUILD_SOURCE_INVENTORY=PASS"

echo
echo "PERMISSIVE_SOURCE_INSTALL=PASS"
echo "只完成 RoboSense 与 SCAN 构建；未下载 GPL 组件，未启动导航，未连接底盘。"
echo "完整导航的外部 GPL 和硬件要求见 docs/GPL_EXTERNAL_DEPENDENCIES.md 与 docs/HARDWARE_COMPATIBILITY.md。"
