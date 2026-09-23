#!/usr/bin/env bash
# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
NAVIGATION_ENV_FILE="${NAVIGATION_ENV_FILE:-${ROOT_DIR}/config/navigation.env}"
if [[ -r "${NAVIGATION_ENV_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${NAVIGATION_ENV_FILE}"
fi
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
JOBS="${BUILD_JOBS:-2}"
BOOTSTRAP=false
INSTALL_MAP_TOOLS=false
MAP_TOOLS_EXPLICIT=false
RUN_TESTS=false
PREFLIGHT_ONLY=false
MAP_SITE_NEW=""

usage() {
  cat <<'EOF'
用法: ./install.sh full-nx [选项]

  --bootstrap          全新 Ubuntu 22.04 aarch64：安装完整编译/运行依赖和地图工具
  --jobs N             编译并行数，默认 2
  --with-tests         构建后运行 PCT、RoboSense、FAST-LIO 和 SCAN 测试
  --install-map-tools  在已有环境中安装隔离的 PCT/CUDA 地图生成工具
  --skip-map-tools     --bootstrap 时不安装地图生成工具
  --preflight-only     只读检查已有的三个仓库和环境，不拉取、不创建链接、不构建
  -h, --help           显示帮助

正常构建会自动拉取缺失的两个 GPL 仓库并校验固定 v1.1.0 提交。三个仓库按
GitHub 仓库名并列放置：
  tron2-navigation-ros2
  tron2-navigation-fastlio-gpl
  tron2-navigation-pct-gpl

已有路径绝不会被自动更新或切换；可通过 FAST_LIO_GPL_ROOT、PCT_GPL_ROOT
指定其他克隆路径。
安装和测试不会启动导航、连接底盘或发送速度命令。
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
    --install-map-tools)
      INSTALL_MAP_TOOLS=true
      MAP_TOOLS_EXPLICIT=true
      shift
      ;;
    --skip-map-tools)
      INSTALL_MAP_TOOLS=false
      MAP_TOOLS_EXPLICIT=true
      shift
      ;;
    --preflight-only) PREFLIGHT_ONLY=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "${JOBS}" =~ ^[1-9][0-9]*$ ]] || {
  echo "--jobs 必须是正整数" >&2
  exit 2
}
if [[ "${BOOTSTRAP}" == true && "${MAP_TOOLS_EXPLICIT}" != true ]]; then
  INSTALL_MAP_TOOLS=true
fi
if [[ "${PREFLIGHT_ONLY}" == true && \
      ( "${BOOTSTRAP}" == true || "${INSTALL_MAP_TOOLS}" == true ) ]]; then
  echo "--preflight-only 不能与 --bootstrap 或 --install-map-tools 组合。" >&2
  exit 2
fi

cleanup() {
  if [[ -n "${MAP_SITE_NEW}" && -e "${MAP_SITE_NEW}" ]]; then
    rm -rf "${MAP_SITE_NEW}"
  fi
}
trap cleanup EXIT

for relative_path in \
  SOURCE_INVENTORY.sha256 \
  config/navigation.env.example \
  deployment/map-tools-constraints.txt \
  deployment/link_external_gpl_sources.sh \
  build_fast_lio_ros2_humble.sh \
  stop_nx_navigation_humble.sh \
  SCAN-Planner-ros2-community/src/planner/plan_manage/package.xml; do
  [[ -f "${ROOT_DIR}/${relative_path}" ]] || {
    echo "完整导航安装文件缺失: ${relative_path}" >&2
    exit 1
  }
done
if ! (cd "${ROOT_DIR}" && sha256sum --quiet -c SOURCE_INVENTORY.sha256); then
  echo "permissive 源码清单校验失败。" >&2
  exit 1
fi
external_source_options=(--verify-only)
if [[ "${PREFLIGHT_ONLY}" != true ]]; then
  external_source_options+=(--fetch)
fi
bash "${SCRIPT_DIR}/link_external_gpl_sources.sh" "${external_source_options[@]}"

[[ -f /etc/os-release ]] || { echo "无法识别操作系统。" >&2; exit 1; }
source /etc/os-release
target_arch="$(uname -m)"
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 22.04 || \
      "${target_arch}" != aarch64 ]]; then
  echo "仅支持 Ubuntu 22.04 aarch64；当前 ${ID:-unknown} ${VERSION_ID:-unknown} ${target_arch}。" >&2
  [[ "${ALLOW_UNSUPPORTED_TARGET:-0}" == 1 ]] || exit 1
  echo "ALLOW_UNSUPPORTED_TARGET=1：仅供安装器测试，继续执行。" >&2
fi

sibling_root="$(dirname "${ROOT_DIR}")"
FAST_LIO_GPL_ROOT="${FAST_LIO_GPL_ROOT:-${sibling_root}/tron2-navigation-fastlio-gpl}"
PCT_GPL_ROOT="${PCT_GPL_ROOT:-${sibling_root}/tron2-navigation-pct-gpl}"

bootstrap_system_dependencies() {
  echo "[初始化] 安装 ROS 2 Humble 和完整导航系统依赖"
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

  # Keep the runtime modules explicit. A previous clean-NX deployment had to
  # add Open3D, transforms3d, websocket-client and pcl_ros manually before the
  # localization, chassis gateway and PCL packages could run.
  sudo apt-get install -y \
    build-essential cmake git pkg-config rsync procps util-linux \
    python3-dev python3-pip python3-setuptools python3-numpy python3-scipy python3-pytest \
    python3-open3d python3-transforms3d python3-websocket python3-yaml \
    python3-rosdep python3-colcon-common-extensions \
    libeigen3-dev libboost-all-dev libarmadillo-dev libpcap-dev \
    libyaml-cpp-dev libtbb-dev libmetis-dev libblas-dev liblapack-dev \
    libglew-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    libpcl-dev ros-humble-pcl-ros ros-humble-rmw-fastrtps-cpp

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
    "${FAST_LIO_GPL_ROOT}/FAST_LIO" \
    "${FAST_LIO_GPL_ROOT}/FAST_LIO_LOCALIZATION2" \
    "${ROOT_DIR}/SCAN-Planner-ros2-community/src/planner" \
    --ignore-src -r -y
}

install_map_tools() {
  local map_site="${ROOT_DIR}/.map-tools/site"
  MAP_SITE_NEW="${ROOT_DIR}/.map-tools/site.new.$$"
  echo "[初始化] 安装隔离的 PCT/CUDA 地图生成工具"
  rm -rf "${MAP_SITE_NEW}"
  mkdir -p "${MAP_SITE_NEW}"
  PIP_NO_CACHE_DIR=1 PYTHONNOUSERSITE=1 /usr/bin/python3 -m pip install \
    --disable-pip-version-check \
    --target "${MAP_SITE_NEW}" \
    --constraint "${SCRIPT_DIR}/map-tools-constraints.txt" \
    cupy-cuda12x==13.6.0 rosbags==0.9.23 pypcd4==1.4.3
  PYTHONNOUSERSITE=1 PYTHONPATH="${MAP_SITE_NEW}" /usr/bin/python3 - <<'PY'
import cupy
import numpy
import pypcd4
import rosbags

if cupy.cuda.runtime.getDeviceCount() < 1:
    raise SystemExit("没有检测到可用的 CUDA 设备")
if tuple(int(part) for part in numpy.__version__.split(".")[:2]) >= (1, 24):
  raise SystemExit("隔离地图工具要求 NumPy < 1.24")
if int(cupy.arange(8, dtype=cupy.int32).sum().get()) != 28:
  raise SystemExit("CuPy/CUDA 计算自检失败")
PY
  rm -rf "${map_site}"
  mv "${MAP_SITE_NEW}" "${map_site}"
  MAP_SITE_NEW=""
  echo "MAP_TOOLS_INSTALL=PASS"
}

preflight_environment() {
  local missing=()
  local executable required_file
  for executable in cmake make ctest gcc g++ git colcon pkg-config rsync rosdep sha256sum; do
    command -v "${executable}" >/dev/null 2>&1 || missing+=("command:${executable}")
  done
  for required_file in \
    "${ROS_SETUP}" \
    /usr/include/eigen3/Eigen/Core \
    /usr/include/pcl-1.12/pcl/point_cloud.h \
    /usr/include/boost/version.hpp \
    /usr/include/armadillo \
    /usr/include/yaml-cpp/yaml.h; do
    [[ -e "${required_file}" ]] || missing+=("file:${required_file}")
  done
  local python_modules=(numpy scipy yaml websocket transforms3d open3d)
  [[ "${RUN_TESTS}" == true ]] && python_modules+=(pytest)
  if ! PYTHONNOUSERSITE=1 /usr/bin/python3 - "${python_modules[@]}" <<'PY'
import importlib
import sys

missing = []
for module_name in sys.argv[1:]:
    try:
        importlib.import_module(module_name)
    except Exception as exc:
        missing.append(f"{module_name} ({exc})")
if missing:
    print("Python 模块不可用: " + ", ".join(missing), file=sys.stderr)
    raise SystemExit(1)

import numpy
version = tuple(int(part) for part in numpy.__version__.split(".")[:2])
if version >= (1, 24):
    print(f"系统 NumPy 必须小于 1.24，当前为 {numpy.__version__}", file=sys.stderr)
    raise SystemExit(1)
PY
  then
    missing+=("python:${python_modules[*]} with numpy<1.24")
  fi
  if (( ${#missing[@]} > 0 )); then
    echo "完整环境预检失败：" >&2
    printf '  - %s\n' "${missing[@]}" >&2
    exit 1
  fi

  set +u
  source "${ROS_SETUP}"
  set -u
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ROS 2 CLI 不可用；请检查 ${ROS_SETUP}。" >&2
    exit 1
  fi
  [[ "${ROS_VERSION:-}" == 2 && "${ROS_DISTRO:-}" == humble ]] || {
    echo "期望 ROS 2 Humble，实际 ROS_VERSION=${ROS_VERSION:-unset} ROS_DISTRO=${ROS_DISTRO:-unset}。" >&2
    exit 1
  }
  local ros_package
  for ros_package in \
    ament_cmake geometry_msgs nav_msgs pcl_ros rclcpp rclpy sensor_msgs \
    std_srvs tf2_ros; do
    if ! ros2 pkg prefix "${ros_package}" >/dev/null 2>&1; then
      echo "ROS 2 Humble 包不可用: ${ros_package}" >&2
      exit 1
    fi
  done
  echo "FULL_ENVIRONMENT_PREFLIGHT=PASS"
}

verify_pct_build() {
  local planner_root="${PCT_GPL_ROOT}/planner"
  local library_root="${planner_root}/lib"
  local gtsam_install="${library_root}/3rdparty/gtsam-4.1.1/install"
  local osqp_install="${library_root}/3rdparty/osqp/install"
  local module osqp_library pct_ld_path
  [[ -f "${gtsam_install}/lib/cmake/GTSAM/GTSAMConfig.cmake" ]] || {
    echo "GTSAM 安装不完整。" >&2; return 1;
  }
  osqp_library="$(find "${osqp_install}/lib" -maxdepth 1 \
    \( -type f -o -type l \) -name 'libosqp.*' -print -quit 2>/dev/null || true)"
  [[ -n "${osqp_library}" ]] || { echo "OSQP 安装不完整。" >&2; return 1; }
  for module in a_star traj_opt ele_planner py_map_manager; do
    compgen -G "${library_root}/${module}*.so" >/dev/null || {
      echo "PCT Python 模块未生成: ${module}" >&2; return 1;
    }
  done
  [[ -f "${library_root}/libcommon_smoothing.so" ]] || {
    echo "PCT smoothing 库未生成。" >&2; return 1;
  }
  pct_ld_path="${gtsam_install}/lib:${osqp_install}/lib:${library_root}:${library_root}/build/src/common/smoothing"
  PYTHONNOUSERSITE=1 PYTHONPATH="${library_root}" \
    LD_LIBRARY_PATH="${pct_ld_path}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
    /usr/bin/python3 -c 'import a_star, ele_planner, py_map_manager, traj_opt'
  echo "PCT_BUILD_VERIFIED=PASS"
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

run_ctest_if_present() {
  local directory="$1"
  local label="$2"
  [[ -d "${directory}" ]] || { echo "测试目录不存在: ${directory}" >&2; return 1; }
  ctest --test-dir "${directory}" --output-on-failure
  echo "${label}_TESTS=PASS"
}

if [[ "${BOOTSTRAP}" == true ]]; then
  echo "INSTALL_MODE=bootstrap-full-navigation"
  bootstrap_system_dependencies
else
  echo "INSTALL_MODE=prepared-full-navigation"
fi
if [[ "${INSTALL_MAP_TOOLS}" == true ]]; then
  install_map_tools
fi
[[ -f "${ROS_SETUP}" ]] || {
  echo "ROS 2 Humble 不存在；全新系统请使用 ./install.sh full-nx --bootstrap。" >&2
  exit 1
}
preflight_environment

if [[ "${PREFLIGHT_ONLY}" == true ]]; then
  echo "FULL_NAVIGATION_PREFLIGHT_ONLY=PASS"
  exit 0
fi

bash "${SCRIPT_DIR}/link_external_gpl_sources.sh"
set +u
source "${ROS_SETUP}"
set -u
export BUILD_JOBS="${JOBS}"
export PYTHONNOUSERSITE=1

bash "${ROOT_DIR}/stop_nx_navigation_humble.sh"
echo "NAVIGATION_STOP_VERIFIED=PASS"

printf '[1/3] 构建 PCT GPL 单元\n'
(
  cd "${PCT_GPL_ROOT}/planner"
  bash build_thirdparty.sh
  bash build.sh
)
verify_pct_build

printf '[2/3] 构建 RoboSense 与 FAST-LIO GPL 单元\n'
rm -rf "${ROOT_DIR}/build/native_fast_lio"
FAST_LIO_BUILD_ROOT="${ROOT_DIR}/build/native_fast_lio" \
  bash "${ROOT_DIR}/build_fast_lio_ros2_humble.sh"

printf '[3/3] 构建 SCAN permissive 单元\n'
scan_output="${ROOT_DIR}/build/permissive_scan"
rm -rf "${scan_output}"
PYTHONDONTWRITEBYTECODE=1 colcon --log-base "${scan_output}/log" build \
  --base-paths "${ROOT_DIR}/SCAN-Planner-ros2-community/src/planner" \
  --packages-up-to scan_planner \
  --build-base "${scan_output}/build" \
  --install-base "${scan_output}/install" \
  --symlink-install --parallel-workers "${JOBS}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

fast_install="${ROOT_DIR}/build/native_fast_lio/install"
scan_install="${scan_output}/install"
scan_bin="${scan_install}/scan_planner/lib/scan_planner"
for executable in \
  "${fast_install}/lib/rslidar_sdk/rslidar_sdk_node" \
  "${fast_install}/lib/fast_lio/fastlio_mapping" \
  "${fast_install}/lib/fast_lio_localization/fastlio_mapping" \
  "${scan_bin}/scan_planner_node" \
  "${scan_bin}/closed_loop_controller" \
  "${scan_bin}/pose_stamped_to_odometry"; do
  verify_dynamic_dependencies "${executable}"
done
[[ -x "${scan_bin}/limx_websocket_cmd_bridge" ]] || {
  echo "LimX legacy WebSocket gateway 未生成。" >&2
  exit 1
}
if find "${scan_install}" -type f \
  \( -name mros_ros2_bridge -o -name limx_websocket_mode_probe \) \
  -print -quit | grep -q .; then
  echo "检测到已停用的 MROS 桥或协议探针。" >&2
  exit 1
fi
echo "FULL_RUNTIME_BUILD_VERIFIED=PASS"

if [[ "${RUN_TESTS}" == true ]]; then
  # Launch tests resolve packages through the ament index. Source both freshly
  # built overlays before invoking CTest so scan_planner and its generated
  # message dependencies are discoverable instead of searching only Humble.
  set +u
  source "${fast_install}/setup.bash"
  source "${scan_install}/setup.bash"
  set -u
  for package in rslidar_msg rslidar_sdk fast_lio fast_lio_localization; do
    run_ctest_if_present "${ROOT_DIR}/build/native_fast_lio/build/${package}" "${package}"
  done
  for package in scan_planner_msgs plan_env path_searching bspline_opt traj_utils scan_planner; do
    run_ctest_if_present "${scan_output}/build/${package}" "SCAN_${package}"
  done
  pct_library_root="${PCT_GPL_ROOT}/planner/lib"
  pct_ld_path="${pct_library_root}/3rdparty/gtsam-4.1.1/install/lib:${pct_library_root}/3rdparty/osqp/install/lib:${pct_library_root}:${pct_library_root}/build/src/common/smoothing"
  PYTHONNOUSERSITE=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONPATH="${PCT_GPL_ROOT}/planner:${pct_library_root}" \
    LD_LIBRARY_PATH="${pct_ld_path}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
    /usr/bin/python3 -m pytest -q -p no:cacheprovider \
    "${PCT_GPL_ROOT}/planner/tests/test_multilayer_astar_state.py" \
    "${PCT_GPL_ROOT}/planner/tests/test_stair_centerline.py"
  PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 \
    /usr/bin/python3 -m pytest -q -p no:cacheprovider \
    "${ROOT_DIR}/tests/test_dual_map_configuration.py" \
    "${ROOT_DIR}/tests/test_full_nx_installer.py" \
    "${ROOT_DIR}/tests/test_select_navigation_map.py" \
    "${ROOT_DIR}/tests/test_select_pcd_map.py"
  echo "FULL_NAVIGATION_TESTS=PASS"
fi

for repository in "${ROOT_DIR}" "${FAST_LIO_GPL_ROOT}" "${PCT_GPL_ROOT}"; do
  (cd "${repository}" && sha256sum --quiet -c SOURCE_INVENTORY.sha256) || {
    echo "构建过程修改了受保护源码: ${repository}" >&2
    exit 1
  }
done
echo "POST_BUILD_SOURCE_INVENTORIES=PASS"

site_config="${ROOT_DIR}/config/navigation.env"
if [[ ! -e "${site_config}" ]]; then
  umask 077
  cp "${ROOT_DIR}/config/navigation.env.example" "${site_config}"
  chmod 0600 "${site_config}"
  echo "SITE_CONFIG=CREATED_PLACEHOLDERS_EDIT_REQUIRED"
else
  chmod 0600 "${site_config}"
  echo "SITE_CONFIG=PRESERVED"
fi

echo
echo "FULL_NAVIGATION_SOURCE_INSTALL=PASS"
echo "安全状态：仅完成构建，未启动导航，未解除底盘暂停。"
echo "仍需编辑 config/navigation.env，并通过 navi.sh map/save/pct 生成实际地图。"