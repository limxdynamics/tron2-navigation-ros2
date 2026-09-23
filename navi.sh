#!/usr/bin/env bash
# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAVIGATION_ENV_FILE="${NAVIGATION_ENV_FILE:-${ROOT_DIR}/config/navigation.env}"
if [[ -r "${NAVIGATION_ENV_FILE}" ]]; then
  # Machine-specific values are intentionally kept outside public source control.
  # shellcheck disable=SC1090
  source "${NAVIGATION_ENV_FILE}"
fi
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
FAST_LIO_SETUP="${FAST_LIO_ROS2_SETUP:-${ROOT_DIR}/build/native_fast_lio/install/setup.bash}"
if [[ -f "${ROOT_DIR}/build/permissive_scan/install/setup.bash" ]]; then
  DEFAULT_SCAN_SETUP="${ROOT_DIR}/build/permissive_scan/install/setup.bash"
else
  DEFAULT_SCAN_SETUP="${ROOT_DIR}/SCAN-Planner-ros2-community/install/setup.bash"
fi
SCAN_SETUP="${SCAN_SETUP:-${DEFAULT_SCAN_SETUP}}"
PYTHON_EXECUTABLE="${PYTHON_EXECUTABLE:-/usr/bin/python3}"
MAP_TOOLS_SITE="${PCT_MAP_TOOLS_SITE:-${ROOT_DIR}/.map-tools/site}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-26}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

usage() {
  cat <<'EOF'
方便入口（NX 上运行，rviz 除外）：
  ./navi.sh map [地图名]       开始 FAST-LIO 双图建图
  ./navi.sh save               保存定位/PCT 源 PCD 并停止建图
  ./navi.sh pct [转换参数]     从近场 PCD 生成 PCT，并配对完整定位图
  ./navi.sh nav [选项]         选择地图，以配置的线速度暂停启动导航
  ./navi.sh go                 检查链路并确认后开始行驶
  ./navi.sh pause              暂停并发送重复零速度
  ./navi.sh cancel             暂停、取消目标并清空路径
  ./navi.sh stop               安全停止全部导航进程
  ./navi.sh status             查看导航和底盘链路状态
  ./navi.sh rviz               在笔记本启动导航 RViz

nav 选项：
  --speed MPS                 线速度上限，范围 (0, 0.80]，默认读取 NAV_SPEED（未配置为 0.60）
  --no-obstacle-avoidance     危险测试：关闭 SCAN 实时点云避障，仅跟随 PCT 路径
  --stair-centerline          楼梯测试：发布经 PCT 支撑验证的起终点中心直线
  --robot-accid ACCID         直接指定底盘身份；省略时启动后交互输入

PCT 可视化和跨层规划保持原有行为，不裁剪或限制地图楼层。
底盘身份格式为 WF_<型号>_<编号>。可在 config/navigation.env 中配置本机默认值；
未配置时必须在启动提示中输入。
导航启动后仍保持暂停，必须另行运行 go。
EOF
}

is_valid_robot_accid() {
  local value="$1"
  [[ ${#value} -le 64 && "${value}" =~ ^WF_[A-Za-z0-9]+_[A-Za-z0-9]+$ ]]
}

require_vendor_mros_stopped() {
  if systemctl is-active --quiet camera-mros.service 2>/dev/null ||
    pgrep -f '/opt/limx/install/bin/([r]slidar_sdk_node|[n]avigation_node)' \
      >/dev/null 2>&1; then
    echo "检测到 LimX camera-mros.service 的旧 MROS/雷达进程。" >&2
    echo "它由 root 管理且会自动重启，不能由导航清理脚本直接杀进程。" >&2
    echo "请先运行: sudo systemctl stop camera-mros.service" >&2
    echo "确认 systemctl is-active camera-mros.service 返回 inactive 后重试。" >&2
    return 1
  fi
}

source_control_environment() {
  [[ -f "${ROS_SETUP}" ]] || {
    echo "ROS 2 Humble setup not found: ${ROS_SETUP}" >&2
    return 1
  }
  [[ -f "${SCAN_SETUP}" ]] || {
    echo "SCAN setup not found: ${SCAN_SETUP}" >&2
    return 1
  }
  set +u
  source "${ROS_SETUP}"
  source "${SCAN_SETUP}"
  set -u
}

publish_pause() {
  local value="$1"
  local required_subscriptions="$2"
  timeout 8s ros2 topic pub -r 20 --times 10 \
    --wait-matching-subscriptions "${required_subscriptions}" \
    /scan_planner/pause std_msgs/msg/Bool "{data: ${value}}"
}

publish_zero_velocity() {
  timeout 4s ros2 topic pub -r 20 --times 10 \
    --wait-matching-subscriptions 1 \
    /sdk_cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
}

read_topic_count() {
  local topic="$1"
  local kind="$2"
  ros2 topic info "${topic}" 2>/dev/null |
    awk -v label="${kind}" '$1 == label && $2 == "count:" {print $3; exit}'
}

pause_navigation() {
  source_control_environment
  local pause_subscriptions cmd_subscriptions
  pause_subscriptions="$(read_topic_count /scan_planner/pause Subscription || true)"
  cmd_subscriptions="$(read_topic_count /sdk_cmd_vel Subscription || true)"
  if [[ "${pause_subscriptions:-0}" -ge 1 ]]; then
    publish_pause true "${pause_subscriptions}" || true
  fi
  if [[ "${cmd_subscriptions:-0}" -ge 1 ]]; then
    publish_zero_velocity || true
  fi
  echo "NAVIGATION_PAUSED_AND_ZEROED"
}

command_map() {
  [[ $# -le 1 ]] || { usage >&2; exit 2; }
  "${ROOT_DIR}/stop_nx_navigation_humble.sh"

  local name="${1:-fairy_$(date +%Y%m%d_%H%M%S)}"
  [[ "${name}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || {
    echo "地图名只能包含字母、数字、点、下划线和短横线。" >&2
    exit 2
  }
  local output="${ROOT_DIR}/FAST_LIO/PCD/${name}.pcd"
  local pct_output="${ROOT_DIR}/FAST_LIO/PCD/${name}.pct-source.pcd"
  local pair_manifest="${ROOT_DIR}/FAST_LIO/PCD/${name}.map-pair.json"
  [[ ! -e "${output}" && ! -e "${pct_output}" && ! -e "${pair_manifest}" ]] || {
    echo "地图或双图清单已存在，拒绝覆盖: ${name}" >&2
    exit 1
  }
  printf '%s\n' "${name}" > "${ROOT_DIR}/.current_mapping_name"
  echo "MAPPING_NAME=${name}"
  echo "LOCALIZATION_MAP_OUTPUT=${output}"
  echo "PCT_SOURCE_MAP_OUTPUT=${pct_output}"
  echo "LOCALIZATION_RANGE=${MAPPING_LOCALIZATION_MAX_DISTANCE:-100}m"
  echo "REGISTRATION_RANGE=${MAPPING_REGISTRATION_MAX_DISTANCE:-15}m"
  echo "PCT_SOURCE_RANGE=${MAPPING_PCT_MAX_DISTANCE:-15}m"
  echo "建图完成后在另一个 NX 终端运行: ./navi.sh save"
  exec env \
    MAPPING_LOCALIZATION_MAX_DISTANCE="${MAPPING_LOCALIZATION_MAX_DISTANCE:-100}" \
    MAPPING_REGISTRATION_MAX_DISTANCE="${MAPPING_REGISTRATION_MAX_DISTANCE:-15}" \
    MAPPING_PCT_MAX_DISTANCE="${MAPPING_PCT_MAX_DISTANCE:-15}" \
    MAP_OUTPUT="${output}" PCT_MAP_OUTPUT="${pct_output}" RVIZ=false \
    "${ROOT_DIR}/run_fast_lio_mapping_humble.sh"
}

command_save() {
  [[ $# -eq 0 ]] || { usage >&2; exit 2; }
  [[ -f "${FAST_LIO_SETUP}" ]] || {
    echo "FAST-LIO setup not found: ${FAST_LIO_SETUP}" >&2
    exit 1
  }
  set +u
  source "${ROS_SETUP}"
  source "${FAST_LIO_SETUP}"
  set -u

  local name_file="${ROOT_DIR}/.current_mapping_name"
  [[ -s "${name_file}" ]] || {
    echo "没有当前建图名称；请先运行 ./navi.sh map。" >&2
    exit 1
  }
  local name output pct_output pair_manifest response
  name="$(<"${name_file}")"
  output="${ROOT_DIR}/FAST_LIO/PCD/${name}.pcd"
  pct_output="${ROOT_DIR}/FAST_LIO/PCD/${name}.pct-source.pcd"
  pair_manifest="${ROOT_DIR}/FAST_LIO/PCD/${name}.map-pair.json"
  response="$(ros2 service call /map_save std_srvs/srv/Trigger '{}')"
  printf '%s\n' "${response}"
  grep -Eiq 'success[^A-Za-z]*(true|True)' <<<"${response}" || {
    echo "地图保存服务没有返回 success=true；建图保持运行，请排查后重试。" >&2
    exit 1
  }
  [[ -s "${output}" ]] || {
    echo "地图服务成功但完整定位 PCD 不存在或为空: ${output}" >&2
    exit 1
  }
  [[ -s "${pct_output}" ]] || {
    echo "地图服务成功但 PCT 近场源 PCD 不存在或为空: ${pct_output}" >&2
    exit 1
  }
  "${PYTHON_EXECUTABLE}" - \
    "${name}" "${output}" "${pct_output}" "${pair_manifest}" \
    "${MAPPING_LOCALIZATION_MAX_DISTANCE:-100}" \
    "${MAPPING_PCT_MAX_DISTANCE:-15}" \
    "${MAPPING_REGISTRATION_MAX_DISTANCE:-15}" <<'PY'
import hashlib
import json
import os
import sys
from pathlib import Path


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


name = sys.argv[1]
localization = Path(sys.argv[2]).resolve()
pct_source = Path(sys.argv[3]).resolve()
manifest = Path(sys.argv[4])
payload = {
    "schema_version": 1,
    "map_name": name,
    "coordinate_frame": "camera_init",
    "same_mapping_session": True,
    "registration_max_range_m": float(sys.argv[7]),
    "localization_pcd": {
        "name": localization.name,
        "sha256": sha256_file(localization),
        "max_range_m": float(sys.argv[5]),
    },
    "pct_source_pcd": {
        "name": pct_source.name,
        "sha256": sha256_file(pct_source),
        "max_range_m": float(sys.argv[6]),
    },
}
temporary = manifest.with_name(manifest.name + ".new.%d" % os.getpid())
try:
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    temporary.chmod(0o664)
    os.replace(str(temporary), str(manifest))
finally:
    if temporary.exists():
        temporary.unlink()
PY
  ls -lh "${output}" "${pct_output}" "${pair_manifest}"
  sha256sum "${output}" "${pct_output}" "${pair_manifest}"
  "${ROOT_DIR}/stop_nx_navigation_humble.sh"
  echo "DUAL_MAP_SAVE=PASS"
  echo "下一步: ./navi.sh pct"
}

command_pct() {
  "${ROOT_DIR}/stop_nx_navigation_humble.sh"
  local name_file="${ROOT_DIR}/.current_mapping_name"
  if [[ $# -eq 0 && -s "${name_file}" ]]; then
    local name localization pct_source pair_manifest
    name="$(<"${name_file}")"
    localization="${ROOT_DIR}/FAST_LIO/PCD/${name}.pcd"
    pct_source="${ROOT_DIR}/FAST_LIO/PCD/${name}.pct-source.pcd"
    pair_manifest="${ROOT_DIR}/FAST_LIO/PCD/${name}.map-pair.json"
    if [[ -s "${localization}" && -s "${pct_source}" && -s "${pair_manifest}" ]]; then
      echo "使用当前双图建图产物生成 PCT: ${name}"
      exec "${ROOT_DIR}/prepare_fast_lio_pcd_map.sh" \
        "${pct_source}" \
        --localization-pcd "${localization}" \
        --pair-manifest "${pair_manifest}" \
        --name "${name}" \
        --resolution "${MAPPING_PCT_RESOLUTION:-0.1}"
    fi
  fi
  exec "${ROOT_DIR}/prepare_fast_lio_pcd_map.sh" "$@"
}

command_nav() {
  local speed="${NAV_SPEED:-0.60}"
  local speed_supplied=false
  local local_obstacle_avoidance=true
  local stair_centerline=false
  local default_websocket_robot_accid="${WEBSOCKET_ROBOT_ACCID:-}"
  local websocket_robot_accid=""
  local robot_accid_supplied=false
  local answer=""
  local prompt=""
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --speed)
        [[ $# -ge 2 ]] || { usage >&2; exit 2; }
        speed="$2"
        speed_supplied=true
        shift 2
        ;;
      --no-obstacle-avoidance)
        local_obstacle_avoidance=false
        shift
        ;;
      --stair-centerline|--stair-centerline-mode)
        stair_centerline=true
        shift
        ;;
      --robot-accid|--accid)
        [[ $# -ge 2 ]] || { usage >&2; exit 2; }
        websocket_robot_accid="$2"
        robot_accid_supplied=true
        shift 2
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        echo "未知 nav 参数: $1" >&2
        usage >&2
        exit 2
        ;;
    esac
  done
  [[ "${speed}" =~ ^[0-9]+([.][0-9]+)?$ ]] &&
    awk -v value="${speed}" 'BEGIN { exit !(value > 0.0 && value <= 0.80) }' || {
      echo "--speed 必须大于 0 且不超过 0.80 m/s。" >&2
      exit 2
    }

  if [[ "${local_obstacle_avoidance}" == false ]]; then
    [[ "${speed_supplied}" == true ]] || {
      echo "无避障模式必须显式指定 --speed，且不得超过 0.60 m/s。" >&2
      exit 2
    }
    awk -v value="${speed}" 'BEGIN { exit !(value > 0.0 && value <= 0.60) }' || {
      echo "无避障模式的 --speed 必须大于 0 且不超过 0.60 m/s。" >&2
      exit 2
    }
  fi
  if [[ "${stair_centerline}" == true && \
        "${local_obstacle_avoidance}" != false ]]; then
    echo "--stair-centerline 必须与 --no-obstacle-avoidance 同时使用。" >&2
    exit 2
  fi

  if [[ "${robot_accid_supplied}" != true ]]; then
    if [[ -t 0 && -t 1 ]]; then
      if [[ -n "${default_websocket_robot_accid}" ]]; then
        prompt="请输入底盘身份 [${default_websocket_robot_accid}]: "
      else
        prompt="请输入底盘身份（WF_<型号>_<编号>）: "
      fi
      read -r -p "${prompt}" answer || {
        echo "未读取到底盘身份，导航未启动。" >&2
        exit 130
      }
      websocket_robot_accid="${answer:-${default_websocket_robot_accid}}"
    else
      websocket_robot_accid="${default_websocket_robot_accid}"
      [[ -n "${websocket_robot_accid}" ]] &&
        echo "非交互终端，使用已配置的底盘身份: ${websocket_robot_accid}"
    fi
  fi
  is_valid_robot_accid "${websocket_robot_accid}" || {
    echo "底盘身份格式错误：${websocket_robot_accid}" >&2
    echo "必须使用 WF_<型号>_<编号>，仅允许英文字母和数字，总长度不超过 64。" >&2
    exit 2
  }

  "${ROOT_DIR}/stop_nx_navigation_humble.sh"
  PYTHONPATH="${MAP_TOOLS_SITE}${PYTHONPATH:+:${PYTHONPATH}}" \
    "${PYTHON_EXECUTABLE}" "${ROOT_DIR}/select_navigation_map.py" \
      --root "${ROOT_DIR}" --mode prompt

  echo "NAVIGATION_SPEED=${speed}m/s"
  echo "LOCAL_OBSTACLE_AVOIDANCE=${local_obstacle_avoidance}"
  echo "PCT_STAIR_CENTERLINE=${stair_centerline}"
  if [[ "${local_obstacle_avoidance}" == false ]]; then
    echo "警告：SCAN 不会订阅实时障碍点云，只按 PCT 全局路径生成局部轨迹。"
  fi
  if [[ "${stair_centerline}" == true ]]; then
    echo "楼梯中心直线模式：仅验证通过的严格 XY 直线会发布到 /pct_path。"
  fi
  echo "CHASSIS_OUTPUT_MODE=websocket"
  echo "WEBSOCKET_PROTOCOL=legacy"
  echo "SELECTED_CHASSIS_ACCID=${websocket_robot_accid}"
  echo "注意：request_twist 成功路径无 ACK；断线、显式拒绝和看门狗仍会闭锁暂停。"
  echo "PCT 楼层显示与跨层规划保持原有行为。"
  echo "导航保持暂停；设置初始位置和目标后运行: ./navi.sh go"

  exec env \
    NAVIGATION_MAP_SELECTION=current \
    START_PAUSED=true \
    CHASSIS_OUTPUT_MODE=websocket \
    WEBSOCKET_PROTOCOL=legacy \
    WEBSOCKET_ROBOT_ACCID="${websocket_robot_accid}" \
    BASE_YAW_OFFSET_DEG=60.0 \
    PCT_TOMOGRAM_STRIDE=1 \
    ENABLE_LOCAL_OBSTACLE_AVOIDANCE="${local_obstacle_avoidance}" \
    ENABLE_PCT_STAIR_CENTERLINE="${stair_centerline}" \
    CONTROLLER_MAX_LINEAR_SPEED="${speed}" \
    CONTROLLER_MAX_ANGULAR_SPEED=0.20 \
    WEBSOCKET_MAX_LINEAR_SPEED="${speed}" \
    WEBSOCKET_MAX_LATERAL_SPEED="${speed}" \
    WEBSOCKET_MAX_ANGULAR_SPEED=0.20 \
    "${ROOT_DIR}/run_fast_lio_navigation_humble.sh"
}

command_go() {
  [[ $# -eq 0 ]] || { usage >&2; exit 2; }
  source_control_environment
  local nodes pause_subscriptions cmd_publishers cmd_subscriptions answer
  local obstacle_mode expected_confirmation
  nodes="$(ros2 node list --no-daemon --spin-time 2 2>/dev/null)"
  for required in /pct_planner /scan_planner_node /closed_loop_controller; do
    grep -qx "${required}" <<<"${nodes}" || {
      echo "不能开始：缺少节点 ${required}。" >&2
      exit 1
    }
  done
  grep -qx '/limx_websocket_cmd_bridge' <<<"${nodes}" || {
    echo "不能开始：缺少 legacy WebSocket 底盘桥节点。" >&2
    exit 1
  }
  grep -qx '/mros_ros2_bridge' <<<"${nodes}" && {
    echo "不能开始：检测到已停用的 MROS 底盘桥。" >&2
    exit 1
  }
  pause_subscriptions="$(read_topic_count /scan_planner/pause Subscription || true)"
  cmd_publishers="$(read_topic_count /sdk_cmd_vel Publisher || true)"
  cmd_subscriptions="$(read_topic_count /sdk_cmd_vel Subscription || true)"
  [[ "${pause_subscriptions:-0}" -eq 2 && "${cmd_publishers:-0}" -eq 1 && \
     "${cmd_subscriptions:-0}" -eq 1 ]] || {
    echo "不能开始：底盘链路不完整。" >&2
    echo "pause subscriptions=${pause_subscriptions:-0}, cmd publishers=${cmd_publishers:-0}, cmd subscriptions=${cmd_subscriptions:-0}" >&2
    exit 1
  }
  obstacle_mode="$(timeout 5s ros2 param get \
    /scan_planner_node grid_map.enable_cloud_subscription 2>/dev/null)" || {
      echo "不能开始：无法确认 SCAN 实时点云避障状态。" >&2
      exit 1
    }
  [[ -t 0 && -t 1 ]] || {
    echo "./navi.sh go 必须在现场交互终端运行。" >&2
    exit 1
  }
  if grep -Eiq 'false[[:space:]]*$' <<<"${obstacle_mode}"; then
    expected_confirmation="NO_OBSTACLE_GO"
    echo "危险：当前未订阅实时障碍点云，机器人不会检测人员、杂物或临时障碍。" >&2
    read -r -p "确认楼梯和路线完全清空、有人监护、急停在手；输入 ${expected_confirmation} 开始: " answer
  elif grep -Eiq 'true[[:space:]]*$' <<<"${obstacle_mode}"; then
    expected_confirmation="GO"
    read -r -p "确认定位重合、轨迹成功、场地清空、急停在手；输入 GO 开始行驶: " answer
  else
    echo "不能开始：SCAN 返回了无法识别的避障状态：${obstacle_mode}" >&2
    exit 1
  fi
  [[ "${answer}" == "${expected_confirmation}" ]] || {
    echo "已取消，机器人保持暂停。"
    exit 130
  }
  publish_pause false 2
  echo "NAVIGATION_RUNNING"
}

command_pause() {
  [[ $# -eq 0 ]] || { usage >&2; exit 2; }
  pause_navigation
}

command_cancel() {
  [[ $# -eq 0 ]] || { usage >&2; exit 2; }
  pause_navigation
  timeout 8s ros2 topic pub -r 20 --times 10 \
    --wait-matching-subscriptions 1 \
    /navigation/cancel std_msgs/msg/Empty '{}'
  echo "NAVIGATION_GOAL_CANCELLED"
}

command_stop() {
  [[ $# -eq 0 ]] || { usage >&2; exit 2; }
  exec "${ROOT_DIR}/stop_nx_navigation_humble.sh"
}

command_status() {
  [[ $# -eq 0 ]] || { usage >&2; exit 2; }
  source_control_environment
  local nodes
  nodes="$(ros2 node list --no-daemon --spin-time 2 2>/dev/null || true)"
  echo "=== 导航节点 ==="
  grep -E '^/(global_localization|transform_fusion|pct_planner|scan_planner_node|closed_loop_controller|limx_websocket_cmd_bridge)$' <<<"${nodes}" || true
  echo "=== 话题端点 ==="
  ros2 topic info /scan_planner/pause 2>/dev/null || true
  ros2 topic info /sdk_cmd_vel 2>/dev/null || true
  echo "=== SCAN 实时点云避障 ==="
  ros2 param get /scan_planner_node grid_map.enable_cloud_subscription 2>/dev/null || true
  echo "=== PCT 楼梯中心直线 ==="
  ros2 param get /pct_planner stair_centerline.enabled 2>/dev/null || true
}

command_rviz() {
  [[ $# -eq 0 ]] || { usage >&2; exit 2; }
  exec "${ROOT_DIR}/run_local_rviz_foxy.sh"
}

command="${1:-help}"
shift || true
case "${command}" in
  map|mapping) require_vendor_mros_stopped; command_map "$@" ;;
  save) command_save "$@" ;;
  pct) command_pct "$@" ;;
  nav) require_vendor_mros_stopped; command_nav "$@" ;;
  go) command_go "$@" ;;
  pause) command_pause "$@" ;;
  cancel) command_cancel "$@" ;;
  stop) command_stop "$@" ;;
  status) command_status "$@" ;;
  rviz) command_rviz "$@" ;;
  help|-h|--help) usage ;;
  *)
    echo "未知命令: ${command}" >&2
    usage >&2
    exit 2
    ;;
esac
