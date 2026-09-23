#!/usr/bin/env bash
# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOMOGRAM_DIR="${ROOT_DIR}/PCT_planner-RC2026_Map_Planner/rsc/tomogram"
PCD_DIR="${ROOT_DIR}/PCT_planner-RC2026_Map_Planner/rsc/pcd"

usage() {
  cat <<'EOF'
Usage:
  ./switch_pct_map.sh --tomogram PATH [--pcd PATH]

Atomically switches the local PCT defaults:
  rsc/tomogram/map.pickle -> the named tomogram
  rsc/pcd/map.pcd         -> the named dense PCD (when --pcd is supplied)

The script refuses to switch while the PCT/navigation stack is running.
A legacy regular map.pickle or map.pcd is renamed to a timestamped backup.
EOF
}

TOMOGRAM=""
PCD=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --tomogram)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      TOMOGRAM="$2"
      shift 2
      ;;
    --pcd)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      PCD="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ -n "${TOMOGRAM}" ]] || { echo "--tomogram is required" >&2; exit 2; }

if pgrep -f 'run_nx_navigation_humble.sh|navigation_plan.py|publish_tomogram_from_pickle.py' >/dev/null 2>&1; then
  echo "Refusing to switch maps while PCT/navigation is running." >&2
  echo "Stop the navigation stack and local PCT RViz first." >&2
  exit 1
fi

resolve_input() {
  local input="$1"
  local default_dir="$2"
  if [[ -e "${input}" ]]; then
    realpath -e "${input}"
  elif [[ -e "${default_dir}/${input}" ]]; then
    realpath -e "${default_dir}/${input}"
  else
    echo "Map file does not exist: ${input}" >&2
    return 1
  fi
}

activate_link() {
  local target="$1"
  local link_path="$2"
  local expected_suffix="$3"
  local target_real link_dir relative_target temporary backup timestamp

  target_real="$(realpath -e "${target}")"
  [[ "${target_real}" == *"${expected_suffix}" ]] || {
    echo "Expected a ${expected_suffix} file: ${target_real}" >&2
    return 1
  }
  link_dir="$(dirname "${link_path}")"
  mkdir -p "${link_dir}"

  if [[ -e "${link_path}" && ! -L "${link_path}" ]]; then
    if [[ "$(realpath -e "${link_path}")" == "${target_real}" ]]; then
      echo "Refusing to replace a regular default file with a link to itself: ${link_path}" >&2
      return 1
    fi
    timestamp="$(date +%Y%m%d-%H%M%S)"
    backup="${link_path%.*}.legacy-${timestamp}.${link_path##*.}"
    mv "${link_path}" "${backup}"
    echo "Preserved legacy default: ${backup}"
  fi

  relative_target="$(realpath --relative-to="${link_dir}" "${target_real}")"
  temporary="${link_dir}/.$(basename "${link_path}").new.$$"
  rm -f "${temporary}"
  ln -s "${relative_target}" "${temporary}"
  mv -Tf "${temporary}" "${link_path}"
  echo "Activated: ${link_path} -> $(readlink "${link_path}")"
}

TOMOGRAM_REAL="$(resolve_input "${TOMOGRAM}" "${TOMOGRAM_DIR}")"
activate_link "${TOMOGRAM_REAL}" "${TOMOGRAM_DIR}/map.pickle" ".pickle"
sha256sum "${TOMOGRAM_DIR}/map.pickle"

if [[ -n "${PCD}" ]]; then
  PCD_REAL="$(resolve_input "${PCD}" "${PCD_DIR}")"
  activate_link "${PCD_REAL}" "${PCD_DIR}/map.pcd" ".pcd"
  sha256sum "${PCD_DIR}/map.pcd"
fi
