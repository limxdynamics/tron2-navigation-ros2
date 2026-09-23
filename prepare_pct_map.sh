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
PCT_ROOT="${ROOT_DIR}/PCT_planner-RC2026_Map_Planner"
EXTRACTOR="${PCT_ROOT}/tomography/scripts/extract_mros_keyframe_map.py"
BUILDER="${PCT_ROOT}/tomography/scripts/build_tomogram_offline.py"
PYTHON_EXECUTABLE="${PYTHON_EXECUTABLE:-/usr/bin/python3}"
MAP_TOOLS_SITE="${PCT_MAP_TOOLS_SITE:-${ROOT_DIR}/.map-tools/site}"
NX_MAP_STORAGE_ROOT="${NX_MAP_STORAGE_ROOT:-/path/to/nx/map-storage}"
NX_NAVIGATION_ROOT="${NX_NAVIGATION_ROOT:-/path/to/navigation}"

if [[ -d "${MAP_TOOLS_SITE}" ]]; then
  export PYTHONPATH="${MAP_TOOLS_SITE}${PYTHONPATH:+:${PYTHONPATH}}"
fi
if [[ -d /usr/local/cuda ]]; then
  export CUDA_PATH="${CUDA_PATH:-/usr/local/cuda}"
  export PATH="${CUDA_PATH}/bin:${PATH}"
  export LD_LIBRARY_PATH="${CUDA_PATH}/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

usage() {
  cat <<'EOF'
Usage:
  ./prepare_pct_map.sh /path/to/copied/map-directory [OPTIONS]

  ./prepare_pct_map.sh --map-dir /path/to/copied/map-directory [OPTIONS]

Advanced input mode:
  ./prepare_pct_map.sh \
    --name MAP_NAME \
    --bag /path/to/result.bag \
    [--reference-pcd /path/to/result.pcd] \
    [--base-transform rs-fairy|livox-mid360|identity] \
    [--profile standard|nx-large] [--large-map] \
    [--xy-dilation-cells 1] [--ground-h METERS] \
    [--activate|--no-activate] [--force]

Directory mode automatically:
  * uses the directory name as MAP_NAME (override with --name)
  * reads result.bag and, when present, result.pcd from that directory
  * uses rs-fairy and one XY dilation cell unless overridden
  * validates the generated PCT map and activates it locally
  * writes an NX handoff manifest containing exact files and destinations

Outputs:
  rsc/bag/nx_localization_MAP_NAME_result.bag
  rsc/pcd/nx_localization_MAP_NAME.pcd             (when reference PCD is given)
  rsc/pcd/nx_localization_MAP_NAME_dense.pcd
  rsc/pcd/nx_localization_MAP_NAME_dense.pcd.json
  rsc/tomogram/nx_localization_MAP_NAME_scene_map_dense_d1.pickle
  rsc/handoff/nx_localization_MAP_NAME_NX_HANDOFF.txt

--no-activate keeps local map.pcd/map.pickle unchanged in directory mode.
--activate switches them in advanced input mode.
--force permits rebuilding an existing named output; it never deletes old maps.
--large-map is an alias for --profile nx-large. It keeps the complete robot
trajectory while filtering each keyframe to 100 m XY range and the trajectory
world-Z envelope, then builds at 0.2 m resolution for a 16 GiB Jetson Orin NX.
EOF
}

MAP_DIR=""
NAME=""
SOURCE_BAG=""
REFERENCE_PCD=""
BASE_TRANSFORM="rs-fairy"
XY_DILATION_CELLS=1
GROUND_H=""
PROFILE="standard"
PROFILE_OPTION_COUNT=0
ACTIVATE="auto"
FORCE=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map-dir)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      MAP_DIR="$2"
      shift 2
      ;;
    --name)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      NAME="$2"
      shift 2
      ;;
    --bag)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      SOURCE_BAG="$2"
      shift 2
      ;;
    --reference-pcd)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      REFERENCE_PCD="$2"
      shift 2
      ;;
    --base-transform)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      BASE_TRANSFORM="$2"
      shift 2
      ;;
    --profile)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      PROFILE="$2"
      PROFILE_OPTION_COUNT=$((PROFILE_OPTION_COUNT + 1))
      shift 2
      ;;
    --large-map)
      PROFILE="nx-large"
      PROFILE_OPTION_COUNT=$((PROFILE_OPTION_COUNT + 1))
      shift
      ;;
    --xy-dilation-cells)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      XY_DILATION_CELLS="$2"
      shift 2
      ;;
    --ground-h)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      GROUND_H="$2"
      shift 2
      ;;
    --activate)
      ACTIVATE=true
      shift
      ;;
    --no-activate)
      ACTIVATE=false
      shift
      ;;
    --force)
      FORCE=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      if [[ "$1" != -* && -z "${MAP_DIR}" ]]; then
        MAP_DIR="$1"
        shift
      else
        echo "Unknown or duplicate argument: $1" >&2
        usage >&2
        exit 2
      fi
      ;;
  esac
done

if [[ -n "${MAP_DIR}" ]]; then
  [[ -d "${MAP_DIR}" ]] || { echo "Map directory not found: ${MAP_DIR}" >&2; exit 1; }
  [[ -z "${SOURCE_BAG}" ]] || {
    echo "Do not combine a map directory with --bag; result.bag is detected automatically." >&2
    exit 2
  }
  MAP_DIR="$(realpath -e "${MAP_DIR}")"
  SOURCE_BAG="${MAP_DIR}/result.bag"
  if [[ -z "${REFERENCE_PCD}" && -f "${MAP_DIR}/result.pcd" ]]; then
    REFERENCE_PCD="${MAP_DIR}/result.pcd"
  fi
  if [[ -z "${NAME}" ]]; then
    NAME="$(basename "${MAP_DIR}")"
  fi
  if [[ "${ACTIVATE}" == auto ]]; then
    ACTIVATE=true
  fi
else
  [[ -n "${NAME}" && -n "${SOURCE_BAG}" ]] || {
    echo "Pass a copied map directory, or provide both --name and --bag." >&2
    usage >&2
    exit 2
  }
  if [[ "${ACTIVATE}" == auto ]]; then
    ACTIVATE=false
  fi
fi

[[ "${NAME}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || {
  echo "MAP_NAME may contain only letters, numbers, dot, underscore and dash." >&2
  echo "Rename the directory or provide --name SAFE_NAME." >&2
  exit 2
}
case "${BASE_TRANSFORM}" in
  rs-fairy|livox-mid360|identity) ;;
  *) echo "Unsupported --base-transform: ${BASE_TRANSFORM}" >&2; exit 2 ;;
esac
case "${PROFILE}" in
  standard|nx-large) ;;
  *) echo "Unsupported --profile: ${PROFILE}" >&2; exit 2 ;;
esac
[[ "${PROFILE_OPTION_COUNT}" -le 1 ]] || {
  echo "Specify only one of --profile or --large-map." >&2
  exit 2
}
[[ "${XY_DILATION_CELLS}" =~ ^[0-4]$ ]] || {
  echo "--xy-dilation-cells must be an integer in [0, 4]" >&2
  exit 2
}
if [[ "${PROFILE}" == nx-large ]]; then
  [[ "${XY_DILATION_CELLS}" == 1 ]] || {
    echo "The validated nx-large profile requires --xy-dilation-cells 1." >&2
    exit 2
  }
  [[ -z "${GROUND_H}" ]] || {
    echo "Do not combine --ground-h with the validated nx-large profile." >&2
    exit 2
  }
fi
[[ -x "${PYTHON_EXECUTABLE}" ]] || {
  echo "Python executable not found: ${PYTHON_EXECUTABLE}" >&2
  exit 1
}
[[ -f "${EXTRACTOR}" && -f "${BUILDER}" ]] || {
  echo "PCT conversion scripts are missing" >&2
  exit 1
}
[[ -f "${SOURCE_BAG}" ]] || { echo "Bag not found: ${SOURCE_BAG}" >&2; exit 1; }
if [[ -n "${REFERENCE_PCD}" && ! -f "${REFERENCE_PCD}" ]]; then
  echo "Reference PCD not found: ${REFERENCE_PCD}" >&2
  exit 1
fi
if [[ -n "${MAP_DIR}" && -z "${REFERENCE_PCD}" ]]; then
  echo "Warning: ${MAP_DIR}/result.pcd is absent; coordinate consistency comparison will be skipped." >&2
fi

if [[ "${ACTIVATE}" == true ]] && \
   pgrep -f 'run_nx_navigation_humble.sh|navigation_plan.py|publish_tomogram_from_pickle.py' >/dev/null 2>&1; then
  echo "Refusing automatic activation while local PCT/navigation is running." >&2
  echo "Stop run_local_rviz_foxy.sh/navigation first, or pass --no-activate." >&2
  exit 1
fi

SOURCE_BAG="$(realpath -e "${SOURCE_BAG}")"
if [[ -n "${REFERENCE_PCD}" ]]; then
  REFERENCE_PCD="$(realpath -e "${REFERENCE_PCD}")"
fi

PREFIX="nx_localization_${NAME}"
BAG_DIR="${PCT_ROOT}/rsc/bag"
PCD_DIR="${PCT_ROOT}/rsc/pcd"
TOMOGRAM_DIR="${PCT_ROOT}/rsc/tomogram"
HANDOFF_DIR="${PCT_ROOT}/rsc/handoff"
BAG_OUT="${BAG_DIR}/${PREFIX}_result.bag"
RAW_PCD_OUT="${PCD_DIR}/${PREFIX}.pcd"
DENSE_PCD_OUT="${PCD_DIR}/${PREFIX}_dense.pcd"
TOMOGRAM_OUT="${TOMOGRAM_DIR}/${PREFIX}_scene_map_dense_d1.pickle"
HANDOFF_OUT="${HANDOFF_DIR}/${PREFIX}_NX_HANDOFF.txt"
mkdir -p "${BAG_DIR}" "${PCD_DIR}" "${TOMOGRAM_DIR}" "${HANDOFF_DIR}"

cat <<EOF
===== Detected map input =====
MAP_NAME=${NAME}
MAP_DIRECTORY=${MAP_DIR:-$(dirname "${SOURCE_BAG}")}
RESULT_BAG=${SOURCE_BAG}
RESULT_PCD=${REFERENCE_PCD:-not found; consistency comparison disabled}
BASE_TRANSFORM=${BASE_TRANSFORM}
PROCESSING_PROFILE=${PROFILE}
XY_DILATION_CELLS=${XY_DILATION_CELLS}
LOCAL_ACTIVATION=${ACTIVATE}
EOF

if [[ "${FORCE}" != true ]]; then
  for output in "${DENSE_PCD_OUT}" "${DENSE_PCD_OUT}.json" "${TOMOGRAM_OUT}" "${HANDOFF_OUT}"; do
    if [[ -e "${output}" ]]; then
      echo "Output already exists: ${output}" >&2
      echo "Use a new --name, or pass --force to rebuild this name." >&2
      exit 1
    fi
  done
fi

# Build into a private directory first. This is essential when map.pcd or
# map.pickle is a symlink to this map's named artifact: a failed rebuild must
# not mutate the currently active target before validation and backup.
BUILD_DIR="${PCT_ROOT}/rsc/.build/${PREFIX}.$$"
BUILD_DENSE_PCD="${BUILD_DIR}/$(basename "${DENSE_PCD_OUT}")"
BUILD_TOMOGRAM="${BUILD_DIR}/$(basename "${TOMOGRAM_OUT}")"
mkdir -p "${BUILD_DIR}"
cleanup_build_dir() {
  rm -rf "${BUILD_DIR}"
}
trap cleanup_build_dir EXIT
trap 'exit 130' INT TERM

copy_atomic() {
  local source="$1"
  local destination="$2"
  local temporary
  if [[ -e "${destination}" ]] && cmp -s "${source}" "${destination}"; then
    echo "Reusing identical input: ${destination}"
    return
  fi
  temporary="${destination}.tmp.$$"
  rm -f "${temporary}"
  cp --reflink=auto "${source}" "${temporary}"
  chmod 664 "${temporary}"
  mv -f "${temporary}" "${destination}"
}

copy_atomic "${SOURCE_BAG}" "${BAG_OUT}"
if [[ -n "${REFERENCE_PCD}" ]]; then
  copy_atomic "${REFERENCE_PCD}" "${RAW_PCD_OUT}"
fi

configure_python_cuda_runtime() {
  local user_site nvidia_root directory joined
  local -a cuda_library_dirs=()
  user_site="$("${PYTHON_EXECUTABLE}" -c 'import site; print(site.getusersitepackages())')"
  nvidia_root="${user_site}/nvidia"
  if [[ -d "${nvidia_root}" ]]; then
    while IFS= read -r -d '' directory; do
      cuda_library_dirs+=("${directory}")
    done < <(find "${nvidia_root}" -mindepth 2 -maxdepth 2 \
      -type d -name lib -print0 | sort -z)
  fi
  if [[ ${#cuda_library_dirs[@]} -gt 0 ]]; then
    printf -v joined '%s:' "${cuda_library_dirs[@]}"
    export LD_LIBRARY_PATH="${joined%:}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    echo "CUDA Python libraries: ${joined%:}"
  fi
  if ! "${PYTHON_EXECUTABLE}" -c \
    'import cupy as cp; assert cp.cuda.runtime.getDeviceCount() > 0' >/dev/null; then
    echo "CuPy cannot access a CUDA GPU; PCT tomogram generation cannot continue." >&2
    return 1
  fi
}

printf '\n===== Extract dense keyframe PCD =====\n'
extract_args=(
  --bag "${BAG_OUT}"
  --output "${BUILD_DENSE_PCD}"
  --base-transform "${BASE_TRANSFORM}"
)
if [[ "${PROFILE}" == nx-large ]]; then
  extract_args+=(
    --max-relative-xy-range 100
    --world-z-below-trajectory 2
    --world-z-above-trajectory 4
  )
fi
"${PYTHON_EXECUTABLE}" "${EXTRACTOR}" "${extract_args[@]}"

printf '\n===== Build GPU PCT tomogram =====\n'
configure_python_cuda_runtime
build_args=(
  --pcd "${BUILD_DENSE_PCD}"
  --output "${BUILD_TOMOGRAM}"
  --xy-dilation-cells "${XY_DILATION_CELLS}"
)
if [[ "${PROFILE}" == nx-large ]]; then
  build_args+=(
    --profile-name nx-large
    --resolution 0.2
    --kernel-size 5
    --slope-max 1.2386243738717622
  )
fi
if [[ -n "${GROUND_H}" ]]; then
  build_args+=(--ground-h "${GROUND_H}")
fi
"${PYTHON_EXECUTABLE}" "${BUILDER}" "${build_args[@]}"

printf '\n===== Validate PCT artifact =====\n'
"${PYTHON_EXECUTABLE}" - "${BUILD_TOMOGRAM}" "${PROFILE}" <<'PY'
import pickle
import sys
from pathlib import Path

import numpy as np

path = Path(sys.argv[1])
profile = sys.argv[2]
with path.open('rb') as stream:
    payload = pickle.load(stream)
data = np.asarray(payload['data'])
if data.ndim != 4 or data.shape[0] < 5 or data.shape[1] < 1:
    raise RuntimeError('invalid tomogram shape: {}'.format(data.shape))
for key in ('resolution', 'center', 'slice_h0', 'slice_dh', 'source_pcd_sha256'):
    if key not in payload:
        raise RuntimeError('tomogram metadata is missing {}'.format(key))
if not np.isfinite(data[0]).all():
    raise RuntimeError('traversability contains non-finite values')
expected_profile = 'nx-large' if profile == 'nx-large' else 'scene_map.py'
actual_profile = payload.get('build_parameters', {}).get('profile')
if actual_profile != expected_profile:
    raise RuntimeError(
        'unexpected processing profile: {} != {}'.format(
            actual_profile, expected_profile
        )
    )
print('TOMOGRAM={}'.format(path.resolve()))
print('SHAPE={}'.format(tuple(data.shape)))
print('RESOLUTION={}'.format(float(payload['resolution'])))
print('CENTER={}'.format(np.asarray(payload['center']).tolist()))
print('GROUND_H={}'.format(payload.get('ground_h')))
print('SLICE_H0={}'.format(float(payload['slice_h0'])))
print('SLICE_DH={}'.format(float(payload['slice_dh'])))
print('PROCESSING_PROFILE={}'.format(actual_profile))
PY
sha256sum "${BAG_OUT}" "${BUILD_DENSE_PCD}" "${BUILD_TOMOGRAM}"

if [[ -n "${REFERENCE_PCD}" ]]; then
  printf '\n===== Compare dense PCD with MROS result.pcd =====\n'
  "${PYTHON_EXECUTABLE}" - \
    "${RAW_PCD_OUT}" "${BUILD_DENSE_PCD}" \
    "${PCT_ROOT}/tomography/scripts" <<'PY'
import sys

import numpy as np

reference_path, dense_path, scripts_path = sys.argv[1:4]
sys.path.insert(0, scripts_path)

from pcd_io import load_xyz, nearest_neighbor_distances

reference = load_xyz(reference_path)
dense = load_xyz(dense_path)
step = max(1, len(reference) // 20000)
distances = nearest_neighbor_distances(reference[::step], dense)
median = float(np.median(distances))
p95 = float(np.percentile(distances, 95))
maximum = float(np.max(distances))
print('REFERENCE_TO_DENSE_METERS median={:.4f} p95={:.4f} max={:.4f}'.format(
    median, p95, maximum
))
if median > 0.15:
    raise RuntimeError(
        'PCD frames appear inconsistent (median error {:.3f}m > 0.15m); '
        'check --base-transform'.format(median)
    )
PY
fi

printf '\n===== Promote validated artifacts =====\n'
chmod 664 "${BUILD_DENSE_PCD}" "${BUILD_DENSE_PCD}.json" "${BUILD_TOMOGRAM}"
mv -f "${BUILD_DENSE_PCD}" "${DENSE_PCD_OUT}"
mv -f "${BUILD_DENSE_PCD}.json" "${DENSE_PCD_OUT}.json"
# Promote the planner artifact last, after every validation and companion file.
mv -f "${BUILD_TOMOGRAM}" "${TOMOGRAM_OUT}"
trap - EXIT INT TERM
cleanup_build_dir

if [[ "${ACTIVATE}" == true ]]; then
  printf '\n===== Activate local defaults =====\n'
  "${ROOT_DIR}/switch_pct_map.sh" \
    --tomogram "${TOMOGRAM_OUT}" \
    --pcd "${DENSE_PCD_OUT}"
fi

printf '\n===== Write manual NX handoff manifest =====\n'
handoff_temporary="${HANDOFF_OUT}.tmp.$$"
rm -f "${handoff_temporary}"
cat >"${handoff_temporary}" <<EOF
MAP_NAME=${NAME}
PROCESSING_PROFILE=${PROFILE}

LOCAL_SOURCE_MAP_DIRECTORY=${MAP_DIR:-$(dirname "${SOURCE_BAG}")}
LOCAL_DENSE_PCD=${DENSE_PCD_OUT}
LOCAL_PCT_PICKLE=${TOMOGRAM_OUT}

NX_ORIGINAL_MAP_DIRECTORY=${NX_MAP_STORAGE_ROOT}/${NAME}
NX_LOCALIZATION_ARCHIVE_BAG=${NX_MAP_STORAGE_ROOT}/${NAME}/result.bag
NX_LOCALIZATION_SAVED_MAP=${NX_MAP_STORAGE_ROOT}/result.bag
NX_PCT_PICKLE_DIRECTORY=${NX_NAVIGATION_ROOT}/PCT_planner-RC2026_Map_Planner/rsc/tomogram
NX_DENSE_PCD_DIRECTORY=${NX_NAVIGATION_ROOT}/PCT_planner-RC2026_Map_Planner/rsc/pcd

REQUIRED_COPY=${TOMOGRAM_OUT}
OPTIONAL_COPY=${DENSE_PCD_OUT}

IMPORTANT: The generated *_dense.pcd must NOT replace the original result.pcd.
IMPORTANT: Keep result.bag, result.pcd, limx.yaml and its PGM together in the original NX map directory.
IMPORTANT: Activate the archive with navigation_node or nx_use_map.sh before localization; FAST-LIO reads the fixed root result.bag.

SHA256:
$(sha256sum "${BAG_OUT}" "${DENSE_PCD_OUT}" "${TOMOGRAM_OUT}")
EOF
chmod 664 "${handoff_temporary}"
mv -f "${handoff_temporary}" "${HANDOFF_OUT}"
cat "${HANDOFF_OUT}"

cat <<EOF

MAP_PREPARED=${NAME}
PROCESSING_PROFILE=${PROFILE}
BAG=${BAG_OUT}
DENSE_PCD=${DENSE_PCD_OUT}
TOMOGRAM=${TOMOGRAM_OUT}
NX_HANDOFF=${HANDOFF_OUT}
LOCAL_ACTIVE=${ACTIVATE}
EOF
