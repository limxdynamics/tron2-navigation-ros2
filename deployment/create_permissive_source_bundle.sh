#!/usr/bin/env bash
# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT_DIR="${ROOT_DIR}/public_source_packages"
VERSION="v1.1.0"

usage() {
  cat <<'EOF'
Usage: ./deployment/create_permissive_source_bundle.sh [OPTIONS]

Options:
  --output DIR    Output directory (default: public_source_packages)
  --version NAME  Archive version label
  -h, --help      Show this help

Creates a deterministic permissive-only source archive. FAST_LIO,
FAST_LIO_LOCALIZATION2, PCT_planner, SCAN simulator/demo packages, maps,
binaries, build output, site data, and private deployment records are excluded.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output)
      [[ $# -ge 2 ]] || { echo "--output requires a directory" >&2; exit 2; }
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --version)
      [[ $# -ge 2 ]] || { echo "--version requires a value" >&2; exit 2; }
      VERSION="$2"
      shift 2
      ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "${VERSION}" =~ ^[A-Za-z0-9._-]+$ ]] || { echo "Invalid version: ${VERSION}" >&2; exit 2; }
[[ "${SOURCE_DATE_EPOCH:-0}" =~ ^[0-9]+$ ]] || { echo "Invalid SOURCE_DATE_EPOCH" >&2; exit 2; }
for command_name in git gzip rsync sha256sum tar; do
  command -v "${command_name}" >/dev/null 2>&1 || {
    echo "Required command not found: ${command_name}" >&2
    exit 1
  }
done

for required_file in \
  LICENSE LICENSING.md \
  deployment/audit_permissive_source.sh \
  deployment/public_overrides/PERMISSIVE_INSTALL_SH \
  deployment/public_overrides/PERMISSIVE_THIRD_PARTY_NOTICES.md \
  deployment/public_overrides/PERMISSIVE_TOP_LEVEL_GITATTRIBUTES; do
  [[ -f "${ROOT_DIR}/${required_file}" ]] || {
    echo "Required source file missing: ${required_file}" >&2
    exit 1
  }
done
if grep -q 'owner has not yet approved\|no repository-wide license grant' \
  "${ROOT_DIR}/LICENSING.md"; then
  echo "Owner-license blocker is not resolved in LICENSING.md." >&2
  exit 3
fi
if [[ -f "${ROOT_DIR}/SOURCE_INVENTORY.sha256" ]] && \
   ! (cd "${ROOT_DIR}" && sha256sum --quiet -c SOURCE_INVENTORY.sha256); then
  echo "Source inventory verification failed; refusing to repackage modified source." >&2
  exit 1
fi

BUNDLE_NAME="navigation-permissive-source-${VERSION}"
OUTPUT_DIR="$(realpath -m "${OUTPUT_DIR}")"
mkdir -p "${OUTPUT_DIR}"
TEMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TEMP_DIR}"' EXIT INT TERM
STAGE="${TEMP_DIR}/${BUNDLE_NAME}"
mkdir -p "${STAGE}"

RSLIDAR_TOP_COMMIT="8b4b4b7ff910799260347821084c59e1c73d50d5"
RSLIDAR_RS_DRIVER_COMMIT="897b14d3bdb6186a75df27ba51b65b5bd5557723"

copy_file() {
  local relative_path="$1"
  [[ -f "${ROOT_DIR}/${relative_path}" ]] || {
    echo "Required source file missing: ${relative_path}" >&2
    exit 1
  }
  mkdir -p "${STAGE}/$(dirname "${relative_path}")"
  cp -a "${ROOT_DIR}/${relative_path}" "${STAGE}/${relative_path}"
}

common_excludes=(
  --exclude='.git' --exclude='.git/' --exclude='.svn/'
  --exclude='.github/' --exclude='.idea/' --exclude='.vscode/'
  --exclude='.cache/' --exclude='.pytest_cache/' --exclude='__pycache__/'
  --exclude='build/' --exclude='install/' --exclude='devel/'
  --exclude='log/' --exclude='Log/' --exclude='obj/'
  --exclude='doc/' --exclude='PCD/' --exclude='maps/' --exclude='recordings/'
  --exclude='navigation.env'
  --exclude='*.o' --exclude='*.obj' --exclude='*.a' --exclude='*.so'
  --exclude='*.pyc' --exclude='*.pcd' --exclude='*.ply' --exclude='*.stl'
  --exclude='*.pickle' --exclude='*.pkl' --exclude='*.npy'
  --exclude='*.bag' --exclude='*.db3' --exclude='*.mcap' --exclude='*.lvx'
  --exclude='*.zip' --exclude='*.tar' --exclude='*.tar.gz' --exclude='*.tgz'
  --exclude='*.pdf' --exclude='*.docx' --exclude='*.xls' --exclude='*.xlsx'
  --exclude='rs_driverConfig.cmake' --exclude='rs_driverConfigVersion.cmake'
)

copy_rslidar_tree() {
  local top="${ROOT_DIR}/RSLIDAR_SDK_ROS2"
  local driver="${top}/src/rs_driver"
  local export_root="${TEMP_DIR}/rslidar"
  mkdir -p "${export_root}"
  if git -C "${top}" cat-file -e "${RSLIDAR_TOP_COMMIT}^{commit}" 2>/dev/null && \
     git -C "${driver}" cat-file -e "${RSLIDAR_RS_DRIVER_COMMIT}^{commit}" 2>/dev/null; then
    git -C "${top}" archive --format=tar "${RSLIDAR_TOP_COMMIT}" \
      | tar -xf - -C "${export_root}"
    rm -rf "${export_root}/src/rs_driver"
    mkdir -p "${export_root}/src/rs_driver"
    git -C "${driver}" archive --format=tar "${RSLIDAR_RS_DRIVER_COMMIT}" \
      | tar -xf - -C "${export_root}/src/rs_driver"
    for local_file in .gitignore CMakeLists.txt config/config.yaml package.xml NOTICE; do
      mkdir -p "${export_root}/$(dirname "${local_file}")"
      cp -a "${top}/${local_file}" "${export_root}/${local_file}"
    done
  else
    [[ -f "${ROOT_DIR}/SOURCE_INVENTORY.sha256" ]] || {
      echo "Pinned RoboSense Git objects are unavailable and no source inventory exists." >&2
      return 1
    }
    rsync -a --prune-empty-dirs "${common_excludes[@]}" \
      "${top}/" "${export_root}/"
  fi
  mkdir -p "${STAGE}/RSLIDAR_SDK_ROS2"
  rsync -a --prune-empty-dirs "${common_excludes[@]}" \
    "${export_root}/" "${STAGE}/RSLIDAR_SDK_ROS2/"
}

copy_scan_tree() {
  local source="${ROOT_DIR}/SCAN-Planner-ros2-community"
  local excludes=(
    --exclude='assets/' --exclude='src/simulator/'
    --exclude='src/planner/plan_manage/launch/default.rviz'
    --exclude='src/planner/plan_manage/launch/run.launch.py'
    --exclude='src/planner/plan_manage/launch/rviz.launch.py'
    --exclude='src/planner/plan_manage/launch/simulator.launch.py'
    --exclude='src/planner/plan_manage/config/simulator.yaml'
  )
  mkdir -p "${STAGE}/SCAN-Planner-ros2-community"
  rsync -a --prune-empty-dirs "${common_excludes[@]}" "${excludes[@]}" \
    "${source}/" "${STAGE}/SCAN-Planner-ros2-community/"
}

rewrite_readme_links() {
  local readme="$1"
  local raw_root="$2"
  [[ -f "${readme}" ]] || return 0
  sed -i \
    -e "s#](\\./doc/#](${raw_root}doc/#g" \
    -e "s#](doc/#](${raw_root}doc/#g" \
    -e "s#](\\./img/#](${raw_root}img/#g" \
    -e "s#](img/#](${raw_root}img/#g" \
    -e "s#src=\"\\./doc/#src=\"${raw_root}doc/#g" \
    -e "s#src=\"doc/#src=\"${raw_root}doc/#g" \
    -e "s#src=\"\\./img/#src=\"${raw_root}img/#g" \
    -e "s#src=\"img/#src=\"${raw_root}img/#g" \
    "${readme}"
  sed -i '1i> **Documentation note:** omitted upstream media links target the pinned upstream snapshot.\n' "${readme}"
}

root_files=(
  LICENSE
  LICENSING.md
  README_zh-CN.md
  build_fast_lio_ros2_humble.sh
  navi.sh
  prepare_fast_lio_pcd_map.sh
  prepare_pct_map.sh
  run_fast_lio_mapping_humble.sh
  run_fast_lio_navigation_humble.sh
  run_local_fast_lio_mapping_rviz_foxy.sh
  run_local_rviz_foxy.sh
  run_nx_navigation_humble.sh
  select_navigation_map.py
  select_pcd_map.py
  stop_nx_navigation_humble.sh
  switch_pct_map.sh
  tests/test_dual_map_configuration.py
  tests/test_full_nx_installer.py
  tests/test_select_navigation_map.py
  tests/test_select_pcd_map.py
  config/navigation.env.example
  docs/GPL_EXTERNAL_DEPENDENCIES.md
  docs/HARDWARE_COMPATIBILITY.md
  docs/PCT_EXTERNAL_PROVENANCE.md
  deployment/audit_permissive_source.sh
  deployment/check_markdown_links.py
  deployment/create_permissive_source_bundle.sh
  deployment/install_full_navigation_nx.sh
  deployment/install_permissive_source_nx.sh
  deployment/link_external_gpl_sources.sh
  deployment/map-tools-constraints.txt
  deployment/public_overrides/PERMISSIVE_INSTALL_SH
  deployment/public_overrides/PERMISSIVE_NON_TEXT_SHA256SUMS
  deployment/public_overrides/PERMISSIVE_THIRD_PARTY_NOTICES.md
  deployment/public_overrides/PERMISSIVE_TOP_LEVEL_GITATTRIBUTES
  deployment/public_overrides/PERMISSIVE_TOP_LEVEL_GITIGNORE
  deployment/public_overrides/SCAN_PLANNER_README.md
  deployment/public_overrides/scan_planner_package.xml
)
for relative_path in "${root_files[@]}"; do
  copy_file "${relative_path}"
done
if [[ -f "${ROOT_DIR}/docs/PERMISSIVE_SOURCE_README.md" ]]; then
  cp -a "${ROOT_DIR}/docs/PERMISSIVE_SOURCE_README.md" "${STAGE}/README.md"
else
  cp -a "${ROOT_DIR}/README.md" "${STAGE}/README.md"
fi
cp -a "${ROOT_DIR}/deployment/public_overrides/PERMISSIVE_TOP_LEVEL_GITIGNORE" "${STAGE}/.gitignore"
cp -a "${ROOT_DIR}/deployment/public_overrides/PERMISSIVE_TOP_LEVEL_GITATTRIBUTES" "${STAGE}/.gitattributes"
cp -a "${ROOT_DIR}/deployment/public_overrides/PERMISSIVE_INSTALL_SH" "${STAGE}/install.sh"
cp -a "${ROOT_DIR}/deployment/public_overrides/PERMISSIVE_THIRD_PARTY_NOTICES.md" \
  "${STAGE}/THIRD_PARTY_NOTICES.md"

mkdir -p "${STAGE}/RSLIDAR_MSG_ROS2"
rsync -a --prune-empty-dirs "${common_excludes[@]}" \
  "${ROOT_DIR}/RSLIDAR_MSG_ROS2/" "${STAGE}/RSLIDAR_MSG_ROS2/"
copy_rslidar_tree
copy_scan_tree

# Apply only pinned plaintext mirrors belonging to included permissive trees.
plaintext_source="${ROOT_DIR}/deployment/public_overrides/plaintext"
plaintext_stage="${STAGE}/deployment/public_overrides/plaintext"
mkdir -p "${plaintext_stage}"
: >"${plaintext_stage}/SHA256SUMS"
while read -r checksum relative_path; do
  case "${relative_path}" in
    RSLIDAR_SDK_ROS2/*|SCAN-Planner-ros2-community/*)
      [[ "$(sha256sum "${plaintext_source}/${relative_path}" | awk '{print $1}')" == "${checksum}" ]] || {
        echo "Plaintext mirror checksum changed: ${relative_path}" >&2
        exit 1
      }
      mkdir -p "${plaintext_stage}/$(dirname "${relative_path}")"
      mkdir -p "${STAGE}/$(dirname "${relative_path}")"
      cp -a "${plaintext_source}/${relative_path}" "${plaintext_stage}/${relative_path}"
      cp -a "${plaintext_source}/${relative_path}" "${STAGE}/${relative_path}"
      printf '%s  %s\n' "${checksum}" "${relative_path}" >>"${plaintext_stage}/SHA256SUMS"
      ;;
  esac
done <"${plaintext_source}/SHA256SUMS"

cp -a "${ROOT_DIR}/deployment/public_overrides/scan_planner_package.xml" \
  "${STAGE}/SCAN-Planner-ros2-community/src/planner/plan_manage/package.xml"
cp -a "${ROOT_DIR}/deployment/public_overrides/SCAN_PLANNER_README.md" \
  "${STAGE}/SCAN-Planner-ros2-community/README.md"

for readme in README.md README_CN.md; do
  rewrite_readme_links "${STAGE}/RSLIDAR_SDK_ROS2/${readme}" \
    "https://raw.githubusercontent.com/RoboSense-LiDAR/rslidar_sdk/${RSLIDAR_TOP_COMMIT}/"
  rewrite_readme_links "${STAGE}/RSLIDAR_SDK_ROS2/src/rs_driver/${readme}" \
    "https://raw.githubusercontent.com/RoboSense-LiDAR/rs_driver/${RSLIDAR_RS_DRIVER_COMMIT}/"
done
sed -i \
  -e 's|doc/howto/07_how_to_decode_online_lidar\.md|doc/howto/08_how_to_decode_online_lidar.md|g' \
  -e 's|doc/howto/09_how_to_decode_pcap_file\.md|doc/howto/10_how_to_decode_pcap_file.md|g' \
  -e 's|doc/howto/13_how_to_use_rs_driver_viewer\.md|doc/howto/14_how_to_use_rs_driver_viewer.md|g' \
  "${STAGE}/RSLIDAR_SDK_ROS2/src/rs_driver/README.md"
sed -i '/raw\.githubusercontent\.com\/RoboSense-LiDAR\/rs_driver\/.*\/img\/01_01_install_pcl\.png/d' \
  "${STAGE}/RSLIDAR_SDK_ROS2/src/rs_driver/README.md" \
  "${STAGE}/RSLIDAR_SDK_ROS2/src/rs_driver/README_CN.md"

# Normalize Git-relevant mode bits before inventory and archive creation.
find "${STAGE}" -type d -exec chmod 0755 {} +
find "${STAGE}" -type f -exec chmod 0644 {} +
while IFS= read -r -d '' path; do
  if cmp -s <(LC_ALL=C head -c 2 -- "${path}") <(printf '#!'); then
    chmod 0755 "${path}"
  fi
done < <(find "${STAGE}" -type f -print0)

(
  cd "${STAGE}"
  find . -type f ! -name SOURCE_INVENTORY.sha256 -print0 \
    | LC_ALL=C sort -z \
    | xargs -0 sha256sum
) >"${STAGE}/SOURCE_INVENTORY.sha256"
chmod 0644 "${STAGE}/SOURCE_INVENTORY.sha256"

PYTHON_EXECUTABLE="${PYTHON_EXECUTABLE:-python3}" \
  bash "${STAGE}/deployment/audit_permissive_source.sh" "${STAGE}"

find "${STAGE}" -exec touch -h -d "@${SOURCE_DATE_EPOCH:-0}" {} +
archive="${OUTPUT_DIR}/${BUNDLE_NAME}.tar.gz"
sidecar="${archive}.sha256"
rm -f "${archive}" "${sidecar}"
(
  cd "${TEMP_DIR}"
  LC_ALL=C tar --sort=name \
    --owner=0 --group=0 --numeric-owner \
    --mtime="@${SOURCE_DATE_EPOCH:-0}" \
    -cf - "${BUNDLE_NAME}" | gzip -n -9 >"${archive}"
)
(
  cd "${OUTPUT_DIR}"
  sha256sum "$(basename "${archive}")" >"$(basename "${sidecar}")"
)

printf 'PERMISSIVE_SOURCE_BUNDLE=%s\n' "${archive}"
printf 'PERMISSIVE_SOURCE_SHA256=%s\n' "$(sha256sum "${archive}" | awk '{print $1}')"
printf 'PERMISSIVE_SOURCE_SCOPE=LimX+SCAN+RSLIDAR\n'
