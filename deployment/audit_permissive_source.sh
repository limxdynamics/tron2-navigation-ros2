#!/usr/bin/env bash
# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

set -Eeuo pipefail

usage() {
  echo "Usage: ./deployment/audit_permissive_source.sh SOURCE_TREE" >&2
}

[[ $# -eq 1 ]] || { usage; exit 2; }
SOURCE_TREE="$(realpath "$1")"
[[ -d "${SOURCE_TREE}" ]] || { echo "Source tree not found: ${SOURCE_TREE}" >&2; exit 2; }
for command_name in file git sha256sum; do
  command -v "${command_name}" >/dev/null 2>&1 || {
    echo "Required command not found: ${command_name}" >&2
    exit 2
  }
done

# Git object storage is not release payload. Audit an otherwise identical copy
# so the same command works against both an extracted archive and a fresh clone.
if [[ -e "${SOURCE_TREE}/.git" || -L "${SOURCE_TREE}/.git" ]]; then
  audit_copy_root="$(mktemp -d)"
  trap 'rm -rf "${audit_copy_root}"' EXIT INT TERM
  mkdir -p "${audit_copy_root}/source"
  cp -a "${SOURCE_TREE}/." "${audit_copy_root}/source/"
  rm -rf "${audit_copy_root}/source/.git"
  bash "${audit_copy_root}/source/deployment/audit_permissive_source.sh" \
    "${audit_copy_root}/source"
  exit $?
fi

failures=0
fail() {
  printf 'AUDIT_FAIL: %s\n' "$*" >&2
  failures=$((failures + 1))
}

# Private identifiers are deliberately absent from this file. The script ships
# with the released tree, so an internal file name, an internal repository name,
# or a private review artefact written here would itself become public. Supply
# them from outside the tree instead: point PERMISSIVE_AUDIT_INTERNAL_DENYLIST at
# a file holding one extended regular expression per line (blank lines and lines
# starting with '#' are ignored). Every expression is matched against both the
# content and the relative path of each exported file. When the variable is
# unset the check reports SKIPPED, so a build can assert that it actually ran.
internal_denylist_path="${PERMISSIVE_AUDIT_INTERNAL_DENYLIST:-}"
internal_denylist_patterns=()
if [[ -n "${internal_denylist_path}" ]]; then
  if [[ ! -f "${internal_denylist_path}" ]]; then
    printf 'Internal denylist not found: %s\n' "${internal_denylist_path}" >&2
    exit 2
  fi
  while IFS= read -r denylist_line; do
    denylist_line="$(printf '%s' "${denylist_line}" | tr -d '\r')"
    [[ -n "${denylist_line}" && "${denylist_line}" != '#'* ]] || continue
    internal_denylist_patterns+=("${denylist_line}")
  done <"${internal_denylist_path}"
  echo "PERMISSIVE_AUDIT_INTERNAL_IDENTIFIERS=LOADED patterns=${#internal_denylist_patterns[@]}"
else
  echo 'PERMISSIVE_AUDIT_INTERNAL_IDENTIFIERS=SKIPPED (PERMISSIVE_AUDIT_INTERNAL_DENYLIST not set)'
fi

required_files=(
  .gitattributes
  .gitignore
  LICENSE
  LICENSING.md
  README.md
  README_zh-CN.md
  THIRD_PARTY_NOTICES.md
  SOURCE_INVENTORY.sha256
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
  deployment/public_overrides/plaintext/SHA256SUMS
  deployment/public_overrides/scan_planner_package.xml
  install.sh
  tests/test_dual_map_configuration.py
  tests/test_full_nx_installer.py
  tests/test_select_navigation_map.py
  tests/test_select_pcd_map.py
  RSLIDAR_MSG_ROS2/LICENSE
  RSLIDAR_MSG_ROS2/NOTICE
  RSLIDAR_MSG_ROS2/package.xml
  RSLIDAR_SDK_ROS2/LICENSE
  RSLIDAR_SDK_ROS2/NOTICE
  RSLIDAR_SDK_ROS2/package.xml
  RSLIDAR_SDK_ROS2/src/rs_driver/LICENSE
  SCAN-Planner-ros2-community/LICENSE
  SCAN-Planner-ros2-community/NOTICE
  SCAN-Planner-ros2-community/UPSTREAM_COMMIT
  SCAN-Planner-ros2-community/src/planner/plan_env/include/plan_env/grid_map.h
  SCAN-Planner-ros2-community/src/planner/plan_env/src/grid_map.cpp
  SCAN-Planner-ros2-community/src/planner/plan_manage/launch/nx_pct_navigation.launch.py
  SCAN-Planner-ros2-community/src/planner/plan_manage/package.xml
  SCAN-Planner-ros2-community/src/planner/plan_manage/test/test_chassis_configuration.py
)
for relative_path in "${required_files[@]}"; do
  [[ -f "${SOURCE_TREE}/${relative_path}" ]] || fail "missing required file: ${relative_path}"
done

if [[ -f "${SOURCE_TREE}/LICENSE" ]]; then
  grep -Fq 'Apache License' "${SOURCE_TREE}/LICENSE" \
    || fail "top-level LICENSE is not Apache-2.0 text"
  grep -Fq 'Version 2.0, January 2004' "${SOURCE_TREE}/LICENSE" \
    || fail "top-level LICENSE version is not Apache-2.0"
fi
if grep -q 'owner has not yet approved\|no repository-wide license grant' \
  "${SOURCE_TREE}/LICENSING.md" 2>/dev/null; then
  fail "license-scope document still reports an unresolved owner-license blocker"
fi
for scope_text in \
  'original top-level' \
  'Apache-2.0' \
  'must remain outside this permissive-source repository'; do
  grep -Fq "${scope_text}" "${SOURCE_TREE}/LICENSING.md" 2>/dev/null \
    || fail "LICENSING.md is missing scope text: ${scope_text}"
done

if ! cmp -s "${SOURCE_TREE}/.gitignore" \
  "${SOURCE_TREE}/deployment/public_overrides/PERMISSIVE_TOP_LEVEL_GITIGNORE"; then
  fail "top-level .gitignore does not match the permissive template"
fi
if ! cmp -s "${SOURCE_TREE}/.gitattributes" \
  "${SOURCE_TREE}/deployment/public_overrides/PERMISSIVE_TOP_LEVEL_GITATTRIBUTES"; then
  fail "top-level .gitattributes does not match the permissive template"
fi
if ! cmp -s "${SOURCE_TREE}/install.sh" \
  "${SOURCE_TREE}/deployment/public_overrides/PERMISSIVE_INSTALL_SH"; then
  fail "top-level installer does not match the permissive template"
fi
if ! cmp -s "${SOURCE_TREE}/THIRD_PARTY_NOTICES.md" \
  "${SOURCE_TREE}/deployment/public_overrides/PERMISSIVE_THIRD_PARTY_NOTICES.md"; then
  fail "third-party notice does not match the permissive template"
fi

for forbidden_root in FAST_LIO FAST_LIO_LOCALIZATION2 PCT_planner-RC2026_Map_Planner PCT_planner; do
  if [[ -e "${SOURCE_TREE}/${forbidden_root}" || -L "${SOURCE_TREE}/${forbidden_root}" ]]; then
    fail "GPL source root is present: ${forbidden_root}"
  fi
done

external_record="${SOURCE_TREE}/docs/GPL_EXTERNAL_DEPENDENCIES.md"
gpl_linker="${SOURCE_TREE}/deployment/link_external_gpl_sources.sh"

# Upstream provenance pins. These name public upstream projects, and pinning them
# is the purpose of the record.
for required_upstream_record in \
  'https://github.com/Ericsii/FAST_LIO.git' \
  '2fffc570a25d0df172720bac034fbdb6a13d2162' \
  'https://github.com/Smart-Wheelchair-RRC/FAST_LIO_LOCALIZATION2.git' \
  'f04974907c8da976dd0495b272d18ac4c534d41f' \
  'https://github.com/byangw/PCT_planner.git' \
  '35cd73fd82bcd51bc538429294af7646b2a09815'; do
  grep -Fq "${required_upstream_record}" "${external_record}" 2>/dev/null \
    || fail "external GPL dependency record is missing: ${required_upstream_record}"
done

# The separately licensed companion repositories are matched by the naming
# convention they share with this repository, never by an account name: this
# script ships with the released tree, and a pinned owner would both publish an
# individual's account and break the gate the day the repositories move.
# What is asserted instead is agreement - the record and the fetch-and-verify
# script must name the same repositories at the same immutable commits and tags.
companion_slug_pattern='https://github\.com/[A-Za-z0-9._-]+/[A-Za-z0-9._-]*gpl[A-Za-z0-9._-]*'
record_slugs="$(mktemp)"
linker_slugs="$(mktemp)"
grep -oE -- "${companion_slug_pattern}" "${external_record}" 2>/dev/null \
  | sed -e 's#^https://github\.com/##' -e 's#\.git$##' \
  | LC_ALL=C sort -u >"${record_slugs}" || true
grep -oE -- "${companion_slug_pattern}" "${gpl_linker}" 2>/dev/null \
  | sed -e 's#^https://github\.com/##' -e 's#\.git$##' \
  | LC_ALL=C sort -u >"${linker_slugs}" || true
if [[ ! -s "${record_slugs}" ]]; then
  fail "no separately licensed companion repository is recorded in docs/GPL_EXTERNAL_DEPENDENCIES.md"
elif ! cmp -s "${record_slugs}" "${linker_slugs}"; then
  fail "companion GPL repository URLs disagree between the record and link_external_gpl_sources.sh"
  diff -u "${record_slugs}" "${linker_slugs}" >&2 || true
else
  while IFS= read -r companion_slug; do
    [[ -n "${companion_slug}" ]] || continue
    record_row="$(grep -F -- "${companion_slug}" "${external_record}" | head -n 1 || true)"
    pinned_commit="$(printf '%s\n' "${record_row}" | grep -oE '[0-9a-f]{40}' | head -n 1 || true)"
    pinned_tag="$(printf '%s\n' "${record_row}" | grep -oE 'v[0-9]+\.[0-9]+\.[0-9]+' | head -n 1 || true)"
    if [[ -z "${pinned_commit}" ]]; then
      fail "companion GPL repository has no recorded commit: ${companion_slug}"
    elif ! grep -Fq -- "${pinned_commit}" "${gpl_linker}"; then
      fail "recorded commit for ${companion_slug} is not the commit the link script verifies"
    fi
    if [[ -z "${pinned_tag}" ]]; then
      fail "companion GPL repository has no recorded release tag: ${companion_slug}"
    elif ! grep -Fq -- "${pinned_tag}" "${gpl_linker}"; then
      fail "recorded release tag for ${companion_slug} is not the tag the link script checks out"
    fi
  done <"${record_slugs}"
fi
rm -f "${record_slugs}" "${linker_slugs}"

license_is_exact() {
  local package_file="$1"
  local expected="$2"
  grep -Eq "<license>[[:space:]]*${expected}[[:space:]]*</license>" \
    "${package_file}" 2>/dev/null
}
license_is_exact "${SOURCE_TREE}/RSLIDAR_MSG_ROS2/package.xml" BSD-3-Clause \
  || fail "rslidar_msg package license is not BSD-3-Clause"
license_is_exact "${SOURCE_TREE}/RSLIDAR_SDK_ROS2/package.xml" BSD-3-Clause \
  || fail "rslidar_sdk package license is not BSD-3-Clause"
for package_file in \
  "${SOURCE_TREE}"/SCAN-Planner-ros2-community/src/planner/*/package.xml; do
  [[ -f "${package_file}" ]] || continue
  license_is_exact "${package_file}" Apache-2.0 \
    || fail "SCAN package license is not Apache-2.0: ${package_file#"${SOURCE_TREE}/"}"
done

scan_root="${SOURCE_TREE}/SCAN-Planner-ros2-community"
if ! cmp -s "${scan_root}/README.md" \
  "${SOURCE_TREE}/deployment/public_overrides/SCAN_PLANNER_README.md"; then
  fail "SCAN README is not the pinned production-only version"
fi
if ! cmp -s "${scan_root}/src/planner/plan_manage/package.xml" \
  "${SOURCE_TREE}/deployment/public_overrides/scan_planner_package.xml"; then
  fail "SCAN package manifest is not the pinned production-only version"
fi
for forbidden_scan_path in \
  src/simulator \
  src/planner/plan_manage/launch/default.rviz \
  src/planner/plan_manage/launch/run.launch.py \
  src/planner/plan_manage/launch/rviz.launch.py \
  src/planner/plan_manage/launch/simulator.launch.py \
  src/planner/plan_manage/config/simulator.yaml; do
  if [[ -e "${scan_root}/${forbidden_scan_path}" || -L "${scan_root}/${forbidden_scan_path}" ]]; then
    fail "SCAN simulator/demo path is present: ${forbidden_scan_path}"
  fi
done
scan_demo_pattern='<(depend|exec_depend)>[[:space:]]*(go2_description|livox_ros_driver2|local_sensing_node|map_generator|mockamap|odom_visualization|robot_state_publisher|xacro)[[:space:]]*</(depend|exec_depend)>'
if grep -RInI -E --include='package.xml' -- "${scan_demo_pattern}" "${scan_root}" >/dev/null; then
  fail "SCAN still declares an excluded simulator/demo dependency"
fi

# Private bundle and legacy artefact names are not listed here - see
# PERMISSIVE_AUDIT_INTERNAL_DENYLIST at the top of this script. Naming them in
# the released tree is what made them public in the first place.

mapfile -d '' forbidden_directories < <(
  find "${SOURCE_TREE}" -mindepth 1 -type d \
    \( -name .git -o -name .svn -o -name build -o -name install \
       -o -name devel -o -name obj -o -iname log -o -name __pycache__ \
       -o -name .pytest_cache -o -name .cache -o -name recordings \
       -o -name portable_packages -o -name public_source_packages \) -print0
)
if (( ${#forbidden_directories[@]} > 0 )); then
  fail "generated/private directories found"
  printf '  %s\n' "${forbidden_directories[@]}" >&2
fi

mapfile -d '' forbidden_files < <(
  find "${SOURCE_TREE}" -type f \
    \( -name '*.o' -o -name '*.obj' -o -name '*.a' -o -name '*.so' \
       -o -name '*.pyc' -o -name '*.pcd' -o -name '*.ply' -o -name '*.stl' \
       -o -name '*.pickle' -o -name '*.pkl' -o -name '*.npy' \
       -o -name '*.bag' -o -name '*.db3' -o -name '*.mcap' -o -name '*.lvx' \
       -o -name '*.zip' -o -name '*.tar' -o -name '*.tar.gz' -o -name '*.tgz' \
       -o -name '*.pdf' -o -name '*.docx' -o -name '*.xls' -o -name '*.xlsx' \
       -o -name navigation.env \) -print0
)
if (( ${#forbidden_files[@]} > 0 )); then
  fail "runtime, map, archive, or generated files found"
  printf '  %s\n' "${forbidden_files[@]}" >&2
fi

if find "${SOURCE_TREE}" -name .gitattributes -type f -print0 \
  | xargs -0 -r grep -Il 'filter=lfs' | grep -q .; then
  fail "Git LFS rules remain in the map-free permissive repository"
fi

protected_header_pattern='%TS''[DZ]-Header-'
protected_magic_pattern='^(TS''[DZ])#'
protected_source_output="$(
  LC_ALL=C grep -R -a -l -m 1 -E -- \
    "${protected_magic_pattern}|${protected_header_pattern}" "${SOURCE_TREE}" || true
)"
if [[ -n "${protected_source_output}" ]]; then
  fail "endpoint-protected source envelopes found"
  printf '%s\n' "${protected_source_output}" >&2
fi

inventory_file="${SOURCE_TREE}/SOURCE_INVENTORY.sha256"
if [[ -f "${inventory_file}" ]]; then
  if ! (cd "${SOURCE_TREE}" && sha256sum -c SOURCE_INVENTORY.sha256 >/dev/null); then
    fail "source inventory checksum verification failed"
  fi
  inventory_paths="$(mktemp)"
  actual_paths="$(mktemp)"
  awk '{ sub(/^\.\//, "", $2); print $2 }' "${inventory_file}" | LC_ALL=C sort >"${inventory_paths}"
  find "${SOURCE_TREE}" -type f ! -name SOURCE_INVENTORY.sha256 -printf '%P\n' \
    | LC_ALL=C sort >"${actual_paths}"
  cmp -s "${inventory_paths}" "${actual_paths}" \
    || fail "source inventory does not exactly match the file set"
  rm -f "${inventory_paths}" "${actual_paths}"
fi

plaintext_root="${SOURCE_TREE}/deployment/public_overrides/plaintext"
if [[ -f "${plaintext_root}/SHA256SUMS" ]]; then
  if ! (cd "${plaintext_root}" && sha256sum -c SHA256SUMS >/dev/null); then
    fail "pinned plaintext checksums do not match"
  fi
  if grep -Eq '[[:space:]]+(FAST_LIO|FAST_LIO_LOCALIZATION2|PCT_planner)' \
    "${plaintext_root}/SHA256SUMS"; then
    fail "GPL plaintext mirror is present"
  fi
  while read -r checksum relative_path; do
    [[ "${checksum}" =~ ^[0-9a-f]{64}$ ]] || { fail "bad plaintext checksum"; continue; }
    if ! cmp -s "${plaintext_root}/${relative_path}" "${SOURCE_TREE}/${relative_path}"; then
      fail "exported source differs from pinned plaintext: ${relative_path}"
    fi
  done <"${plaintext_root}/SHA256SUMS"
fi

non_text_manifest="${SOURCE_TREE}/deployment/public_overrides/PERMISSIVE_NON_TEXT_SHA256SUMS"
allowed_non_text="$(mktemp)"
actual_non_text="$(mktemp)"
if [[ -f "${non_text_manifest}" ]]; then
  (cd "${SOURCE_TREE}" && sha256sum -c \
    deployment/public_overrides/PERMISSIVE_NON_TEXT_SHA256SUMS >/dev/null) \
    || fail "non-text allowlist checksum verification failed"
  awk '{ print $2 }' "${non_text_manifest}" | LC_ALL=C sort >"${allowed_non_text}"
fi
while IFS= read -r -d '' path; do
  [[ -s "${path}" ]] || continue
  encoding="$(file -Lb --mime-encoding "${path}")"
  if [[ "${encoding}" != us-ascii && "${encoding}" != utf-8 ]]; then
    printf '%s\n' "${path#"${SOURCE_TREE}/"}" >>"${actual_non_text}"
  fi
done < <(find "${SOURCE_TREE}" -type f -print0)
LC_ALL=C sort -o "${actual_non_text}" "${actual_non_text}"
cmp -s "${allowed_non_text}" "${actual_non_text}" || {
  fail "non-text file set differs from the pinned allowlist"
  diff -u "${allowed_non_text}" "${actual_non_text}" >&2 || true
}
rm -f "${allowed_non_text}" "${actual_non_text}"

mapfile -d '' oversized_files < <(find "${SOURCE_TREE}" -type f -size +25M -print0)
if (( ${#oversized_files[@]} > 0 )); then
  fail "files larger than 25 MiB found"
  printf '  %s\n' "${oversized_files[@]}" >&2
fi

long_paths=()
while IFS= read -r -d '' path; do
  relative_path="${path#"${SOURCE_TREE}/"}"
  (( ${#relative_path} <= 180 )) || long_paths+=("${#relative_path} ${relative_path}")
done < <(find "${SOURCE_TREE}" -mindepth 1 -print0)
if (( ${#long_paths[@]} > 0 )); then
  fail "paths longer than 180 characters found"
  printf '  %s\n' "${long_paths[@]}" >&2
fi

mapfile -d '' escaping_links < <(
  find "${SOURCE_TREE}" -type l -print0 | while IFS= read -r -d '' link; do
    target="$(realpath -m "${link}")"
    [[ "${target}" == "${SOURCE_TREE}"/* ]] || printf '%s\0' "${link} -> $(readlink "${link}")"
  done
)
if (( ${#escaping_links[@]} > 0 )); then
  fail "symlinks escaping the source tree found"
  printf '  %s\n' "${escaping_links[@]}" >&2
fi

bad_modes=()
while IFS= read -r -d '' path; do
  mode="$(stat -c '%a' "${path}")"
  value=$((8#${mode}))
  relative_path="${path#"${SOURCE_TREE}/"}"
  if (( (value & 0002) != 0 || (value & 06000) != 0 )); then
    bad_modes+=("${mode} unsafe ${relative_path}")
  elif [[ -d "${path}" ]]; then
    (( (value & 0100) != 0 )) || bad_modes+=("${mode} no-owner-search ${relative_path}")
  elif cmp -s <(LC_ALL=C head -c 2 -- "${path}") <(printf '#!'); then
    (( (value & 0111) != 0 )) || bad_modes+=("${mode} script-not-executable ${relative_path}")
  else
    (( (value & 0111) == 0 )) || bad_modes+=("${mode} ordinary-file-executable ${relative_path}")
  fi
done < <(find "${SOURCE_TREE}" -mindepth 1 \( -type f -o -type d \) -print0)
if (( ${#bad_modes[@]} > 0 )); then
  fail "unsafe or noncanonical executable bits found"
  printf '  %s\n' "${bad_modes[@]}" >&2
fi

git_probe="$(mktemp -d)"
ignored_paths="$(mktemp)"
git -C "${git_probe}" init -q
(
  cd "${SOURCE_TREE}"
  GIT_DIR="${git_probe}/.git" GIT_WORK_TREE="${SOURCE_TREE}" \
    git ls-files --others --ignored --exclude-standard -z || true
) >"${ignored_paths}"
mapfile -d '' ignored_files <"${ignored_paths}"
if (( ${#ignored_files[@]} > 0 )); then
  fail "exported files would be omitted by normal git add"
  printf '  %s\n' "${ignored_files[@]}" >&2
fi
rm -rf "${git_probe}" "${ignored_paths}"

# A literal that appears in this file is a literal this file has to skip when it
# scans itself, so each one is assembled from adjacent quoted fragments - the same
# technique the protected-envelope patterns above already use. Nothing is excluded
# from its own scan: excluding a file from the gate is what creates a blind spot.
private_value_pattern='(^|[^0-9])(10\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}|172\.(1[6-9]|2[0-9]|3[01])\.[0-9]{1,3}\.[0-9]{1,3}|192\.168\.[0-9]{1,3}\.[0-9]{1,3})([^0-9]|$)|WF_''TRON[A-Za-z0-9_]*|guest''@|ssh://''git@|/var/''limx'
machine_path_pattern='/home/[A-Za-z0-9_.-]+(/|[^A-Za-z0-9_])'
credential_pattern='ghp_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|glpat-[A-Za-z0-9_-]{20,}|AKIA[0-9A-Z]{16}'
sensitive_output="$(
  {
    grep -RInI -E -- "${private_value_pattern}|${credential_pattern}" \
      "${SOURCE_TREE}" || true
    grep -RInI -E --exclude-dir='RSLIDAR_SDK_ROS2' \
      -- "${machine_path_pattern}" "${SOURCE_TREE}" || true
  } | grep -v -F 'doi=10.1.1.42.3443' || true
)"
if [[ -n "${sensitive_output}" ]]; then
  fail "private address, machine path, credential, internal Git URL, or real robot identity found"
  printf '%s\n' "${sensitive_output}" >&2
fi

identity_output="$(
  grep -RInI -o -E -- 'WF_[A-Z0-9]+_[A-Z0-9]+' "${SOURCE_TREE}" \
    | while IFS= read -r match; do
        identity="${match##*:}"
        [[ "${identity}" =~ ^WF_(MODEL|TESTMODEL)_[0-9]+$ ]] || printf '%s\n' "${match}"
      done || true
)"
if [[ -n "${identity_output}" ]]; then
  fail "non-placeholder robot identity found"
  printf '%s\n' "${identity_output}" >&2
fi

# Generic release-control markers. Content markers, not file names: a private
# release-control file is caught by the marker it carries, whatever it is called.
release_marker_pattern='DO_NOT_''PUBLISH|RELEASE_''CANDIDATE'
release_marker_output="$(
  grep -RInI -E -- "${release_marker_pattern}" "${SOURCE_TREE}" || true
)"
if [[ -n "${release_marker_output}" ]]; then
  fail "release-control marker found"
  printf '%s\n' "${release_marker_output}" >&2
fi

if (( ${#internal_denylist_patterns[@]} > 0 )); then
  internal_identifier_pattern=''
  for denylist_pattern in "${internal_denylist_patterns[@]}"; do
    internal_identifier_pattern="${internal_identifier_pattern:+${internal_identifier_pattern}|}${denylist_pattern}"
  done
  internal_content_output="$(
    grep -RInI -E -- "${internal_identifier_pattern}" "${SOURCE_TREE}" || true
  )"
  if [[ -n "${internal_content_output}" ]]; then
    fail "internal-only identifier found in exported content"
    printf '%s\n' "${internal_content_output}" >&2
  fi
  internal_path_output="$(
    find "${SOURCE_TREE}" -mindepth 1 -printf '%P\n' \
      | LC_ALL=C grep -E -- "${internal_identifier_pattern}" || true
  )"
  if [[ -n "${internal_path_output}" ]]; then
    fail "internal-only identifier found in an exported path"
    printf '  %s\n' "${internal_path_output}" >&2
  fi
fi

# Every LimX-authored script in this repository carries the LimX Dynamics
# copyright header, and the top-level LICENSE names the copyright holder. Both
# are asserted here so that a later export cannot silently drop the attribution.
#
# Scripts are discovered by walking the tree instead of naming directories
# explicitly: an explicit list silently stops covering a directory the moment a
# new one is added, and this check would then keep passing while asserting
# nothing about the files inside it. The vendored third-party roots are excluded
# on purpose - their files keep upstream bytes and upstream attribution - and
# inside them the LimX-authored files are matched by the nx_*/limx_*/
# test_chassis_*/test_limx_* naming convention.
mapfile -d '' original_scripts < <(
  find "${SOURCE_TREE}" -type f \( -name '*.sh' -o -name '*.py' -o -name '*.bash' \) \
    -not -path "${SOURCE_TREE}/.git/*" \
    -not -path "${SOURCE_TREE}/SCAN-Planner-ros2-community/*" \
    -not -path "${SOURCE_TREE}/RSLIDAR_SDK_ROS2/*" \
    -not -path "${SOURCE_TREE}/RSLIDAR_MSG_ROS2/*" \
    -not -path "${SOURCE_TREE}/deployment/public_overrides/*" \
    -print0 | sort -z
)
for vendored_prefix in \
    "${SOURCE_TREE}"/SCAN-Planner-ros2-community/src/planner/plan_manage/launch/nx_*.py \
    "${SOURCE_TREE}"/SCAN-Planner-ros2-community/src/planner/plan_manage/scripts/limx_*.py \
    "${SOURCE_TREE}"/SCAN-Planner-ros2-community/src/planner/plan_manage/test/test_chassis_*.py \
    "${SOURCE_TREE}"/SCAN-Planner-ros2-community/src/planner/plan_manage/test/test_limx_*.py; do
  for vendored_script in ${vendored_prefix}; do
    if [[ -f "${vendored_script}" ]]; then
      original_scripts+=("${vendored_script}")
    fi
  done
done
if (( ${#original_scripts[@]} == 0 )); then
  fail "no LimX-authored scripts discovered, so the copyright header check would be vacuous"
fi
for original_script in "${original_scripts[@]}"; do
  [[ -f "${original_script}" ]] || continue
  grep -Fq 'Copyright information' "${original_script}" \
    && grep -Fq 'LimX Dynamics Technology Co., Ltd.' "${original_script}" \
    || fail "original script missing a LimX copyright header: ${original_script#"${SOURCE_TREE}/"}"
done
grep -Fq 'Copyright 2026 LimX Dynamics Technology Co., Ltd.' "${SOURCE_TREE}/LICENSE" \
  || fail "top-level LICENSE does not name the copyright holder"

python_executable="${PYTHON_EXECUTABLE:-python3}"
if ! command -v "${python_executable}" >/dev/null 2>&1; then
  fail "Python required for Markdown link check"
elif ! "${python_executable}" "${SOURCE_TREE}/deployment/check_markdown_links.py" "${SOURCE_TREE}"; then
  fail "broken local Markdown links found"
fi

if (( failures > 0 )); then
  printf 'PERMISSIVE_SOURCE_AUDIT=FAIL failures=%d\n' "${failures}" >&2
  exit 1
fi
echo 'PERMISSIVE_SOURCE_AUDIT=PASS'
