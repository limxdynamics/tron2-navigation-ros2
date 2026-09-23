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

SIBLING_ROOT="$(dirname "${ROOT_DIR}")"
FAST_LIO_GPL_ROOT="${FAST_LIO_GPL_ROOT:-${SIBLING_ROOT}/tron2-navigation-fastlio-gpl}"
PCT_GPL_ROOT="${PCT_GPL_ROOT:-${SIBLING_ROOT}/tron2-navigation-pct-gpl}"
FAST_LIO_GPL_URL="https://github.com/limxdynamics/tron2-navigation-fastlio-gpl.git"
PCT_GPL_URL="https://github.com/limxdynamics/tron2-navigation-pct-gpl.git"
FAST_LIO_COMMIT="0fbf6e9cca72a66c330a22d13823f52d49f05948"
PCT_COMMIT="1c553202e50798819715679a7f8c8a878b247ace"
RELEASE_TAG="v1.1.0"
VERIFY_ONLY=false
FETCH_MISSING=false

usage() {
  cat <<'EOF'
Usage: ./deployment/link_external_gpl_sources.sh [--fetch] [--verify-only]

By default, the script expects these sibling repositories:
  ../tron2-navigation-fastlio-gpl
  ../tron2-navigation-pct-gpl

Set FAST_LIO_GPL_ROOT or PCT_GPL_ROOT to use another absolute clone path.
With --fetch, a missing repository is cloned from its public GitHub URL at the
fixed v1.1.0 release. Existing paths are never fetched, checked out, or changed.
The script verifies each fixed commit, release tag, source inventory, license
check, and required source path. Unless --verify-only is used, it creates three
Git-ignored compatibility symlinks in the main repository. It never copies,
merges, or commits GPL source.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --fetch) FETCH_MISSING=true; shift ;;
    --verify-only) VERIFY_ONLY=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

for command_name in git realpath tar; do
  command -v "${command_name}" >/dev/null 2>&1 || {
    echo "Required command not found: ${command_name}" >&2
    exit 2
  }
done

verify_repository() {
  local repository="$1"
  local expected_commit="$2"
  local audit_script="$3"
  local label="$4"
  local actual_head actual_tag

  [[ "$(git -C "${repository}" rev-parse --is-inside-work-tree 2>/dev/null || true)" == true ]] || {
    echo "${label} repository not found: ${repository}" >&2
    return 1
  }
  actual_head="$(git -C "${repository}" rev-parse HEAD 2>/dev/null || true)"
  [[ "${actual_head}" == "${expected_commit}" ]] || {
    echo "${label} must be checked out at ${expected_commit}; current HEAD is ${actual_head:-unknown}." >&2
    return 1
  }
  actual_tag="$(git -C "${repository}" rev-parse "refs/tags/${RELEASE_TAG}^{commit}" 2>/dev/null || true)"
  [[ "${actual_tag}" == "${expected_commit}" ]] || {
    echo "${label} tag ${RELEASE_TAG} is missing or points to another commit." >&2
    return 1
  }
  if [[ -n "$(git -C "${repository}" status --porcelain --untracked-files=no)" ]]; then
    echo "${label} has tracked modifications; refusing unverified source." >&2
    return 1
  fi
  [[ -f "${repository}/${audit_script}" ]] || {
    echo "${label} source check is missing: ${audit_script}" >&2
    return 1
  }
  if ! (
    audit_root="$(mktemp -d)"
    trap 'rm -rf "${audit_root}"' EXIT
    git -C "${repository}" archive "${expected_commit}" | tar -xf - -C "${audit_root}"
    bash "${audit_root}/${audit_script}"
  ); then
    echo "${label} fixed source failed its integrity check." >&2
    return 1
  fi
  printf '%s_SOURCE=PASS commit=%s root=%s\n' \
    "${label}" "${expected_commit}" "$(realpath -e "${repository}")"
}

fetch_repository() {
  local repository="$1"
  local repository_url="$2"
  local expected_commit="$3"
  local label="$4"

  if [[ -e "${repository}" || -L "${repository}" ]]; then
    return 0
  fi

  mkdir -p "$(dirname "${repository}")"
  (
    local temporary_clone actual_head actual_tag
    temporary_clone="$(mktemp -d "$(dirname "${repository}")/.${label}.clone.XXXXXX")"
    trap 'rm -rf -- "${temporary_clone}"' EXIT

    echo "Fetching ${label} ${RELEASE_TAG} from ${repository_url}"
    git -c advice.detachedHead=false clone \
      --depth 1 --single-branch --branch "${RELEASE_TAG}" \
      "${repository_url}" "${temporary_clone}"
    actual_head="$(git -C "${temporary_clone}" rev-parse HEAD 2>/dev/null || true)"
    actual_tag="$(git -C "${temporary_clone}" rev-parse \
      "refs/tags/${RELEASE_TAG}^{commit}" 2>/dev/null || true)"
    if [[ "${actual_head}" != "${expected_commit}" || \
          "${actual_tag}" != "${expected_commit}" ]]; then
      echo "${label} ${RELEASE_TAG} did not resolve to ${expected_commit}." >&2
      exit 1
    fi
    if [[ -e "${repository}" || -L "${repository}" ]]; then
      echo "Destination appeared while cloning; refusing to replace it: ${repository}" >&2
      exit 1
    fi
    mv -T -- "${temporary_clone}" "${repository}"
  )
  printf '%s_FETCH=PASS commit=%s root=%s\n' \
    "${label}" "${expected_commit}" "$(realpath -e "${repository}")"
}

if [[ "${FETCH_MISSING}" == true ]]; then
  fetch_repository \
    "${FAST_LIO_GPL_ROOT}" "${FAST_LIO_GPL_URL}" "${FAST_LIO_COMMIT}" FAST_LIO_GPL
  fetch_repository \
    "${PCT_GPL_ROOT}" "${PCT_GPL_URL}" "${PCT_COMMIT}" PCT_GPL
fi

verify_repository \
  "${FAST_LIO_GPL_ROOT}" "${FAST_LIO_COMMIT}" audit_source.sh FAST_LIO_GPL
verify_repository \
  "${PCT_GPL_ROOT}" "${PCT_COMMIT}" audit_source.sh PCT_GPL

for required_path in \
  "${FAST_LIO_GPL_ROOT}/FAST_LIO/package.xml" \
  "${FAST_LIO_GPL_ROOT}/FAST_LIO/config/rs_fairy.yaml" \
  "${FAST_LIO_GPL_ROOT}/FAST_LIO_LOCALIZATION2/package.xml" \
  "${FAST_LIO_GPL_ROOT}/FAST_LIO_LOCALIZATION2/config/rs_fairy.yaml" \
  "${PCT_GPL_ROOT}/planner/build_thirdparty.sh" \
  "${PCT_GPL_ROOT}/planner/build.sh" \
  "${PCT_GPL_ROOT}/planner/run_navigation_humble.sh" \
  "${PCT_GPL_ROOT}/tomography/scripts/build_tomogram_offline.py" \
  "${PCT_GPL_ROOT}/tomography/scripts/publish_tomogram_from_pickle.py"; do
  [[ -f "${required_path}" ]] || {
    echo "Required full-navigation source file is missing: ${required_path}" >&2
    exit 1
  }
done

if [[ "${VERIFY_ONLY}" == true ]]; then
  echo "EXTERNAL_GPL_SOURCE_PREFLIGHT=PASS"
  exit 0
fi

link_component() {
  local source_path="$1"
  local destination_path="$2"
  local expected_source relative_source
  expected_source="$(realpath -e "${source_path}")"

  if [[ -L "${destination_path}" ]]; then
    [[ "$(realpath -e "${destination_path}" 2>/dev/null || true)" == "${expected_source}" ]] || {
      echo "Existing symlink points elsewhere: ${destination_path}" >&2
      return 1
    }
    return 0
  fi
  [[ ! -e "${destination_path}" ]] || {
    echo "Destination exists and is not a managed symlink: ${destination_path}" >&2
    return 1
  }
  relative_source="$(realpath --relative-to="${ROOT_DIR}" "${expected_source}")"
  ln -s "${relative_source}" "${destination_path}"
}

link_component "${FAST_LIO_GPL_ROOT}/FAST_LIO" "${ROOT_DIR}/FAST_LIO"
link_component \
  "${FAST_LIO_GPL_ROOT}/FAST_LIO_LOCALIZATION2" \
  "${ROOT_DIR}/FAST_LIO_LOCALIZATION2"
link_component "${PCT_GPL_ROOT}" "${ROOT_DIR}/PCT_planner-RC2026_Map_Planner"

echo "EXTERNAL_GPL_LAYOUT=PASS"
echo "FAST_LIO_LINK=${ROOT_DIR}/FAST_LIO"
echo "FAST_LIO_LOCALIZATION_LINK=${ROOT_DIR}/FAST_LIO_LOCALIZATION2"
echo "PCT_LINK=${ROOT_DIR}/PCT_planner-RC2026_Map_Planner"