#!/usr/bin/env bash
# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

case "${1:-}" in
  nx)
    shift
    exec bash "${ROOT_DIR}/deployment/install_permissive_source_nx.sh" "$@"
    ;;
  full-nx)
    shift
    exec bash "${ROOT_DIR}/deployment/install_full_navigation_nx.sh" "$@"
    ;;
  -h|--help|help|"")
    cat <<'EOF'
用法：
  ./install.sh nx [--bootstrap] [--jobs N] [--with-tests] [--preflight-only]
  ./install.sh full-nx [--bootstrap] [--jobs N] [--with-tests] [--preflight-only]

nx 仅构建本仓库包含的 RoboSense 与 SCAN 宽松许可证源码。
full-nx 自动拉取缺失的固定版本 GPL 仓库，然后验证并构建三个仓库。
已有 GPL 仓库绝不会被自动更新或切换。
两种模式都不会启动导航、连接底盘或发送速度命令。
EOF
    ;;
  *)
    echo "仅支持 nx 或 full-nx 模式；未知参数: $1" >&2
    exit 2
    ;;
esac
