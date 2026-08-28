#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 默认不传 --enable-control，只启动状态/相机服务并对 set 返回 ACK。
python3 G2_groot_adapter.py "$@"
