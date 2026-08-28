#!/bin/bash
# ─────────────────────────────────────────────────────────
#  Humanoid 机器人控制服务 - 启动脚本
#
#  功能：
#    1. source /home/agi/app/env.sh 加载 GDK 环境变量
#    2. 后台启动单一服务入口 services/main.py
#    3. 同时启动 web 静态服务器（端口 8002）
#
#  架构说明：
#    旧架构需启动 5 个独立服务（app/camera/data/status/runtime），
#    新架构合并为 1 个 main.py 入口，内部按组件模块分工：
#      - common.py     共享基础设施（GDK / MQTT）
#      - camera.py     相机数据发布与控制
#      - joints.py     关节运动控制与数据持久化
#      - status.py     机器人状态与点云发布
#      - commands.py   动作命令处理（tts/grab/go 等）
#      - map.py        地图点位管理
#      - programs.py   runtime 程序调试
#
#  日志：
#    logs/humanoid_server.log
#    logs/web_server.log
#
#  用法：
#    chmod +x run.sh
#    ./run.sh           # 启动
#    ./run.sh stop       # 停止
#    ./run.sh status     # 查看状态
#    ./run.sh restart    # 重启
# ─────────────────────────────────────────────────────────

# 切换到脚本所在目录（services/）
RUN_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 项目根目录（services 的上一级）
PROJECT_DIR="$(cd "$RUN_DIR/.." && pwd)"
cd "$RUN_DIR"

# 加载 GDK 环境变量
# 注意：env.sh 通过 $1 指定 GDK 安装根目录（其下需有 lib/ bin/ gdk/lib 等）。
# 必须显式传入，否则会继承 run.sh 自身的位置参数（如 "start"），导致 find "start/lib" 失败。
ENV_FILE="/home/agi/app/env.sh"
ENV_DIR="$(dirname "$ENV_FILE")"   # /home/agi/app
if [ -f "$ENV_FILE" ]; then
    echo "[run.sh] 正在加载环境变量: source $ENV_FILE $ENV_DIR"
    source "$ENV_FILE" "$ENV_DIR"
    cd "$RUN_DIR"
else
    echo "[run.sh] ⚠ 警告: 环境文件不存在: $ENV_FILE"
fi

# 服务列表：格式 "服务名|启动方式"
#   humanoid_server : 启动 services/main.py（核心服务）
#   web_server      : 在 web/ 目录启动 http 服务器
SERVICES=(
    "humanoid_server|$RUN_DIR/main.py"
    "web_server|CMD:python3 -m http.server 8002|$PROJECT_DIR/web"
)

# 日志目录
LOG_DIR="$RUN_DIR/logs"
mkdir -p "$LOG_DIR"

# ── 启动 ──────────────────────────────────────────────────
start_service() {
    local entry="$1"
    local name="${entry%%|*}"
    local rest="${entry#*|}"
    local log_file="$LOG_DIR/${name}.log"
    local pid_file="$LOG_DIR/${name}.pid"
    local pid

    # 检查是否已在运行
    if [ -f "$pid_file" ]; then
        local old_pid=$(cat "$pid_file")
        if kill -0 "$old_pid" 2>/dev/null; then
            echo "[run.sh] ⚠ $name 已在运行 (PID=$old_pid)，跳过"
            return 0
        else
            rm -f "$pid_file"
        fi
    fi

    if [[ "$rest" == CMD:* ]]; then
        # 命令形式：CMD:command|relative_cwd
        local cmd="${rest#CMD:}"
        local cwd_rel="."
        if [[ "$cmd" == *"|"* ]]; then
            cwd_rel="${cmd##*|}"
            cmd="${cmd%|*}"
        fi
        local work_dir="$cwd_rel"
        if [ ! -d "$work_dir" ]; then
            echo "[run.sh] ❌ 工作目录不存在: $work_dir"
            return 1
        fi
        (cd "$work_dir" && nohup bash -c "$cmd" > "$log_file" 2>&1 & echo $! > "$pid_file")
    else
        # 文件形式：直接启动 python3
        local py_file="$rest"
        if [ ! -f "$py_file" ]; then
            echo "[run.sh] ❌ 文件不存在: $py_file"
            return 1
        fi
        nohup python3 "$py_file" > "$log_file" 2>&1 &
        echo "$!" > "$pid_file"
    fi

    pid=$(cat "$pid_file")
    sleep 1
    if kill -0 "$pid" 2>/dev/null; then
        echo "[run.sh] ✅ $name 已启动 (PID=$pid, 日志=$log_file)"
    else
        echo "[run.sh] ❌ $name 启动失败，请查看日志: $log_file"
        rm -f "$pid_file"
        return 1
    fi
}

# ── 停止 ──────────────────────────────────────────────────
stop_service() {
    local name="$1"
    local pid_file="$LOG_DIR/${name}.pid"

    if [ ! -f "$pid_file" ]; then
        echo "[run.sh] ⚠ $name 未在运行 (无 PID 文件)"
        return 0
    fi

    local pid=$(cat "$pid_file")
    if kill -0 "$pid" 2>/dev/null; then
        kill "$pid"
        sleep 1
        if kill -0 "$pid" 2>/dev/null; then
            echo "[run.sh] 强制终止 $name (PID=$pid)"
            kill -9 "$pid"
        fi
        echo "[run.sh] ✅ $name 已停止 (PID=$pid)"
    else
        echo "[run.sh] ⚠ $name 进程不存在 (PID=$pid 可能已退出)"
    fi
    rm -f "$pid_file"
}

# ── 状态 ──────────────────────────────────────────────────
status_service() {
    local name="$1"
    local pid_file="$LOG_DIR/${name}.pid"

    if [ -f "$pid_file" ]; then
        local pid=$(cat "$pid_file")
        if kill -0 "$pid" 2>/dev/null; then
            echo "[run.sh] ● $name 运行中 (PID=$pid)"
        else
            echo "[run.sh] ○ $name 已停止 (PID=$pid 已退出)"
        fi
    else
        echo "[run.sh] ○ $name 未启动"
    fi
}

# ── 命令分发 ──────────────────────────────────────────────
case "${1:-start}" in
    start)
        echo "════════════════════════════════════════════════════"
        echo "  Humanoid 机器人控制服务启动  $(date '+%Y-%m-%d %H:%M:%S')"
        echo "════════════════════════════════════════════════════"
        for svc in "${SERVICES[@]}"; do
            start_service "$svc"
        done
        echo "────────────────────────────────────────────────────"
        echo "  日志目录: $LOG_DIR"
        echo "  查看状态: ./run.sh status"
        echo "  停止服务: ./run.sh stop"
        echo "════════════════════════════════════════════════════"
        ;;

    stop)
        echo "正在停止所有服务..."
        for svc in "${SERVICES[@]}"; do
            stop_service "${svc%%|*}"
        done
        echo "停止完成。"
        ;;

    status)
        echo "════════════════════════════════════════════════════"
        echo "  Humanoid 服务状态  $(date '+%Y-%m-%d %H:%M:%S')"
        echo "════════════════════════════════════════════════════"
        for svc in "${SERVICES[@]}"; do
            status_service "${svc%%|*}"
        done
        ;;

    restart)
        "$0" stop
        sleep 2
        "$0" start
        ;;

    *)
        echo "用法: $0 {start|stop|status|restart}"
        exit 1
        ;;
esac
