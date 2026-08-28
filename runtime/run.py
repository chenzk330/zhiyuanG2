#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 使用 runtime 虚拟环境运行: ./runtime/bin/python run.py
"""
Runtime 单步调试服务

监听 /runtime_debug 接口，执行以下命令：
  - {cmd: "run"}                : 直接运行 main.py
  - {cmd: "debug"}              : 单步调试模式运行 main.py，每行执行信息发布到 /runtime_step
  - {cmd: "next"}               : 在调试模式下，执行下一行（手动单步）
  - {cmd: "stop"}               : 停止当前运行的程序（调试或普通运行）
  - {cmd: "copy", data: "a.py"} : 将 programs/{data} 复制到 main.py，并在第一行加入调试库 import
  - {cmd: "codes"}              : 读取 main.py 内容，发布到 /runtime_codes
  - {cmd: "read_program_files"} : 读取 programs/ 下所有 .py 文件列表，发布到 /runtime_program_files

发布 topic：
  - /runtime_step          : {"lineno": 3, "code": "print(1)", "filename": "main.py"}
  - /runtime_codes         : {"code": "import sys\\nprint(1)\\n"}
  - /runtime_program_files : {"files": ["a.py", "b.py"]}
"""

import os
import sys
import json
import time
import runpy
import threading

import paho.mqtt.client as mqtt

# ── 路径配置 ───────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROGRAMS_DIR = os.path.join(SCRIPT_DIR, "programs")
MAIN_PY = os.path.join(SCRIPT_DIR, "main.py")

# ── MQTT 配置 ─────────────────────────────────────────────
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_CLIENT_ID = "runtime_debug_service"

TOPIC_DEBUG = "/runtime_debug"              # 接收命令
TOPIC_STEP = "/runtime_step"                # 发布执行步骤
TOPIC_CODES = "/runtime_codes"              # 发布代码内容
TOPIC_PROGRAM_FILES = "/runtime_program_files"  # 发布程序文件列表

# 调试时每步停顿时间（秒），让前端有时间高亮
STEP_DELAY = 0.5

# ── 全局 ───────────────────────────────────────────────────
mqtt_client = None
_running = False
_running_lock = threading.Lock()
# 单步调试事件：trace 函数等待该事件被 set 后才继续执行下一行
_step_event = threading.Event()
_debugging = False
# 停止请求标志：用户点击「停止程序」后置为 True，trace 检测到后抛异常中断执行
_stop_requested = False


class _StopExecution(Exception):
    """用于中断 main.py 执行的内部异常"""
    pass


# ═══════════════════════════════════════════════════════════
#  命令处理
# ═══════════════════════════════════════════════════════════

def handle_run(debug=False):
    """运行 main.py"""
    global _running, _debugging, _stop_requested

    with _running_lock:
        if _running:
            print("[运行] 已有程序在运行，跳过")
            return
        _running = True

    _stop_requested = False

    if not os.path.exists(MAIN_PY):
        print(f"[错误] {MAIN_PY} 不存在")
        _running = False
        return

    # 确保 SCRIPT_DIR 在 sys.path 中，使 main.py 能 import minth 等模块
    if SCRIPT_DIR not in sys.path:
        sys.path.insert(0, SCRIPT_DIR)

    # 读取源码用于行号映射
    with open(MAIN_PY, "r", encoding="utf-8") as f:
        source_lines = f.readlines()

    mode = "debug" if debug else "run"
    print(f"\n{'=' * 50}")
    print(f"[{mode}] 开始执行 main.py ({len(source_lines)} 行)")
    print(f"{'=' * 50}")

    if debug:
        _debugging = True
        _step_event.clear()  # 确保第一步等待用户点击「下一行」
        _run_with_trace(source_lines)
        _debugging = False
    else:
        # 非调试模式：安装只检查停止请求的轻量 trace，同时发布步骤信息
        def stop_trace(frame, event, arg):
            if event == 'line':
                if _stop_requested:
                    raise _StopExecution()
                filename = frame.f_code.co_filename
                # 只跟踪 main.py，发布步骤信息
                if MAIN_PY in filename or filename.endswith("main.py"):
                    lineno = frame.f_lineno
                    code = source_lines[lineno - 1].rstrip() if 0 < lineno <= len(source_lines) else ""
                    step_msg = json.dumps({
                        "filename": "main.py",
                        "lineno": lineno,
                        "code": code
                    }, ensure_ascii=False)
                    if mqtt_client:
                        mqtt_client.publish(TOPIC_STEP, step_msg, qos=0)
                    print(f"  [step] L{lineno}: {code}")
            return stop_trace
        old_trace = sys.gettrace()
        sys.settrace(stop_trace)
        try:
            runpy.run_path(MAIN_PY, run_name="__main__")
        except _StopExecution:
            print("[run] 用户请求停止，已中断执行")
        except Exception as e:
            print(f"[错误] {e}")
        finally:
            sys.settrace(old_trace)
            with _running_lock:
                _running = False

    print(f"[{mode}] 执行完成")


def _run_with_trace(source_lines):
    """带单步跟踪的执行（手动单步：等待 /runtime_debug 的 next 命令）"""
    global _running, _stop_requested

    def trace_lines(frame, event, arg):
        if event == 'line':
            # 检查停止请求
            if _stop_requested:
                raise _StopExecution()
            filename = frame.f_code.co_filename
            # 只跟踪 main.py
            if MAIN_PY in filename or filename.endswith("main.py"):
                lineno = frame.f_lineno
                code = source_lines[lineno - 1].rstrip() if 0 < lineno <= len(source_lines) else ""
                # 发布步骤信息
                step_msg = json.dumps({
                    "filename": "main.py",
                    "lineno": lineno,
                    "code": code
                }, ensure_ascii=False)
                if mqtt_client:
                    mqtt_client.publish(TOPIC_STEP, step_msg, qos=0)
                print(f"  [step] L{lineno}: {code}  (等待 next...)")
                # 等待用户发送 next 命令才继续执行下一行
                _step_event.wait()
                _step_event.clear()
                # 被唤醒后再次检查停止请求（可能是 stop 触发的唤醒）
                if _stop_requested:
                    raise _StopExecution()
        return trace_lines

    def trace_calls(frame, event, arg):
        if event == 'call':
            filename = frame.f_code.co_filename
            if MAIN_PY in filename or filename.endswith("main.py"):
                return trace_lines
        return None

    old_trace = sys.gettrace()
    sys.settrace(trace_calls)
    try:
        runpy.run_path(MAIN_PY, run_name="__main__")
    except _StopExecution:
        print("[调试] 用户请求停止，已中断执行")
    except Exception as e:
        print(f"[调试错误] {e}")
        import traceback
        traceback.print_exc()
    finally:
        sys.settrace(old_trace)
        # 程序结束，释放可能还在等待的 trace
        _step_event.set()
        with _running_lock:
            _running = False


def handle_next():
    """单步调试：执行下一行"""
    if not _debugging:
        print("[next] 当前不在调试模式，忽略")
        return
    print("[next] 继续执行下一行")
    _step_event.set()


def handle_stop():
    """停止程序执行"""
    global _stop_requested
    if not _running:
        print("[stop] 当前没有程序在运行，忽略")
        return
    _stop_requested = True
    # 如果正在等待 next，先唤醒让它能检查停止标志
    _step_event.set()
    print("[stop] 已请求停止程序")


def handle_copy(data):
    """将 programs/{data} 复制到 main.py，第一行加入调试库 import"""
    src_name = data if data else "a.py"
    src_path = os.path.join(PROGRAMS_DIR, src_name)

    if not os.path.exists(src_path):
        print(f"[错误] 源文件不存在: {src_path}")
        return

    with open(src_path, "r", encoding="utf-8") as f:
        content = f.read()

    # 在第一行引入 Python 调试库（pdb），实现单步调试能力
    debug_header = "import pdb\n"

    with open(MAIN_PY, "w", encoding="utf-8") as f:
        f.write(debug_header + content)

    print(f"[复制] {src_name} → main.py (已加入 import pdb)")


def handle_read_program_files():
    """读取 programs/ 目录下所有 .py 文件，发布文件列表到 /runtime_program_files"""
    if not os.path.exists(PROGRAMS_DIR):
        print(f"[错误] programs 目录不存在: {PROGRAMS_DIR}")
        return

    files = []
    for name in sorted(os.listdir(PROGRAMS_DIR)):
        if name.endswith('.py') and name != '__init__.py':
            files.append(name)

    msg = json.dumps({"files": files}, ensure_ascii=False)
    if mqtt_client:
        mqtt_client.publish(TOPIC_PROGRAM_FILES, msg, qos=0)
    print(f"[文件列表] 已发布 {len(files)} 个文件: {files}")


def handle_codes():
    """读取 main.py 并发布到 /runtime_codes"""
    if not os.path.exists(MAIN_PY):
        print(f"[错误] {MAIN_PY} 不存在")
        return

    with open(MAIN_PY, "r", encoding="utf-8") as f:
        code = f.read()

    msg = json.dumps({"code": code}, ensure_ascii=False)
    if mqtt_client:
        mqtt_client.publish(TOPIC_CODES, msg, qos=0)
    print(f"[代码] 已发布 main.py ({len(code)} 字符)")


# ═══════════════════════════════════════════════════════════
#  MQTT
# ═══════════════════════════════════════════════════════════

def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print(f"[MQTT] 已连接到 {MQTT_BROKER}:{MQTT_PORT}")
        client.subscribe(TOPIC_DEBUG, qos=0)
        print(f"[MQTT] 已订阅: {TOPIC_DEBUG}")
        print("-" * 50)
    else:
        print(f"[MQTT] 连接失败，返回码: {rc}")


def on_disconnect(client, userdata, rc, properties=None):
    if rc != 0:
        print(f"[MQTT] 意外断开 (rc={rc})，可能是 client_id 冲突，将自动重连...")
    else:
        print("[MQTT] 正常断开")


def on_message(client, userdata, msg):
    """收到 MQTT 消息时分发命令"""
    try:
        payload = msg.payload.decode("utf-8")
        cmd_msg = json.loads(payload)
        cmd = cmd_msg.get("cmd")
        data = cmd_msg.get("data")
    except Exception as e:
        print(f"[解析失败] {e}")
        return

    print(f"\n[收到] cmd={cmd}, data={data}")

    if cmd == "run":
        # 在新线程中运行，避免阻塞 MQTT loop
        t = threading.Thread(target=handle_run, args=(False,), daemon=True)
        t.start()
    elif cmd == "debug":
        t = threading.Thread(target=handle_run, args=(True,), daemon=True)
        t.start()
    elif cmd == "next":
        handle_next()
    elif cmd == "stop":
        handle_stop()
    elif cmd == "copy":
        handle_copy(data)
    elif cmd == "codes":
        handle_codes()
    elif cmd == "read_program_files":
        handle_read_program_files()
    else:
        print(f"[未知命令] {cmd}")


# ═══════════════════════════════════════════════════════════
#  主入口
# ═══════════════════════════════════════════════════════════

def main():
    global mqtt_client

    print("#" * 50)
    print("#   Runtime 调试服务 - 启动   #")
    print("#" * 50)
    print(f"programs 目录: {PROGRAMS_DIR}")
    print(f"main.py      : {MAIN_PY}")
    print(f"调试 topic   : {TOPIC_DEBUG}")
    print(f"步骤 topic   : {TOPIC_STEP}")
    print(f"代码 topic   : {TOPIC_CODES}")
    print(f"文件列表 topic: {TOPIC_PROGRAM_FILES}")
    print()

    # 确保目录存在
    os.makedirs(PROGRAMS_DIR, exist_ok=True)

    mqtt_client = mqtt.Client(
        client_id=MQTT_CLIENT_ID,
        callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
    )
    mqtt_client.on_connect = on_connect
    mqtt_client.on_disconnect = on_disconnect
    mqtt_client.on_message = on_message
    # 设置重连延迟：首次 2 秒，最大 30 秒
    mqtt_client.reconnect_delay_set(min_delay=2, max_delay=30)

    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
        print(f"[MQTT] 正在连接 {MQTT_BROKER}:{MQTT_PORT} ...")
        mqtt_client.loop_forever()
    except KeyboardInterrupt:
        print("\n[退出] 用户中断")
    except Exception as e:
        print(f"[错误] {e}")
    finally:
        try:
            mqtt_client.disconnect()
        except Exception:
            pass
        print("程序结束")


if __name__ == "__main__":
    main()
