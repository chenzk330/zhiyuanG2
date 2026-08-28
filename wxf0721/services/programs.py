#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
programs.py — 程序调试组件

职责：
  接收 /humanoid/programs/control 命令，执行 runtime 程序的运行与调试：
    - run              直接运行 main.py
    - debug            单步调试模式运行 main.py，每行执行信息发布到 /humanoid/programs/step
    - next             在调试模式下，执行下一行（手动单步）
    - stop             停止当前运行的程序
    - copy             将 programs/{data} 复制到 main.py
    - codes            读取 main.py 内容，发布到 /humanoid/programs/codes
    - read_files       读取 programs/ 下所有 .py 文件列表，发布到 /humanoid/programs/files
    - read_file        读取 programs/{data} 指定文件内容，发布到 /humanoid/programs/file_content
    - upload           上传 .py 文件到 programs/，data: {"filename": "xxx.py", "content": "..."}
    - delete           删除 programs/ 下指定 .py 文件，data: "xxx.py"

发布主题：
  - /humanoid/programs/step   : {"lineno": 3, "code": "print(1)", "filename": "main.py"}
  - /humanoid/programs/codes  : {"code": "import sys\\nprint(1)\\n"}
  - /humanoid/programs/files  : {"files": ["a.py", "b.py"]}
  - /humanoid/programs/file_content : {"filename": "a.py", "code": "...", "success": true/false}
  - /humanoid/programs/upload_result : {"success": true, "filename": "xxx.py"} 或 {"success": false, "error": "..."}
  - /humanoid/programs/delete_result : {"success": true, "filename": "xxx.py"} 或 {"success": false, "error": "..."}
"""

import os
import sys
import json
import time
import runpy
import threading

import common

# 调试时每步停顿时间（秒），让前端有时间高亮
STEP_DELAY = 0.5

# ── 全局状态 ───────────────────────────────────────────────
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
#  程序运行 / 调试
# ═══════════════════════════════════════════════════════════

def handle_run(debug=False):
    """运行 main.py

    Parameters
    ----------
    debug : bool
        True=单步调试模式，False=普通运行模式
    """
    global _running, _debugging, _stop_requested

    with _running_lock:
        if _running:
            print("[程序] 已有程序在运行，跳过")
            return
        _running = True

    _stop_requested = False

    if not os.path.exists(common.MAIN_PY):
        print(f"[程序] {common.MAIN_PY} 不存在")
        _running = False
        return

    # 确保 runtime 目录在 sys.path 中，使 main.py 能 import minth 等模块
    if common.RUNTIME_DIR not in sys.path:
        sys.path.insert(0, common.RUNTIME_DIR)

    # 读取源码用于行号映射
    with open(common.MAIN_PY, "r", encoding="utf-8") as f:
        source_lines = f.readlines()

    mode = "debug" if debug else "run"
    print(f"\n[程序] {'=' * 40}")
    print(f"[程序] 开始执行 main.py ({len(source_lines)} 行, mode={mode})")

    if debug:
        _debugging = True
        _step_event.clear()  # 确保第一步等待用户点击「下一行」
        _run_with_trace(source_lines)
        _debugging = False
    else:
        _run_normal(source_lines)

    print(f"[程序] 执行完成 (mode={mode})")


def _run_normal(source_lines):
    """普通运行模式：安装只检查停止请求的轻量 trace，同时发布步骤信息"""
    global _running, _stop_requested

    def stop_trace(frame, event, arg):
        if event == 'line':
            if _stop_requested:
                raise _StopExecution()
            filename = frame.f_code.co_filename
            # 只跟踪 main.py，发布步骤信息
            if common.MAIN_PY in filename or filename.endswith("main.py"):
                lineno = frame.f_lineno
                code = source_lines[lineno - 1].rstrip() if 0 < lineno <= len(source_lines) else ""
                step_msg = {
                    "filename": "main.py",
                    "lineno": lineno,
                    "code": code,
                }
                common.publish(common.TOPIC_PROGRAMS_STEP, step_msg, qos=0)
                print(f"  [step] L{lineno}: {code}")
        return stop_trace

    old_trace = sys.gettrace()
    sys.settrace(stop_trace)
    try:
        runpy.run_path(common.MAIN_PY, run_name="__main__")
    except _StopExecution:
        print("[程序] 用户请求停止，已中断执行")
    except Exception as e:
        print(f"[程序] 错误: {e}")
    finally:
        sys.settrace(old_trace)
        with _running_lock:
            _running = False


def _run_with_trace(source_lines):
    """带单步跟踪的执行（手动单步：等待 next 命令）"""
    global _running, _stop_requested

    def trace_lines(frame, event, arg):
        if event == 'line':
            if _stop_requested:
                raise _StopExecution()
            filename = frame.f_code.co_filename
            if common.MAIN_PY in filename or filename.endswith("main.py"):
                lineno = frame.f_lineno
                code = source_lines[lineno - 1].rstrip() if 0 < lineno <= len(source_lines) else ""
                step_msg = {
                    "filename": "main.py",
                    "lineno": lineno,
                    "code": code,
                }
                common.publish(common.TOPIC_PROGRAMS_STEP, step_msg, qos=0)
                print(f"  [step] L{lineno}: {code}  (等待 next...)")
                # 等待用户发送 next 命令才继续执行下一行
                _step_event.wait()
                _step_event.clear()
                if _stop_requested:
                    raise _StopExecution()
        return trace_lines

    def trace_calls(frame, event, arg):
        if event == 'call':
            filename = frame.f_code.co_filename
            if common.MAIN_PY in filename or filename.endswith("main.py"):
                return trace_lines
        return None

    old_trace = sys.gettrace()
    sys.settrace(trace_calls)
    try:
        runpy.run_path(common.MAIN_PY, run_name="__main__")
    except _StopExecution:
        print("[程序] 用户请求停止，已中断执行")
    except Exception as e:
        print(f"[程序] 调试错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        sys.settrace(old_trace)
        _step_event.set()  # 释放可能还在等待的 trace
        with _running_lock:
            _running = False


def handle_next():
    """单步调试：执行下一行"""
    if not _debugging:
        print("[程序] 当前不在调试模式，忽略 next")
        return
    print("[程序] 继续执行下一行")
    _step_event.set()


def handle_stop():
    """停止程序执行"""
    global _stop_requested
    if not _running:
        print("[程序] 当前没有程序在运行，忽略 stop")
        return
    _stop_requested = True
    _step_event.set()  # 唤醒等待中的 trace
    print("[程序] 已请求停止程序")


# ═══════════════════════════════════════════════════════════
#  程序文件管理
# ═══════════════════════════════════════════════════════════

def handle_copy(data):
    """将 programs/{data} 复制到 main.py

    Parameters
    ----------
    data : str
        源文件名，如 "a.py"
    """
    src_name = data if data else "a.py"
    src_path = os.path.join(common.PROGRAMS_DIR, src_name)

    if not os.path.exists(src_path):
        print(f"[程序] 源文件不存在: {src_path}")
        return

    with open(src_path, "r", encoding="utf-8") as f:
        content = f.read()

    with open(common.MAIN_PY, "w", encoding="utf-8") as f:
        f.write(content)

    print(f"[程序] {src_name} → main.py")


def handle_read_program_files():
    """读取 programs/ 目录下所有 .py 文件，发布文件列表到 /humanoid/programs/files"""
    if not os.path.exists(common.PROGRAMS_DIR):
        print(f"[程序] programs 目录不存在: {common.PROGRAMS_DIR}")
        return

    files = []
    for name in sorted(os.listdir(common.PROGRAMS_DIR)):
        if name.endswith('.py') and name != '__init__.py':
            files.append(name)

    msg = {"files": files}
    common.publish(common.TOPIC_PROGRAMS_FILES, msg, qos=0)
    print(f"[程序] 已发布 {len(files)} 个文件: {files}")


def handle_upload(data):
    """上传 .py 文件到 programs/ 目录

    Parameters
    ----------
    data : dict
        {"filename": "xxx.py", "content": "文件内容字符串"}
    """
    if not data or not isinstance(data, dict):
        _publish_upload_result(False, error="上传数据格式错误")
        return

    filename = data.get("filename", "").strip()
    content = data.get("content", "")

    if not filename:
        _publish_upload_result(False, error="文件名为空")
        return

    if not filename.endswith('.py'):
        _publish_upload_result(False, error=f"只支持 .py 文件: {filename}")
        return

    if '/' in filename or '\\' in filename or '..' in filename:
        _publish_upload_result(False, error=f"非法文件名: {filename}")
        return

    if not os.path.exists(common.PROGRAMS_DIR):
        os.makedirs(common.PROGRAMS_DIR, exist_ok=True)

    target_path = os.path.join(common.PROGRAMS_DIR, filename)
    try:
        with open(target_path, "w", encoding="utf-8") as f:
            f.write(content)
        print(f"[程序] 上传成功: {filename} ({len(content)} 字符)")
        _publish_upload_result(True, filename=filename)
        handle_read_program_files()
    except Exception as e:
        print(f"[程序] 上传失败: {e}")
        _publish_upload_result(False, error=str(e))


def _publish_upload_result(success, filename=None, error=None):
    """发布上传结果"""
    msg = {"success": success}
    if filename:
        msg["filename"] = filename
    if error:
        msg["error"] = error
    common.publish(common.TOPIC_PROGRAMS_UPLOAD_RESULT, msg, qos=0)


def handle_read_file(filename):
    """读取 programs/ 下指定 .py 文件内容，发布到 /humanoid/programs/file_content

    Parameters
    ----------
    filename : str
        文件名，如 "a.py"
    """
    if not filename or not isinstance(filename, str):
        _publish_file_content(filename or "", "", success=False, error="文件名为空")
        return

    if '/' in filename or '\\' in filename or '..' in filename:
        _publish_file_content(filename, "", success=False, error=f"非法文件名: {filename}")
        return

    if not filename.endswith('.py'):
        _publish_file_content(filename, "", success=False, error=f"只支持 .py 文件: {filename}")
        return

    filepath = os.path.join(common.PROGRAMS_DIR, filename)
    if not os.path.exists(filepath):
        _publish_file_content(filename, "", success=False, error=f"文件不存在: {filename}")
        return

    try:
        with open(filepath, "r", encoding="utf-8") as f:
            code = f.read()
        _publish_file_content(filename, code, success=True)
        print(f"[程序] 已读取文件: {filename} ({len(code)} 字符)")
    except Exception as e:
        _publish_file_content(filename, "", success=False, error=str(e))


def _publish_file_content(filename, code, success=True, error=None):
    """发布文件内容"""
    msg = {"filename": filename, "code": code, "success": success}
    if error:
        msg["error"] = error
    common.publish(common.TOPIC_PROGRAMS_FILE_CONTENT, msg, qos=0)


def handle_delete(filename):
    """删除 programs/ 下指定 .py 文件

    Parameters
    ----------
    filename : str
        要删除的文件名，如 "a.py"
    """
    if not filename or not isinstance(filename, str):
        _publish_delete_result(False, filename=filename or "", error="文件名为空")
        return

    if '/' in filename or '\\' in filename or '..' in filename:
        _publish_delete_result(False, filename=filename, error=f"非法文件名: {filename}")
        return

    if filename == "main.py":
        _publish_delete_result(False, filename=filename, error="不能删除 main.py")
        return

    filepath = os.path.join(common.PROGRAMS_DIR, filename)
    if not os.path.exists(filepath):
        _publish_delete_result(False, filename=filename, error=f"文件不存在: {filename}")
        return

    try:
        os.remove(filepath)
        print(f"[程序] 已删除文件: {filename}")
        _publish_delete_result(True, filename=filename)
        handle_read_program_files()
    except Exception as e:
        print(f"[程序] 删除失败: {e}")
        _publish_delete_result(False, filename=filename, error=str(e))


def _publish_delete_result(success, filename=None, error=None):
    """发布删除结果"""
    msg = {"success": success}
    if filename:
        msg["filename"] = filename
    if error:
        msg["error"] = error
    common.publish(common.TOPIC_PROGRAMS_DELETE_RESULT, msg, qos=0)


def handle_codes():
    """读取 main.py 并发布到 /humanoid/programs/codes"""
    if not os.path.exists(common.MAIN_PY):
        print(f"[程序] {common.MAIN_PY} 不存在")
        return

    with open(common.MAIN_PY, "r", encoding="utf-8") as f:
        code = f.read()

    msg = {"code": code}
    common.publish(common.TOPIC_PROGRAMS_CODES, msg, qos=0)
    print(f"[程序] 已发布 main.py ({len(code)} 字符)")


# ═══════════════════════════════════════════════════════════
#  命令处理
# ═══════════════════════════════════════════════════════════

def handle_control(payload):
    """处理 /humanoid/programs/control 命令

    Parameters
    ----------
    payload : dict
        命令消息，如 {"command": "run"}
    """
    cmd = payload.get("command")
    data = payload.get("data")

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
    elif cmd == "read_files":
        handle_read_program_files()
    elif cmd == "upload":
        handle_upload(data)
    elif cmd == "read_file":
        handle_read_file(data)
    elif cmd == "delete":
        handle_delete(data)
    else:
        print(f"[程序] 未知命令: {cmd}")
