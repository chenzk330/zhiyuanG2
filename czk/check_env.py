#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_env.py — 环境前置检查

在运行纠偏/标定前, 检测所有外部依赖是否就绪:
  1. GDK 环境变量是否加载
  2. humanoid_server 是否在 MQTT 1883 端口监听
  3. 辉羲 batch_inference 工具是否可执行
  4. 模型文件是否齐全
  5. 相机图片保存目录是否可写

注: SLAM 重定位状态无法通过 minth API 直接查询 (无对应方法).
    若导航/拍照异常, 请通过 HMI 确认 SLAM 已完成全局定位.
"""
import os
import sys
import socket

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)


# ═══════════════════════════════════════════════════════════
#  检查项
# ═══════════════════════════════════════════════════════════

def check_gdk_env():
    """检查 GDK 环境是否加载

    GDK 的 .so 库通过 LD_LIBRARY_PATH 加载 (分布在 /home/agi/app/lib/* 子目录),
    Python 模块 agibot_gdk 通过 PYTHONPATH 的 /home/agi/app/gdk/lib 加载.
    两者都需检查.
    """
    ld_path = os.environ.get("LD_LIBRARY_PATH", "")
    py_path = os.environ.get("PYTHONPATH", "")

    ld_ok = "/home/agi/app/lib" in ld_path
    py_ok = "gdk/lib" in py_path

    if ld_ok and py_ok:
        return True, "GDK 环境已加载 (LD_LIBRARY_PATH + PYTHONPATH)"
    missing = []
    if not ld_ok:
        missing.append("LD_LIBRARY_PATH 缺少 /home/agi/app/lib")
    if not py_ok:
        missing.append("PYTHONPATH 缺少 gdk/lib")
    return False, f"GDK 环境未完全加载 ({'; '.join(missing)}). " \
                  f"请先运行: source /home/agi/app/env.sh /home/agi/app"


def check_mqtt_broker(broker="localhost", port=1883):
    """检查 MQTT broker 是否监听 (mosquitto, 消息中转站)"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2.0)
        s.connect((broker, port))
        s.close()
        return True, f"MQTT broker {broker}:{port} 已监听"
    except Exception as e:
        return False, f"MQTT broker {broker}:{port} 未监听: {e}. " \
                      f"请确认 mosquitto 已启动 (通常由系统服务管理)"


def check_humanoid_server():
    """检查 humanoid_server 进程是否运行

    humanoid_server 是服务端总控 (负责相机/关节/状态的 MQTT 命令订阅).
    没有它, 即使 MQTT broker 正常, 相机命令也没人处理.
    """
    try:
        import subprocess
        result = subprocess.run(
            ["ps", "aux"], capture_output=True, text=True, timeout=5
        )
        # 匹配运行 services/main.py 或 run.sh 启动的 python 进程
        lines = [l for l in result.stdout.splitlines()
                 if ("main.py" in l and "services" in l)
                 or ("humanoid" in l.lower() and "python" in l.lower())]
        if lines:
            return True, f"humanoid_server 运行中 ({len(lines)} 个进程)"
        return False, "humanoid_server 未运行. " \
                      "请启动: cd /data/wxf/wxf0721/services && bash run.sh"
    except Exception as e:
        return False, f"无法检查 humanoid_server 进程: {e}"


def check_batch_infer(bin_path="/home/agi/app/bin/examples/batch_inference"):
    """检查辉羲推理工具是否可执行"""
    if os.path.isfile(bin_path) and os.access(bin_path, os.X_OK):
        return True, f"辉羲推理工具可执行: {bin_path}"
    return False, f"辉羲推理工具不存在或不可执行: {bin_path}"


def check_models():
    """检查模型文件是否齐全"""
    models_dir = os.path.join(_HERE, "models")
    required = ["shangliaoqu.ref", "best_new.ref", "best_new.pt", "jitai_new.ref", "jitai.pt"]
    missing = []
    for name in required:
        path = os.path.join(models_dir, name)
        if not os.path.isfile(path):
            missing.append(name)
    if missing:
        return False, f"模型文件缺失: {missing}. 请确认模型已放入 models/ 目录"
    return True, f"模型文件齐全 ({len(required)} 个)"


def check_image_save_dir(path="/data/wxf/wxf0721/images"):
    """检查相机图片保存目录"""
    if os.path.isdir(path) and os.access(path, os.W_OK):
        return True, f"图片保存目录可写: {path}"
    return False, f"图片保存目录不存在或不可写: {path}"


def check_python_imports():
    """检查关键 Python 模块是否可导入 (cv2, numpy, yaml, paho.mqtt)"""
    missing = []
    for mod in ("cv2", "numpy", "yaml"):
        try:
            __import__(mod)
        except ImportError:
            missing.append(mod)
    # paho.mqtt 独立检查 (夹爪控制需要)
    try:
        import paho.mqtt.client  # noqa: F401
    except ImportError:
        missing.append("paho.mqtt")
    if not missing:
        return True, "关键 Python 模块就绪 (cv2, numpy, yaml, paho.mqtt)"
    return False, f"缺少 Python 模块: {missing}. 请安装: pip install {' '.join(m if m != 'paho.mqtt' else 'paho-mqtt' for m in missing)}"


def check_wzd_arm_scripts(wzd_dir=_HERE):
    """检查 wzd/execute/ 下的手臂控制脚本是否齐全 (full_pipeline 需要)"""
    exec_dir = os.path.join(wzd_dir, "execute")
    required = [
        "move_arms_to_standby.py",
        "move_arms_to_pre_pick.py",
        "move_arms_to_pick.py",
        "move_arms_to_lift.py",
        "move_arms_to_pre_place.py",
        "move_arms_to_place.py",
        "move_arms_to_pose.py",
    ]
    missing = []
    for name in required:
        path = os.path.join(exec_dir, name)
        if not os.path.isfile(path):
            missing.append(name)
    if not missing:
        return True, f"手臂控制脚本齐全 ({len(required)} 个, {exec_dir})"
    return False, f"手臂控制脚本缺失: {missing}. 请检查 {exec_dir}"


def check_wzd_pose_jsons(wzd_dir=_HERE):
    """检查 wzd/position/ 下的位姿 JSON 是否齐全 (full_pipeline 需要)"""
    pose_dir = os.path.join(wzd_dir, "position")
    required = [
        "pose_initial.json",
        "pose_standby.json",
        "pose_pre_pick.json",
        "pose_pick.json",
        "pose_lift.json",
        "pose_pre_place.json",
        "pose_place.json",
    ]
    missing = []
    for name in required:
        path = os.path.join(pose_dir, name)
        if not os.path.isfile(path):
            missing.append(name)
    if not missing:
        return True, f"位姿 JSON 齐全 ({len(required)} 个, {pose_dir})"
    return False, f"位姿 JSON 缺失: {missing}. 请通过 function/get_current_pose.py 录制或检查 {pose_dir}"


# ═══════════════════════════════════════════════════════════
#  主检查流程
# ═══════════════════════════════════════════════════════════

def run_all_checks(skip_slam=False):
    """运行所有环境检查

    Args:
        skip_slam: 兼容参数 (保留, SLAM 状态无法通过 API 检查, 请通过 HMI 确认)

    Returns:
        bool: 所有检查是否通过
    """
    print(f"\n{'=' * 60}")
    print(f"环境前置检查")
    print(f"{'=' * 60}\n")

    checks = [
        ("GDK 环境", check_gdk_env),
        ("MQTT broker", check_mqtt_broker),
        ("humanoid_server", check_humanoid_server),
        ("辉羲推理工具", check_batch_infer),
        ("Python 模块", check_python_imports),
        ("模型文件", check_models),
        ("图片保存目录", check_image_save_dir),
        ("手臂控制脚本", check_wzd_arm_scripts),
        ("位姿 JSON 文件", check_wzd_pose_jsons),
    ]

    all_ok = True
    for name, check_fn in checks:
        try:
            ok, msg = check_fn()
        except Exception as e:
            ok, msg = False, f"检查异常: {e}"
        status = "✓" if ok else "✗"
        print(f"  [{status}] {name}: {msg}")
        if not ok:
            all_ok = False

    # SLAM 提示信息
    print(f"\n  [!] SLAM 定位: 请通过 HMI 确认 SLAM 已完成全局定位")
    print(f"       (minth API 无法直接查询 odom 定位状态)")

    print(f"\n{'=' * 60}")
    if all_ok:
        print(f"✓ 所有环境检查通过")
    else:
        print(f"✗ 存在环境问题, 请按上述提示修复后再运行")
    print(f"{'=' * 60}\n")

    return all_ok


def main():
    import argparse
    parser = argparse.ArgumentParser(description="环境前置检查")
    parser.add_argument("--skip-slam", action="store_true",
                        help="兼容参数 (保留, SLAM 状态请通过 HMI 确认)")
    args = parser.parse_args()

    ok = run_all_checks(skip_slam=args.skip_slam)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
