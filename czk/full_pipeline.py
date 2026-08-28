#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""full_pipeline.py — 全自动上料流水线总控 (仿 wzd/full_pick_pipeline.py)

流程 (6 Phase):
  Phase 0. 双臂待机姿态 (pose_standby.json)
  Phase A. 取货三步纠偏 (shangliaoqu.ref, 辉羲 RPU 推理)
    ① 前后纠偏 (FB)  — 深度对齐 target_depth
    ② 角度纠偏 (YAW) — a/b 连线斜率归零
    ③ 左右纠偏 (LR)  — a/b 中点对齐目标 x
  Phase B. 取货动作 (自动手臂+夹爪)
    张开夹爪 → 预抓取 → 抓取 → 闭合夹爪 → 抬升 → 待机
  Phase C. 底盘后退 300mm + 逆时针旋转 179° + 向右横移 200mm
  Phase D. 放货三步纠偏 (jitai_new.ref, YAW→FB→标定→LR, 辉羲 RPU 推理)
  Phase E. 放货动作 (自动手臂+夹爪, 与取货对称)
    待机 → 放货抬升姿态 → 预放货 → 放货 → 张开夹爪 → 待机

★ czk 约束: 仅辉羲 RPU 推理 (无 REMOTE_INFER / 本地 YOLO 回退)
★ 所有位姿 JSON 和手臂脚本使用本目录自带 execute/ 与 position/ 资源

用法:
  python3 full_pipeline.py                              # 完整流水线
  python3 full_pipeline.py --dry-run                    # 只打印流程, 不实际执行
  python3 full_pipeline.py --skip-init-pose             # 跳过Phase0待机姿态
  python3 full_pipeline.py --skip-correct-pick          # 跳过取货三步纠偏
  python3 full_pipeline.py --skip-pick                  # 跳过取货动作
  python3 full_pipeline.py --skip-rotate                # 跳过后退+旋转+横移
  python3 full_pipeline.py --skip-correct-place         # 跳过放货三步纠偏
  python3 full_pipeline.py --skip-place                 # 跳过放货动作
  python3 full_pipeline.py --rotate-deg -180            # 自定义旋转角度 (正值=逆时针, 负值=顺时针)
  python3 full_pipeline.py --backward-m 0.3             # 旋转先后退距离 (米, 默认0.3)
  python3 full_pipeline.py --lateral-m -0.2             # 旋转后横移距离 (米, 正=左负=右)
  python3 full_pipeline.py --target-depth 1050          # 覆盖取货前后纠偏目标深度 mm
  python3 full_pipeline.py --skip-calibration           # 跳过取货LR前标定, 用yaml缓存px_to_meter
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import detector
import chassis
import correct
import check_env
from main import load_config, setup_modules


# ═══════════════════════════════════════════════════════════
#  路径配置 (自带资源, 自给自足)
# ═══════════════════════════════════════════════════════════

WZD_DIR      = _HERE
EXEC_DIR     = os.path.join(WZD_DIR, "execute")
POSITION_DIR = os.path.join(WZD_DIR, "position")

# Python 解释器 (用于 subprocess 调用手臂脚本, 含 paho-mqtt)
DEFAULT_PY = "/usr/bin/python3"

# ── 手臂控制脚本 ──
SCRIPT_STANDBY   = os.path.join(EXEC_DIR, "move_arms_to_standby.py")
SCRIPT_PRE_PICK  = os.path.join(EXEC_DIR, "move_arms_to_pre_pick.py")
SCRIPT_PICK      = os.path.join(EXEC_DIR, "move_arms_to_pick.py")
SCRIPT_LIFT      = os.path.join(EXEC_DIR, "move_arms_to_lift.py")
SCRIPT_PRE_PLACE = os.path.join(EXEC_DIR, "move_arms_to_pre_place.py")
SCRIPT_PLACE     = os.path.join(EXEC_DIR, "move_arms_to_place.py")
SCRIPT_MOVE_POSE = os.path.join(EXEC_DIR, "move_arms_to_pose.py")

# ── 位姿 JSON ──
POSE_INITIAL   = os.path.join(POSITION_DIR, "pose_initial.json")
POSE_STANDBY   = os.path.join(POSITION_DIR, "pose_standby.json")
POSE_PRE_PICK  = os.path.join(POSITION_DIR, "pose_pre_pick.json")
POSE_PICK      = os.path.join(POSITION_DIR, "pose_pick.json")
POSE_LIFT      = os.path.join(POSITION_DIR, "pose_lift.json")
POSE_PRE_PLACE = os.path.join(POSITION_DIR, "pose_pre_place.json")
POSE_PLACE     = os.path.join(POSITION_DIR, "pose_place.json")
POSE_PLACING_LIFT = os.path.join(POSITION_DIR, "pose_placing_lift.json")

# ── MQTT 夹爪控制配置 ──
MQTT_BROKER = "localhost"
MQTT_PORT   = 1883
TOPIC_COMMANDS_DATA = "/humanoid/commands/data"
TOPIC_DONE = "/humanoid/commands/done"

# 夹爪位置 (参考 pick.py: 0.0=闭合, -0.7=张开)
RIGHT_GRIPPER_CLOSE_POS = 0.0
LEFT_GRIPPER_CLOSE_POS  = 0.0
RIGHT_GRIPPER_OPEN_POS  = -0.7
LEFT_GRIPPER_OPEN_POS   = -0.7

# ── 默认参数 ──
DEFAULT_ROTATE_DEG    = 179.0   # 默认逆时针 179° (正值=逆时针, 负值=顺时针)
DEFAULT_BACKWARD_M    = 0.3     # 旋转先后退距离 (米, 0.3=300mm)
DEFAULT_LATERAL_M     = -0.2    # 旋转后向右横移距离 (米, 正=左, 负=右, -0.2=向右200mm)
DEFAULT_TARGET_DEPTH = 700     # 取货前后纠偏目标深度 mm (取 correct.yaml pick 场景值)
DEFAULT_OUTPUT_DIR   = os.path.join(_HERE, "output")
DEFAULT_STEP_PAUSE   = 0.1     # 取/放货每步之间停顿秒数
DEFAULT_GRIPPER_WAIT = 1.0     # 夹爪动作完成后额外等待秒数


# ═══════════════════════════════════════════════════════════
#  夹爪控制 (MQTT grab 命令)
# ═══════════════════════════════════════════════════════════

def send_gripper(broker=MQTT_BROKER, port=MQTT_PORT,
                 left_pos=LEFT_GRIPPER_CLOSE_POS,
                 right_pos=RIGHT_GRIPPER_CLOSE_POS,
                 wait=DEFAULT_GRIPPER_WAIT, timeout=5.0, tag="夹爪"):
    """发送夹爪控制命令 (grab)

    发送 {"command":"grab","data":{"left":L,"right":R}} 到 /humanoid/commands/data
    订阅 /humanoid/commands/done 等待执行完成, 超时后继续.

    Parameters
    ----------
    left_pos / right_pos: 0.0=闭合, -0.7=张开
    wait: 收到完成通知后额外等待秒数
    timeout: 等待完成通知的超时秒数
    tag: 日志标签
    """
    import paho.mqtt.client as mqtt

    payload = json.dumps({
        "command": "grab",
        "data": {"left": left_pos, "right": right_pos}
    })
    state = {"ok": False, "error": False}

    def on_connect(client, _userdata, _flags, rc, _properties=None):
        if rc == 0:
            client.subscribe(TOPIC_DONE, qos=0)
            client.publish(TOPIC_COMMANDS_DATA, payload, qos=2)
            print(f"[{tag}] 已发送 grab 命令 (left={left_pos:.2f}, right={right_pos:.2f})")
        else:
            print(f"[{tag}] MQTT 连接失败, rc={rc}")
            state["error"] = True

    def on_message(client, _userdata, msg):
        if msg.topic == TOPIC_DONE:
            try:
                data = json.loads(msg.payload.decode("utf-8"))
                if data.get("cmd") == "grab":
                    state["ok"] = True
                    client.disconnect()
            except Exception:
                pass

    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message
    try:
        client.connect(broker, port, keepalive=60)
    except Exception as e:
        print(f"[{tag}] ✗ 连接 MQTT 失败: {e}")
        return False

    t_start = time.time()
    while not state["ok"] and not state["error"] and time.time() - t_start < timeout:
        client.loop(timeout=0.1)

    try:
        client.disconnect()
    except Exception:
        pass

    if state["error"]:
        print(f"[{tag}] ✗ 连接失败")
        return False
    if not state["ok"]:
        print(f"[{tag}] ✗ 等待完成超时 ({timeout}s), 夹爪动作可能未执行")
        return False
    print(f"[{tag}] ✓ 收到完成通知")
    if wait > 0:
        time.sleep(wait)
    return True


# ═══════════════════════════════════════════════════════════
#  手臂运动 (subprocess 调用 wzd/execute/*.py)
# ═══════════════════════════════════════════════════════════

def run_arm_script(name, script, py_bin=DEFAULT_PY, dry_run=False, timeout=30.0):
    """调用 wzd/execute/ 下的手臂控制脚本

    Returns
    -------
    bool: True=成功 (exit code 0)
    """
    print(f"\n{'='*60}")
    print(f"[{name}] 开始")
    print(f"[{name}] 脚本: {script}")
    print(f"{'='*60}")

    if dry_run:
        print(f"[{name}] [DRY-RUN] 跳过")
        return True

    if not os.path.exists(script):
        print(f"[{name}] ✗ 脚本不存在: {script}")
        return False

    t0 = time.time()
    try:
        cmd = [py_bin, "-u", script, "--timeout", str(timeout)]
        ret = subprocess.run(cmd, check=False)
        dt = time.time() - t0
        if ret.returncode == 0:
            print(f"[{name}] ✓ 成功 (耗时 {dt:.1f}s)")
            return True
        else:
            print(f"[{name}] ✗ 失败 (退出码 {ret.returncode}, 耗时 {dt:.1f}s)")
            return False
    except Exception as e:
        print(f"[{name}] ✗ 异常: {e}")
        return False


def run_arm_pose(name, pose_file, py_bin=DEFAULT_PY, dry_run=False, timeout=30.0):
    """运动到指定 JSON 位姿 (通过 move_arms_to_pose.py --pose)

    Returns
    -------
    bool: True=成功
    """
    print(f"\n{'='*60}")
    print(f"[{name}] 开始")
    print(f"[{name}] 位姿: {pose_file}")
    print(f"{'='*60}")

    if dry_run:
        print(f"[{name}] [DRY-RUN] 跳过")
        return True

    if not os.path.exists(pose_file):
        print(f"[{name}] ✗ 位姿文件不存在: {pose_file}")
        return False
    if not os.path.exists(SCRIPT_MOVE_POSE):
        print(f"[{name}] ✗ 脚本不存在: {SCRIPT_MOVE_POSE}")
        return False

    t0 = time.time()
    try:
        cmd = [py_bin, "-u", SCRIPT_MOVE_POSE,
               "--pose", pose_file, "--timeout", str(timeout)]
        ret = subprocess.run(cmd, check=False)
        dt = time.time() - t0
        if ret.returncode == 0:
            print(f"[{name}] ✓ 成功 (耗时 {dt:.1f}s)")
            return True
        else:
            print(f"[{name}] ✗ 失败 (退出码 {ret.returncode}, 耗时 {dt:.1f}s)")
            return False
    except Exception as e:
        print(f"[{name}] ✗ 异常: {e}")
        return False


# ═══════════════════════════════════════════════════════════
#  Phase A: 取货三步纠偏 (FB → YAW → LR)
# ═══════════════════════════════════════════════════════════

def run_pick_correct(model, g2, target_depth, output_dir, cfg,
                     skip_calibration=False, dry_run=False):
    """取货三步纠偏

    Parameters
    ----------
    model: detector.RhinoInfer (辉羲 RPU 推理实例)
    g2: minth.G2 实例
    target_depth: 前后纠偏目标深度 mm
    output_dir: 结果图保存目录
    cfg: correct.yaml 配置 dict (取场景覆盖参数)
    skip_calibration: True=跳过LR前标定, 用yaml缓存px_to_meter_override
    dry_run: True=只打印不执行

    Returns
    -------
    (ok, results): ok=False 表示有异常需终止流程
    """
    pick_cfg = cfg.get("scenes", {}).get("pick", {})
    model_ref = pick_cfg.get("model_ref", "models/best_new.ref")
    lr_target_x_override = pick_cfg.get("lr_target_x_override", None)
    lr_px_to_meter_cache = pick_cfg.get("px_to_meter_override", None)
    calib_depth = pick_cfg.get("px_to_meter_calib_depth", None)
    correct_yaml = os.path.join(_HERE, "config", "correct.yaml")

    # 自动判定 LR 前是否标定: 目标深度改变→重新标定, 未变→跳过
    if skip_calibration:
        calibrate_before_lr = False
        calib_reason = "手动跳过(--skip-calibration)"
    elif calib_depth is None or calib_depth != target_depth:
        calibrate_before_lr = True
        calib_reason = (f"目标深度变化 ({calib_depth} → {target_depth}mm), 重新标定"
                        if calib_depth is not None else "无标定深度记录, 首次标定")
    else:
        calibrate_before_lr = False
        calib_reason = f"目标深度未变 ({target_depth}mm), 跳过标定(用缓存)"

    print(f"\n{'#'*60}")
    print(f"# Phase A: 取货三步纠偏 ({model_ref}, 辉羲 RPU)")
    print(f"{'#'*60}")
    print(f"[纠偏] 模型: {model_ref}")
    print(f"[纠偏] 目标深度: {target_depth}mm")
    print(f"[纠偏] 结果保存: {output_dir}")
    print(f"[纠偏] LR前标定: {calib_reason}")

    if dry_run:
        print(f"[纠偏] [DRY-RUN] 跳过")
        return True, {}

    # 直接调用 correct.run_correct (与 main.py/pipeline.py 共用逻辑)
    results = correct.run_correct(
        model, g2, target_depth, output_dir, dry_run=dry_run,
        lr_target_x_override=lr_target_x_override,
        calibrate_before_lr=calibrate_before_lr,
        scene_name="pick",
        yaml_path=correct_yaml,
        lr_px_to_meter_cache=lr_px_to_meter_cache)

    abort = results.get("abort", False)

    # 汇总各步收敛状态
    fb_ok  = results.get("fb", {}).get("success") and results.get("fb", {}).get("converged", False)
    yaw_ok = results.get("yaw", {}).get("success") and results.get("yaw", {}).get("converged", False)
    lr_ok  = results.get("lr", {}).get("success") and results.get("lr", {}).get("converged", False)
    print(f"\n[取货纠偏] 前后={'✓' if fb_ok else '⚠'}  角度={'✓' if yaw_ok else '⚠'}  左右={'✓' if lr_ok else '⚠'}")

    no_error = not abort
    return no_error, results


# ═══════════════════════════════════════════════════════════
#  Phase B: 取货动作 (张爪→预抓→抓取→闭合→抬升→待机)
# ═══════════════════════════════════════════════════════════

def run_pick(py_bin=DEFAULT_PY, dry_run=False,
             pause=DEFAULT_STEP_PAUSE, gripper_wait=DEFAULT_GRIPPER_WAIT):
    """执行取货动作 (张开夹爪发生在取货纠偏之后, 即本阶段第一步)

    流程: 张开夹爪 → 预抓取 → 抓取 → 闭合夹爪 → 抬升 → 待机

    Returns
    -------
    (ok, step_results, elapsed_s)
    """
    print(f"\n{'#'*60}")
    print(f"# Phase B: 取货动作")
    print(f"{'#'*60}")

    t_start = time.time()
    results = []

    def _record(name, ok):
        results.append((name, ok))
        return ok

    def _pause():
        if pause > 0:
            time.sleep(pause)

    # ── 步骤 0: 张开夹爪 ──
    print(f"\n{'='*60}")
    print(f"[0.张开夹爪] 开始")
    print(f"{'='*60}")
    if dry_run:
        print("[0.张开夹爪] [DRY-RUN] 跳过")
        _record("0.张开夹爪", True)
    else:
        ok = send_gripper(left_pos=LEFT_GRIPPER_OPEN_POS,
                          right_pos=RIGHT_GRIPPER_OPEN_POS,
                          wait=gripper_wait, tag="0.张开夹爪")
        _record("0.张开夹爪", ok)
        if not ok:
            print("⚠ 张开夹爪失败, 终止取货流程")
            return False, results, time.time() - t_start
        print("[0.张开夹爪] ✓ 完成")
    _pause()

    # ── 步骤 1: 预抓取姿态 ──
    ok = run_arm_script("1.预抓取姿态", SCRIPT_PRE_PICK, py_bin, dry_run=dry_run)
    _record("1.预抓取姿态", ok)
    if not ok:
        return False, results, time.time() - t_start
    _pause()

    # ── 步骤 2: 抓取姿态 ──
    ok = run_arm_script("2.抓取姿态", SCRIPT_PICK, py_bin, dry_run=dry_run)
    _record("2.抓取姿态", ok)
    if not ok:
        return False, results, time.time() - t_start
    _pause()

    # ── 步骤 3: 闭合夹爪 ──
    print(f"\n{'='*60}")
    print(f"[3.闭合夹爪] 开始")
    print(f"{'='*60}")
    if dry_run:
        print("[3.闭合夹爪] [DRY-RUN] 跳过")
        _record("3.闭合夹爪", True)
    else:
        ok = send_gripper(left_pos=LEFT_GRIPPER_CLOSE_POS,
                          right_pos=RIGHT_GRIPPER_CLOSE_POS,
                          wait=gripper_wait, tag="3.闭合夹爪")
        _record("3.闭合夹爪", ok)
        if not ok:
            print("⚠ 闭合夹爪失败, 终止取货流程")
            return False, results, time.time() - t_start
        print("[3.闭合夹爪] ✓ 完成")
    _pause()

    # ── 步骤 4: 抬升姿态 ──
    ok = run_arm_script("4.抬升姿态", SCRIPT_LIFT, py_bin, dry_run=dry_run)
    _record("4.抬升姿态", ok)
    if not ok:
        return False, results, time.time() - t_start
    _pause()

    # ── 步骤 5: 回到待机姿态 ──
    ok = run_arm_script("5.回到待机姿态", SCRIPT_STANDBY, py_bin, dry_run=dry_run)
    _record("5.回到待机姿态", ok)
    if not ok:
        return False, results, time.time() - t_start

    return True, results, time.time() - t_start


# ═══════════════════════════════════════════════════════════
#  Phase C: 底盘后退 + 旋转 + 横向移动
# ═══════════════════════════════════════════════════════════

def rotate_chassis(g2, deg, settle_s=3.0, dry_run=False, backward_m=0.0, lateral_m=0.0):
    """底盘后退 + 旋转 + 横向移动 (后退 → 旋转 → 横移)

    Parameters
    ----------
    deg: 旋转角度 (度). 正值=逆时针, 负值=顺时针 (右手定则)
    settle_s: 单段运动后稳定时间 (秒)
    backward_m: 后退距离 (米, 正值=后退), 0 表示不后退
    lateral_m: 横向移动距离 (米, 正值=左, 负值=右), 0 表示不横移
    """
    yaw_rad = math.radians(deg)
    direction = "逆时针" if deg > 0 else "顺时针"
    abs_deg = abs(deg)

    print(f"\n{'#'*60}")
    print(f"# Phase C: 底盘后退 + 旋转 + 横向移动")
    print(f"{'#'*60}")
    if backward_m:
        print(f"[后退] 后退 {backward_m * 1000:.0f}mm")
    print(f"[旋转] 原地{direction} {abs_deg:.0f}° (yaw_rad={yaw_rad:+.4f})")
    if lateral_m:
        side = "左" if lateral_m > 0 else "右"
        print(f"[横移] 向{side}移动 {abs(lateral_m) * 1000:.0f}mm")

    if dry_run:
        print(f"[后退+旋转+横移] [DRY-RUN] 跳过实际运动")
        print(f"[后退+旋转+横移] [DRY-RUN] 跳过 {settle_s:.1f}s 稳定等待")
        return True

    # 1) 后退
    if backward_m:
        ok = chassis.move_chassis(g2, dx_m=-backward_m)
        if not ok:
            print(f"[后退] ✗ 后退命令执行失败")
            return False
        print(f"[后退] ✓ 后退完成, 等待底盘稳定 {settle_s:.1f}s ...")
        time.sleep(settle_s)
        print(f"[后退] ✓ 底盘稳定")

    # 2) 旋转
    ok = chassis.move_chassis(g2, yaw_rad=yaw_rad)
    if not ok:
        print(f"[旋转] ✗ 旋转命令执行失败")
        return False
    print(f"[旋转] ✓ 旋转完成, 等待底盘稳定 {settle_s:.1f}s ...")
    time.sleep(settle_s)
    print(f"[旋转] ✓ 底盘稳定")

    # 3) 横向移动
    if lateral_m:
        ok = chassis.move_chassis(g2, dy_m=lateral_m)
        if not ok:
            print(f"[横移] ✗ 横移命令执行失败")
            return False
        print(f"[横移] ✓ 横移完成, 等待底盘稳定 {settle_s:.1f}s ...")
        time.sleep(settle_s)
        print(f"[横移] ✓ 底盘稳定")

    return True


# ═══════════════════════════════════════════════════════════
#  Phase D: 放货三步纠偏 (YAW → FB → 标定 → LR)
# ═══════════════════════════════════════════════════════════

def run_place_correct(model, g2, output_dir, cfg, dry_run=False):
    """放货三步纠偏 (YAW → FB → 标定 → LR)

    使用 jitai_new.ref 模型, 复用 correct.run_correct, 场景设为 place
    顺序: 角度 → 前后 → [标定] → 左右 (标定绑在 LR 前, 目标深度变化才重新标定)
    """
    place_cfg = cfg.get("scenes", {}).get("place", {})
    target_depth = place_cfg.get("target_depth", DEFAULT_TARGET_DEPTH)
    lr_target_x_override = place_cfg.get("lr_target_x_override", None)
    lr_px_to_meter_cache = place_cfg.get("px_to_meter_override", None)
    calib_depth = place_cfg.get("px_to_meter_calib_depth", None)
    correct_yaml = os.path.join(_HERE, "config", "correct.yaml")

    # 自动判定 LR 前是否标定: 目标深度改变→重新标定, 未变→跳过 (与取货一致)
    if calib_depth is None or calib_depth != target_depth:
        calibrate_before_lr = True
        calib_reason = (f"目标深度变化 ({calib_depth} → {target_depth}mm), 重新标定"
                        if calib_depth is not None else "无标定深度记录, 首次标定")
    else:
        calibrate_before_lr = False
        calib_reason = f"目标深度未变 ({target_depth}mm), 跳过标定(用缓存)"

    print(f"\n{'#'*60}")
    print(f"# Phase D: 放货三步纠偏 (jitai_new.ref, YAW→FB→标定→LR)")
    print(f"{'#'*60}")
    print(f"[纠偏] 模型: jitai_new.ref")
    print(f"[纠偏] 目标深度: {target_depth}mm")
    print(f"[纠偏] 结果保存: {output_dir}")
    print(f"[纠偏] LR前标定: {calib_reason}")

    if dry_run:
        print(f"[纠偏] [DRY-RUN] 跳过")
        return True

    results = correct.run_correct(
        model, g2, target_depth, output_dir, dry_run=dry_run,
        lr_target_x_override=lr_target_x_override,
        calibrate_before_lr=calibrate_before_lr,
        scene_name="place",
        yaml_path=correct_yaml,
        lr_px_to_meter_cache=lr_px_to_meter_cache,
        order=["yaw", "fb", "lr"])

    abort = results.get("abort", False)
    fb_ok  = results.get("fb", {}).get("success") and results.get("fb", {}).get("converged", False)
    yaw_ok = results.get("yaw", {}).get("success") and results.get("yaw", {}).get("converged", False)
    lr_ok  = results.get("lr", {}).get("success") and results.get("lr", {}).get("converged", False)
    print(f"\n[放货纠偏] 角度={'✓' if yaw_ok else '⚠'}  前后={'✓' if fb_ok else '⚠'}  标定→左右={'✓' if lr_ok else '⚠'}")

    # 放货纠偏未收敛不终止 (放货优先), 仅当 abort 时返回 False
    return not abort


# ═══════════════════════════════════════════════════════════
#  Phase E: 放货动作 (待机→放货抬升→预放→放货→张开→待机)
# ═══════════════════════════════════════════════════════════

def run_place(py_bin=DEFAULT_PY, dry_run=False,
              pause=DEFAULT_STEP_PAUSE, gripper_wait=DEFAULT_GRIPPER_WAIT):
    """执行放货动作 (与取货对称)

    流程: 1.待机 → 2.放货抬升姿态 → 3.预放货 → 4.放货 → 5.张开夹爪 → 6.待机
    """
    print(f"\n{'#'*60}")
    print(f"# Phase E: 放货动作")
    print(f"{'#'*60}")

    t_start = time.time()
    results = []

    def _record(name, ok):
        results.append((name, ok))
        return ok

    def _pause():
        if pause > 0:
            time.sleep(pause)

    # ── 步骤 1: 待机姿态 (确认) ──
    ok = run_arm_script("1.待机姿态", SCRIPT_STANDBY, py_bin, dry_run=dry_run)
    _record("1.待机姿态", ok)
    if not ok:
        return False, results, time.time() - t_start
    _pause()

    # ── 步骤 2: 放货抬升姿态 ──
    ok = run_arm_pose("2.放货抬升姿态", POSE_PLACING_LIFT, py_bin, dry_run=dry_run)
    _record("2.放货抬升姿态", ok)
    if not ok:
        return False, results, time.time() - t_start
    _pause()

    # ── 步骤 3: 预放货姿态 ──
    ok = run_arm_script("3.预放货姿态", SCRIPT_PRE_PLACE, py_bin, dry_run=dry_run)
    _record("3.预放货姿态", ok)
    if not ok:
        return False, results, time.time() - t_start
    _pause()

    # ── 步骤 4: 放货姿态 ──
    ok = run_arm_script("4.放货姿态", SCRIPT_PLACE, py_bin, dry_run=dry_run)
    _record("4.放货姿态", ok)
    if not ok:
        return False, results, time.time() - t_start
    _pause()

    # ── 步骤 5: 张开夹爪 (放货) ──
    print(f"\n{'='*60}")
    print(f"[5.张开夹爪(放货)] 开始")
    print(f"{'='*60}")
    if dry_run:
        print("[5.张开夹爪(放货)] [DRY-RUN] 跳过")
        _record("5.张开夹爪(放货)", True)
    else:
        ok = send_gripper(left_pos=LEFT_GRIPPER_OPEN_POS,
                          right_pos=RIGHT_GRIPPER_OPEN_POS,
                          wait=gripper_wait, tag="5.张开夹爪(放货)")
        _record("5.张开夹爪(放货)", ok)
        if not ok:
            print("⚠ 张开夹爪失败, 但继续 (放货完成优先)")
        else:
            print("[5.张开夹爪(放货)] ✓ 完成 (货物已放下)")
    _pause()

    # ── 步骤 6: 回到待机姿态 ──
    ok = run_arm_script("6.回到待机姿态", SCRIPT_STANDBY, py_bin, dry_run=dry_run)
    _record("6.回到待机姿态", ok)
    if not ok:
        # 放货已完成, 待机失败不影响整体结果
        print("⚠ 回到待机姿态失败, 但放货已完成, 流程结束")

    return True, results, time.time() - t_start


# ═══════════════════════════════════════════════════════════
#  主流程
# ═══════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="全自动上料流水线总控 (仿 wzd/full_pick_pipeline.py)")
    # 解释器 / 时序
    parser.add_argument("--python-bin", default=DEFAULT_PY,
                        help=f"手臂子脚本Python解释器 (默认: {DEFAULT_PY})")
    parser.add_argument("--pause", type=float, default=DEFAULT_STEP_PAUSE,
                        help=f"取/放货动作每步停顿秒数 (默认: {DEFAULT_STEP_PAUSE})")
    parser.add_argument("--gripper-wait", type=float, default=DEFAULT_GRIPPER_WAIT,
                        help=f"夹爪动作完成后额外等待秒数 (默认: {DEFAULT_GRIPPER_WAIT})")
    parser.add_argument("--rotate-settle", type=float, default=3.0,
                        help="旋转后稳定时间 (秒, 默认 3.0)")
    # 纠偏参数
    parser.add_argument("--target-depth", type=int, default=None,
                        help="取货前后纠偏目标深度mm (默认读取correct.yaml pick场景值)")
    parser.add_argument("--rotate-deg", type=float, default=DEFAULT_ROTATE_DEG,
                        help=f"旋转角度, 正值=逆时针, 负值=顺时针 (默认: {DEFAULT_ROTATE_DEG})")
    parser.add_argument("--backward-m", type=float, default=DEFAULT_BACKWARD_M,
                        help=f"旋转先后退距离(米), 0=不后退 (默认: {DEFAULT_BACKWARD_M})")
    parser.add_argument("--lateral-m", type=float, default=DEFAULT_LATERAL_M,
                        help=f"旋转后横向移动距离(米), 正=左负=右 (默认: {DEFAULT_LATERAL_M})")
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR,
                        help=f"纠偏结果保存根目录 (默认: {DEFAULT_OUTPUT_DIR})")
    parser.add_argument("--skip-calibration", action="store_true",
                        help="跳过取货LR前标定, 用correct.yaml缓存px_to_meter_override")
    # 流程控制 (跳过)
    parser.add_argument("--skip-env-check", action="store_true",
                        help="跳过环境前置检查")
    parser.add_argument("--skip-slam-check", action="store_true",
                        help="环境检查时跳过SLAM提示(兼容)")
    parser.add_argument("--skip-init-pose", action="store_true",
                        help="跳过Phase0待机姿态")
    parser.add_argument("--skip-correct-pick", action="store_true",
                        help="跳过取货三步纠偏")
    parser.add_argument("--skip-pick", action="store_true",
                        help="跳过取货动作")
    parser.add_argument("--skip-rotate", action="store_true",
                        help="跳过后退+旋转+横移")
    parser.add_argument("--skip-correct-place", action="store_true",
                        help="跳过放货三步纠偏")
    parser.add_argument("--skip-place", action="store_true",
                        help="跳过放货动作")
    parser.add_argument("--dry-run", action="store_true",
                        help="只打印流程, 不实际执行")

    args = parser.parse_args()

    py = args.python_bin
    dry = args.dry_run
    t_total = time.time()

    # ── 加载配置 ──
    correct_yaml = os.path.join(_HERE, "config", "correct.yaml")
    if not os.path.exists(correct_yaml):
        print(f"✗ 配置文件不存在: {correct_yaml}")
        return 1
    cfg = load_config(correct_yaml)
    setup_modules(cfg)

    # ── 输出目录 ──
    pick_output  = os.path.join(args.output_dir, "pick")
    place_output = os.path.join(args.output_dir, "place")
    os.makedirs(pick_output, exist_ok=True)
    os.makedirs(place_output, exist_ok=True)

    # ── 从配置读取场景级 target_depth (如果没显式指定) ──
    pick_cfg = cfg.get("scenes", {}).get("pick", {})
    target_depth = args.target_depth or pick_cfg.get("target_depth", DEFAULT_TARGET_DEPTH)

    # ── 环境检查 ──
    if not args.skip_env_check:
        ok = check_env.run_all_checks(skip_slam=args.skip_slam_check)
        if not ok:
            print(f"\n环境检查未通过, 终止. (加 --skip-env-check 可跳过)")
            return 1

    # ── 流程概览 ──
    rot_dir = "逆时针" if args.rotate_deg > 0 else "顺时针"
    backward_label = f"后退{args.backward_m * 1000:.0f}mm" if args.backward_m else "不后退"
    if args.lateral_m == 0:
        lateral_label = "无横移"
    else:
        side = "左" if args.lateral_m > 0 else "右"
        lateral_label = f"{side}{abs(args.lateral_m) * 1000:.0f}mm"
    print(f"\n{'=' * 60}")
    print(f"全自动上料流水线总控")
    print(f"{'=' * 60}")
    print(f"  Dry Run: {dry}")
    print(f"  取货目标深度: {target_depth}mm")
    print(f"  旋转: {rot_dir} {abs(args.rotate_deg):.0f}°")
    print(f"  旋转前后退: {backward_label}")
    print(f"  旋转后横移: {lateral_label}")
    print(f"  旋转稳定: {args.rotate_settle:.1f}s")
    print(f"  动作步停顿: {args.pause}s")
    print(f"  夹爪等待: {args.gripper_wait}s")
    print(f"  LR前标定: {'手动跳过(--skip-calibration)' if args.skip_calibration else '自动(目标深度改变时重新标定)'}")

    parts = []
    if not args.skip_init_pose:
        parts.append("0.待机姿态")
    if not args.skip_correct_pick:
        parts.append("A.取货三步纠偏")
    if not args.skip_pick:
        parts.append("B.取货动作")
    if not args.skip_rotate:
        parts.append(f"C.后退+旋转+横移({backward_label}+{rot_dir}{abs(args.rotate_deg):.0f}°+{lateral_label})")
    if not args.skip_correct_place:
        parts.append("D.放货三步纠偏(YAW→FB→标定→LR)")
    if not args.skip_place:
        parts.append("E.放货动作")
    print(f"  流程: {' → '.join(parts)}")
    print(f"{'=' * 60}")

    # ── 加载模型 (辉羲 RPU 仅 RPU, 无回退) ──
    model_pick = None
    model_place = None
    g2 = None

    if not dry:
        need_pick_model  = (not args.skip_correct_pick)
        need_place_model = (not args.skip_correct_place)
        need_robot = (need_pick_model or need_place_model or (not args.skip_rotate))

        if need_robot:
            print(f"\n[初始化] 连接机器人...")
            g2 = chassis.setup_minth()
            print(f"[初始化] ✓ Minth 已就绪")

        if need_pick_model:
            pick_ref = pick_cfg.get("model_ref", "models/best_new.ref")
            if not os.path.isabs(pick_ref):
                pick_ref = os.path.join(_HERE, pick_ref)
            print(f"[初始化] 辉羲 RPU 取货模型: {pick_ref}")
            model_pick = detector.RhinoInfer(pick_ref)
            print(f"[初始化] ✓ 取货模型就绪")

        if need_place_model:
            place_cfg = cfg.get("scenes", {}).get("place", {})
            place_ref = place_cfg.get("model_ref", "models/jitai_new.ref")
            if not os.path.isabs(place_ref):
                place_ref = os.path.join(_HERE, place_ref)
            print(f"[初始化] 辉羲 RPU 放货模型: {place_ref}")
            model_place = detector.RhinoInfer(place_ref)
            print(f"[初始化] ✓ 放货模型就绪")

    all_results = []

    def _log(name, ok):
        all_results.append((name, ok))

    def _log_many(items):
        """批量写入 results (items: list of (name, ok))"""
        all_results.extend(items)

    try:
        # ════════════════════════════════════════
        # Phase 0: 待机姿态
        # ════════════════════════════════════════
        if not args.skip_init_pose:
            ok = run_arm_pose("Phase0-待机姿态", POSE_STANDBY, py, dry_run=dry)
            _log("0.待机姿态", ok)
            if not ok:
                print(f"\n⚠⚠ 待机姿态失败, 终止!")
                _summary(all_results, t_total)
                return 1
        else:
            print(f"\n[Phase 0] 已跳过 (--skip-init-pose)")

        # ════════════════════════════════════════
        # Phase A: 取货三步纠偏
        # ════════════════════════════════════════
        if not args.skip_correct_pick:
            ok, _ = run_pick_correct(model_pick, g2, target_depth,
                                     pick_output, cfg,
                                     skip_calibration=args.skip_calibration,
                                     dry_run=dry)
            _log("A.取货三步纠偏", ok)
            if not ok:
                print(f"\n⚠⚠ 取货纠偏异常, 终止整个流水线!")
                _summary(all_results, t_total)
                return 1
        else:
            print(f"\n[Phase A] 已跳过 (--skip-correct-pick)")

        # ════════════════════════════════════════
        # Phase B: 取货动作
        # ════════════════════════════════════════
        if not args.skip_pick:
            ok, pick_steps, pick_dt = run_pick(
                py, dry_run=dry, pause=args.pause,
                gripper_wait=args.gripper_wait)
            _log_many([(f"B.{n}", o) for n, o in pick_steps])
            if not ok:
                print(f"\n⚠⚠ 取货动作失败, 终止整个流水线!")
                _summary(all_results, t_total)
                return 1
            print(f"\n[Phase B] ✓ 取货完成 (总耗时 {pick_dt:.1f}s)")
        else:
            print(f"\n[Phase B] 已跳过 (--skip-pick)")

        # ════════════════════════════════════════
        # Phase C: 后退 + 旋转 + 横向移动
        # ════════════════════════════════════════
        if not args.skip_rotate:
            ok = rotate_chassis(g2, args.rotate_deg, settle_s=args.rotate_settle,
                                dry_run=dry, backward_m=args.backward_m, lateral_m=args.lateral_m)
            _log(f"C.后退+旋转+横移({backward_label}+{rot_dir}{abs(args.rotate_deg):.0f}°+{lateral_label})", ok)
            if not ok and not dry:
                print(f"\n⚠⚠ 旋转失败, 终止!")
                _summary(all_results, t_total)
                return 1
        else:
            print(f"\n[Phase C] 已跳过 (--skip-rotate)")

        # ════════════════════════════════════════
        # Phase D: 放货三步纠偏
        # ════════════════════════════════════════
        if not args.skip_correct_place:
            ok = run_place_correct(model_place, g2, place_output, cfg, dry_run=dry)
            _log("D.放货三步纠偏(YAW→FB→标定→LR)", ok)
            # 放货纠偏未收敛不终止 (放货优先)
        else:
            print(f"\n[Phase D] 已跳过 (--skip-correct-place)")

        # ════════════════════════════════════════
        # Phase E: 放货动作
        # ════════════════════════════════════════
        if not args.skip_place:
            ok, place_steps, place_dt = run_place(
                py, dry_run=dry, pause=args.pause,
                gripper_wait=args.gripper_wait)
            _log_many([(f"E.{n}", o) for n, o in place_steps])
            if not ok:
                print(f"\n⚠ 放货动作部分失败, 流程继续")
            else:
                print(f"\n[Phase E] ✓ 放货完成 (总耗时 {place_dt:.1f}s)")
        else:
            print(f"\n[Phase E] 已跳过 (--skip-place)")

        _summary(all_results, t_total)
        return 0

    except KeyboardInterrupt:
        print(f"\n[中断] 用户中止 (Ctrl+C)")
        return 130

    finally:
        if g2 is not None:
            try:
                g2.close()
                print(f"\n[清理] Minth 连接已关闭")
            except Exception:
                pass


def _summary(results, t_start):
    """打印流水线总结"""
    elapsed = time.time() - t_start
    print(f"\n{'=' * 60}")
    print(f"全自动上料流水线总结 (总耗时 {elapsed:.1f}s)")
    print(f"{'=' * 60}")
    for name, ok in results:
        status = "✓ 成功" if ok else "✗ 失败"
        print(f"  {name}: {status}")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        import traceback
        print(f"\n{'=' * 60}")
        print(f"✗ 程序异常崩溃: {type(e).__name__}: {e}")
        print(f"{'=' * 60}")
        traceback.print_exc()
        sys.exit(2)
