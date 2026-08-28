#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
commands.py — 动作命令组件

职责：
  接收 /humanoid/commands/data 命令，执行机器人动作：
    - tts           TTS 语音播报
    - offset_move   末端执行器相对移动（单位：毫米）
    - grab          控制左右夹爪开合
    - cam_head      拍摄头部彩深图并发送给 YOLO 检测服务
    - go            导航到指定地图点位
    - go_rel        底盘相对运动

  每条命令执行完成后，向 /humanoid/commands/done 发布 {"command": "done"}

消息格式（/humanoid/commands/data，订阅）：
  {"command": "tts", "data": "你好"}
  {"command": "offset_move", "data": {"lx": 20, "ly": 0, "lz": 0, "rx": 0, "ry": 0, "rz": 0}}
  {"command": "grab", "data": {"left": 0.5, "right": 0.5}}
  {"command": "go", "data": 9}
  {"command": "go_rel", "data": {"x": 1, "y": 1, "yaw_rad": 0.1}}
"""

import time
import json

import agibot_gdk

import common


# ═══════════════════════════════════════════════════════════
#  TTS 语音播报
# ═══════════════════════════════════════════════════════════

def handle_tts(data, msg=None):
    """TTS 语音播报

    data: 要播报的文本字符串
    """
    text = data
    print(f"[TTS] {text}")
    try:
        common.interaction.play_tts(text)
        print("  TTS 播放成功")
        time.sleep(1)
    except Exception as e:
        print(f"  TTS 播放失败: {e}")


# ═══════════════════════════════════════════════════════════
#  末端相对移动
# ═══════════════════════════════════════════════════════════

def handle_offset_move(data, msg=None):
    """末端执行器相对移动

    data 格式: {"lx": 20, "ly": 0, "lz": 0, "rx": 0, "ry": 0, "rz": 0}
    数值单位：毫米，内部转换为米
    """
    # 毫米 → 米
    offset_l = (
        data.get("lx", 0.0) / 1000.0,
        data.get("ly", 0.0) / 1000.0,
        data.get("lz", 0.0) / 1000.0,
    )
    offset_r = (
        data.get("rx", 0.0) / 1000.0,
        data.get("ry", 0.0) / 1000.0,
        data.get("rz", 0.0) / 1000.0,
    )
    print(f"  左臂偏移 (mm): lx={data.get('lx', 0)}, ly={data.get('ly', 0)}, lz={data.get('lz', 0)}")
    print(f"  右臂偏移 (mm): rx={data.get('rx', 0)}, ry={data.get('ry', 0)}, rz={data.get('rz', 0)}")
    print(f"  换算 (m): L={offset_l}, R={offset_r}")
    try:
        common.ee_controller.adjust_arms_relative(offset_l=offset_l, offset_r=offset_r)
        print("  末端移动完成")
    except Exception as e:
        print(f"  末端移动失败: {e}")


# ═══════════════════════════════════════════════════════════
#  夹爪控制
# ═══════════════════════════════════════════════════════════

def handle_grab(data, msg=None):
    """控制夹爪开合

    data 格式: {"left": -0.7, "right": -0.0}
    只操作传入的手，未传入的不操作
    负值=张开，正值=闭合
    """
    has_left = "left" in data
    has_right = "right" in data
    left_pos = data.get("left", 0.0)
    right_pos = data.get("right", 0.0)

    if has_left:
        print(f"  左夹爪 position={left_pos}")
    if has_right:
        print(f"  右夹爪 position={right_pos}")

    # 右夹爪
    if has_right:
        _control_gripper("right", right_pos)

    # 左夹爪
    if has_left:
        _control_gripper("left", left_pos)


def _control_gripper(side, position):
    """控制单侧夹爪

    Parameters
    ----------
    side : str
        "left" 或 "right"
    position : float
        夹爪位置
    """
    joint_states = agibot_gdk.JointStates()
    joint_states.group = f"{side}_tool"
    joint_states.target_type = "omnipicker"
    joint_state = agibot_gdk.JointState()
    joint_state.position = position
    joint_states.states = [joint_state]
    joint_states.nums = 1
    try:
        common.robot.move_ee_pos(joint_states)
        print(f"  {side} 夹爪控制成功")
        time.sleep(0.02)
    except Exception as e:
        print(f"  {side} 夹爪控制失败: {e}")


# ═══════════════════════════════════════════════════════════
#  头部相机检测（旧 cam_head 命令）
# ═══════════════════════════════════════════════════════════

def handle_cam_head(data, msg=None):
    """拍摄头部彩色+深度相机，通过 TCP 发送给检测服务

    data 可选指定 model 名称
    """
    # 委托给 camera 组件的 YOLO 检测逻辑
    import camera
    model_name = data if isinstance(data, str) else "shelf.pt"
    camera.run_yolo_detect(model_name)


# ═══════════════════════════════════════════════════════════
#  底盘导航
# ═══════════════════════════════════════════════════════════

def handle_go(data, msg=None):
    """导航到指定地图点位

    data 格式：整数导航点索引，如 9
    """
    try:
        waypoint = int(data)
    except (TypeError, ValueError):
        print(f"  [导航] 无效的导航点: {data}")
        return
    print(f"  [导航] 导航到点位 {waypoint}")
    try:
        if not common.nav.go(waypoint):
            print(f"  [导航] 导航到点位 {waypoint} 失败")
        else:
            print(f"  [导航] 已到达点位 {waypoint}")
    except Exception as e:
        print(f"  [导航] 异常: {e}")


def handle_go_rel(data, msg=None):
    """底盘相对运动

    data 格式: {"x": 1, "y": 1, "yaw_rad": 0.1}
      x: 前进(+)/后退(-)，单位米
      y: 左(+)/右(-)，单位米
      yaw_rad: 左转(+)/右转(-)，单位弧度
    """
    dx = float(data.get("x", 0.0))
    dy = float(data.get("y", 0.0))
    dz = float(data.get("z", 0.0))
    yaw_rad = float(data.get("yaw_rad", 0.0))
    print(f"  [底盘] 相对运动: dx={dx}, dy={dy}, dz={dz}, yaw={yaw_rad}")
    try:
        if not common.nav.go_rel(dx=dx, dy=dy, dz=dz, yaw_rad=yaw_rad):
            print(f"  [底盘] 相对运动失败")
        else:
            print(f"  [底盘] 相对运动完成")
    except Exception as e:
        print(f"  [底盘] 异常: {e}")


# ═══════════════════════════════════════════════════════════
#  命令分发表
# ═══════════════════════════════════════════════════════════

# 动作命令处理器映射
COMMAND_HANDLERS = {
    "tts":         handle_tts,
    "offset_move": handle_offset_move,
    "grab":        handle_grab,
    "cam_head":    handle_cam_head,
    "go":          handle_go,
    "go_rel":      handle_go_rel,
}


def handle_control(payload):
    """处理 /humanoid/commands/data 命令

    Parameters
    ----------
    payload : dict
        命令消息，如 {"command": "tts", "data": "你好"}
    """
    cmd = payload.get("command")
    data = payload.get("data")

    handler = COMMAND_HANDLERS.get(cmd)
    if handler is None:
        print(f"[命令] 未知命令: {cmd}，支持: {list(COMMAND_HANDLERS.keys())}")
        return

    # 状态检查：busy 时拒绝新命令
    if common.get_state() == "busy":
        print(f"[命令] 有命令正在执行，拒绝: {cmd}")
        return

    common.set_state("busy")
    try:
        handler(data, payload)
    except Exception as e:
        print(f"[命令] 执行异常: {e}")
    finally:
        common.set_state("idle")
        common.publish_done(cmd)
