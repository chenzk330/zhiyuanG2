#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
joints.py — 关节组件

职责：
  1. 关节运动控制（接收 /humanoid/joints/control 命令）
     - WBC / arms / left / right / head / waist：从 datas/joints/{type}/{name}.json
       加载关节角或使用内联关节角，控制对应身体部位运动
     - joint：单关节增量微调或运动到指定角度

  2. 数据持久化（接收 /humanoid/joints/save 命令）
     - save_joints：保存关节角到 datas/joints/{type}/{name}.json
     - save_position：保存末端位姿到 datas/positions/{type}/{name}.json

  3. 数据读取（通过 /humanoid/joints/data 发布）
     - read：扫描 datas/ 目录，发布所有关节数据和位姿数据列表
     - update / delete：更新或删除指定数据文件

消息格式（/humanoid/joints/control，订阅）：
  {"command": "WBC", "data": "hold"}                       # 加载 datas/joints/WBC/hold.json
  {"command": "arms", "data": {"idx21_arm_l_joint1": 0.1}} # 内联关节角
  {"command": "joint", "data": {"name": "idx11_head_joint1", "offset": 0.01}}

消息格式（/humanoid/joints/save，订阅）：
  {"command": "save_joints", "type": "WBC", "name": "hold", "data": {...}}
  {"command": "save_position", "type": "left", "name": "pick", "data": {...}}
  {"command": "read"}
  {"command": "update", "category": "joints", "type": "WBC", "name": "hold", "data": {...}}
  {"command": "delete", "category": "joints", "type": "WBC", "name": "hold"}
"""

import os
import json
import time

import agibot_gdk

import common
import data as db

# ── 关节名分组 ─────────────────────────────────────────────
HEAD_JOINT_KEYS = [
    "idx11_head_joint1", "idx12_head_joint2", "idx13_head_joint3",
]
WAIST_JOINT_KEYS = [
    "idx01_body_joint1", "idx02_body_joint2", "idx03_body_joint3",
    "idx04_body_joint4", "idx05_body_joint5",
]
LEFT_ARM_JOINT_KEYS = [
    "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3",
    "idx24_arm_l_joint4", "idx25_arm_l_joint5", "idx26_arm_l_joint6",
    "idx27_arm_l_joint7",
]
RIGHT_ARM_JOINT_KEYS = [
    "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3",
    "idx64_arm_r_joint4", "idx65_arm_r_joint5", "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]

# 运动速度
HEAD_SPEED = 0.3
WAIST_SPEED = 0.3
ARM_SPEED = 0.2

# 关节类型目录（datas/joints/ 下的子目录名）
JOINT_TYPES = ["WBC", "arms", "left", "right", "head", "waist"]
# 位姿类型目录（datas/positions/ 下的子目录名）
POSITION_TYPES = ["left", "right", "both"]


# ═══════════════════════════════════════════════════════════
#  关节运动控制
# ═══════════════════════════════════════════════════════════

def _extract_positions(data, keys):
    """从字典中按 keys 顺序提取关节角，缺失的补 0.0"""
    return [data.get(key, 0.0) for key in keys]


def _load_joints_data(cmd_type, data):
    """加载关节角数据

    data 可以是：
      - 字符串：从数据库加载 {cmd_type}/{data} 的关节角
      - 字典：内联关节角，直接使用

    返回 (pos_data, desc) 或 (None, error_msg)
    """
    if isinstance(data, dict):
        return data, f"内联关节角 ({len(data)} 个关节)"

    rows = db.get_joints(cmd_type, data)
    if not rows:
        return None, f"找不到 joints/{cmd_type}/{data}"
    pos_data = rows[0]["value"]
    return pos_data, f"已加载 joints/{cmd_type}/{data}"


def _move_head(pos_data):
    """控制头部 3 个关节运动"""
    pos = _extract_positions(pos_data, HEAD_JOINT_KEYS)
    vel = [HEAD_SPEED] * len(pos)
    print(f"  头部 → {[f'{p:.3f}' for p in pos]}")
    common.robot.move_head_joint(pos, vel)


def _move_waist(pos_data):
    """控制腰部 5 个关节运动"""
    pos = _extract_positions(pos_data, WAIST_JOINT_KEYS)
    vel = [WAIST_SPEED] * len(pos)
    print(f"  腰部 → {[f'{p:.3f}' for p in pos]}")
    common.robot.move_waist_joint(pos, vel)


def _move_both_arms(pos_data):
    """控制双臂 14 个关节运动（左臂 7 + 右臂 7）"""
    left = _extract_positions(pos_data, LEFT_ARM_JOINT_KEYS)
    right = _extract_positions(pos_data, RIGHT_ARM_JOINT_KEYS)
    positions = left + right
    velocities = [ARM_SPEED] * len(positions)
    print(f"  左臂 → {[f'{p:.3f}' for p in left]}")
    print(f"  右臂 → {[f'{p:.3f}' for p in right]}")
    common.robot.move_arm_joint(positions, velocities, 2)


def _move_left_arm(pos_data):
    """仅控制左臂运动（右臂补 0）"""
    left = _extract_positions(pos_data, LEFT_ARM_JOINT_KEYS)
    right = [0.0] * len(RIGHT_ARM_JOINT_KEYS)
    positions = left + right
    velocities = [ARM_SPEED] * len(positions)
    print(f"  左臂 → {[f'{p:.3f}' for p in left]}")
    common.robot.move_arm_joint(positions, velocities, 2)


def _move_right_arm(pos_data):
    """仅控制右臂运动（左臂补 0）"""
    left = [0.0] * len(LEFT_ARM_JOINT_KEYS)
    right = _extract_positions(pos_data, RIGHT_ARM_JOINT_KEYS)
    positions = left + right
    velocities = [ARM_SPEED] * len(positions)
    print(f"  右臂 → {[f'{p:.3f}' for p in right]}")
    common.robot.move_arm_joint(positions, velocities, 2)


# 各身体部位的执行函数映射
BODY_PART_MOVERS = {
    "head":      _move_head,
    "waist":     _move_waist,
    "arms":      _move_both_arms,
    "left_arm":  _move_left_arm,
    "right_arm": _move_right_arm,
}


def _make_joint_handler(cmd_type, body_parts):
    """创建关节运动处理器

    Parameters
    ----------
    cmd_type : str
        命令类型，对应 datas/joints/ 下的子目录名
    body_parts : list[str]
        要执行的身体部位列表，如 ["head", "waist", "arms"]
    """
    def handler(data, msg=None):
        pos_data, desc = _load_joints_data(cmd_type, data)
        if pos_data is None:
            print(f"[关节] {desc}")
            return
        print(f"[关节] {desc}")

        for part in body_parts:
            mover = BODY_PART_MOVERS.get(part)
            if mover is None:
                continue
            try:
                mover(pos_data)
                print(f"  {part} 控制成功")
            except Exception as e:
                print(f"  {part} 控制失败: {e}")
            time.sleep(0.2)

    return handler


# 关节运动命令分发表
# WBC=全身(head+waist+arms), arms=双臂, left=左臂, right=右臂, head=头部, waist=腰部
JOINT_MOTION_HANDLERS = {
    "WBC":   _make_joint_handler("WBC",   ["head", "waist", "arms"]),
    "arms":  _make_joint_handler("arms",  ["arms"]),
    "left":  _make_joint_handler("left",  ["left_arm"]),
    "right": _make_joint_handler("right", ["right_arm"]),
    "head":  _make_joint_handler("head",  ["head"]),
    "waist": _make_joint_handler("waist", ["waist"]),
}


def handle_joint_single(data, msg=None):
    """单关节控制（增量微调或直接运动到角度）

    data 格式:
      {"name": "idx11_head_joint1", "offset": 0.01}   # 增量微调
      {"name": "idx11_head_joint1", "value": 0.0}     # 运动到指定角度
    """
    if not isinstance(data, dict):
        print(f"[关节] joint 命令 data 非字典: {data}")
        return

    joint_name = data.get("name")
    if not joint_name:
        print("[关节] joint 命令缺少 name 字段")
        return

    # 读取当前关节状态
    joint_states = common.robot.get_joint_states()
    current_angles = {}
    for state in joint_states['states']:
        current_angles[state['name']] = state['motor_position']

    current = current_angles.get(joint_name, 0.0)

    if "value" in data:
        target = float(data["value"])
    elif "offset" in data:
        target = current + float(data["offset"])
    else:
        print("[关节] joint 命令需要 offset 或 value 字段")
        return

    print(f"  {joint_name}: {current:.4f} → {target:.4f}")

    # 根据关节名前缀选择运动接口
    if joint_name.startswith("idx1"):       # head
        keys = [k for k in HEAD_JOINT_KEYS]
        pos = [current_angles.get(k, 0.0) for k in keys]
        idx = keys.index(joint_name) if joint_name in keys else 0
        pos[idx] = target
        common.robot.move_head_joint(pos, [HEAD_SPEED] * len(pos))
    elif joint_name.startswith("idx0"):     # waist
        keys = WAIST_JOINT_KEYS
        pos = [current_angles.get(k, 0.0) for k in keys]
        idx = keys.index(joint_name) if joint_name in keys else 0
        pos[idx] = target
        common.robot.move_waist_joint(pos, [WAIST_SPEED] * len(pos))
    elif joint_name.startswith("idx2"):     # left arm
        keys = LEFT_ARM_JOINT_KEYS
        left = [current_angles.get(k, 0.0) for k in keys]
        idx = keys.index(joint_name) if joint_name in keys else 0
        left[idx] = target
        right = [0.0] * len(RIGHT_ARM_JOINT_KEYS)
        common.robot.move_arm_joint(left + right, [ARM_SPEED] * (len(left) + len(right)), 2)
    elif joint_name.startswith("idx6"):     # right arm
        keys = RIGHT_ARM_JOINT_KEYS
        right = [current_angles.get(k, 0.0) for k in keys]
        idx = keys.index(joint_name) if joint_name in keys else 0
        right[idx] = target
        left = [0.0] * len(LEFT_ARM_JOINT_KEYS)
        common.robot.move_arm_joint(left + right, [ARM_SPEED] * (len(left) + len(right)), 2)
    else:
        print(f"[关节] 未知关节名前缀: {joint_name}")


# ═══════════════════════════════════════════════════════════
#  命令分发
# ═══════════════════════════════════════════════════════════

def handle_control(payload):
    """处理 /humanoid/joints/control 命令（关节运动）

    Parameters
    ----------
    payload : dict
        命令消息，如 {"command": "WBC", "data": "hold"}
    """
    cmd = payload.get("command")
    data = payload.get("data")

    if cmd == "joint":
        handler = handle_joint_single
    else:
        handler = JOINT_MOTION_HANDLERS.get(cmd)

    if handler is None:
        print(f"[关节] 未知命令: {cmd}，支持: {list(JOINT_MOTION_HANDLERS.keys()) + ['joint']}")
        return

    try:
        handler(data, payload)
    except Exception as e:
        print(f"[关节] 命令执行异常: {e}")


def handle_save(payload):
    """处理 /humanoid/joints/save 命令（数据持久化）

    Parameters
    ----------
    payload : dict
        命令消息，如 {"command": "save_joints", "type": "WBC", "name": "hold", "data": {...}}
    """
    cmd = payload.get("command")

    if cmd == "save_joints":
        _save_joints(payload)
    elif cmd == "save_position":
        _save_position(payload)
    elif cmd == "read":
        _handle_read()
    elif cmd == "update":
        _handle_update(payload)
    elif cmd == "delete":
        _handle_delete(payload)
    else:
        print(f"[关节] 未知 save 命令: {cmd}")


# ═══════════════════════════════════════════════════════════
#  数据保存
# ═══════════════════════════════════════════════════════════

def _save_joints(msg):
    """保存关节角到数据库

    msg: {"type": "WBC", "name": "hold", "data": {关节名: 弧度}}
    """
    save_type = msg.get("type", "WBC")
    save_name = msg.get("name", "unnamed")
    joints = msg.get("data", {})
    if not isinstance(joints, dict):
        print(f"  [保存] data 不是字典: {type(joints)}")
        return

    db.save_joints(save_type, save_name, joints)
    print(f"  [保存] 关节角已保存到数据库: {save_type}/{save_name} ({len(joints)} 个关节)")


def _save_position(msg):
    """保存末端位姿到数据库

    msg: {"type": "left"/"right"/"both", "name": "pick",
          "data": {"x":0.1, "y":0.2, "z":0.3, "rx":0, "ry":0, "rz":0}}
    both 时 data = {"left": {...}, "right": {...}}
    """
    save_type = msg.get("type", "both")
    save_name = msg.get("name", "unnamed")
    pos_data = msg.get("data", {})
    if not isinstance(pos_data, dict):
        print(f"  [保存] data 不是字典: {type(pos_data)}")
        return

    db.save_positions(save_type, save_name, pos_data)
    print(f"  [保存] 末端位姿已保存到数据库: {save_type}/{save_name}")


# ═══════════════════════════════════════════════════════════
#  数据读取 / 更新 / 删除
# ═══════════════════════════════════════════════════════════

def _read_all_data():
    """从数据库查询所有关节数据和位姿数据"""
    items = db.get_all_data()
    print(f"  [读取] 共 {len(items)} 条数据")
    return items


def _handle_read():
    """处理 read 命令：从数据库查询并发布数据列表到 /humanoid/joints/data"""
    items = _read_all_data()
    resp = {"command": "response", "data": items}
    common.publish(common.TOPIC_JOINTS_DATA, resp, qos=0)
    print(f"  [读取] 已发布 {len(items)} 条数据到 {common.TOPIC_JOINTS_DATA}")


def _handle_update(msg):
    """处理 update 命令：更新数据库中的指定数据

    msg: {category: joints/positions, type: WBC, name: hold, data: {...}}
    """
    category = msg.get("category", "joints")
    update_type = msg.get("type", "WBC")
    update_name = msg.get("name", "unnamed")
    data = msg.get("data", {})

    db.update_data(category, update_type, update_name, data)
    print(f"  [更新] 已更新 {category}/{update_type}/{update_name}")


def _handle_delete(msg):
    """处理 delete 命令：删除数据库中的指定数据

    msg: {category: joints/positions, type: WBC, name: hold}
    """
    category = msg.get("category", "joints")
    del_type = msg.get("type", "WBC")
    del_name = msg.get("name", "")

    db.delete_data(category, del_type, del_name)
    print(f"  [删除] 已删除 {category}/{del_type}/{del_name}")
