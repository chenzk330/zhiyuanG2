#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
map.py — 地图管理与SLAM建图组件

职责：
  - 读取所有地图点位（地图引导点 + 本地保存的点位），发布到 /humanoid/map/points
  - 接收 /humanoid/map/control 命令：
      read_points      读取并发布所有地图点位
      save_point       保存当前底盘位姿为地图点位
      start_mapping    开始SLAM建图
      stop_mapping     结束建图（保存地图）
      read_maps        读取并发布地图列表
      switch_map       切换到指定地图

消息格式（/humanoid/map/points，发布）：
  {"command": "map_points", "data": [{"name": "A", "source": "map", "position": [...], "orientation": [...]}]}

消息格式（/humanoid/map/info，发布）：
  {"command": "maps", "data": [{"id": "xxx", "name": "xxx", "is_current": true}]}
  {"command": "slam_state", "data": {"state": "mapping/idle", "is_mapping": true/false}}

消息格式（/humanoid/map/control，订阅）：
  {"command": "read_points"}
  {"command": "save_point", "data": {"name": "point_name"}}
  {"command": "start_mapping"}
  {"command": "stop_mapping"}
  {"command": "read_maps"}
  {"command": "switch_map", "data": {"map_id": "xxx"}}
"""

import os
import json

import common
import data as db

# 本地跟踪建图状态（GDK接口没有直接暴露is_mapping布尔值）
_is_mapping = False


# ═══════════════════════════════════════════════════════════
#  地图点位读取
# ═══════════════════════════════════════════════════════════

def read_all_map_points():
    """读取所有地图点位（地图引导点 + 数据库中的本地点位）

    Returns
    -------
    list[dict]
        点位列表，每项包含 name / source / position / orientation
    """
    points = []

    # 1. 从地图读取引导点
    try:
        for name, wp in common.nav.waypoints.items():
            pos = wp.get("position", [0, 0, 0])
            ori = wp.get("orientation", [0, 0, 0, 1])
            points.append({
                "name": name,
                "source": wp.get("source", "map"),
                "position": pos,
                "orientation": ori,
            })
    except Exception as e:
        print(f"  [地图] 读取地图引导点失败: {e}")

    # 2. 从数据库读取本地保存的点位
    for pt in db.get_map_points(source="local"):
        points.append(pt)

    return points


def handle_read_points():
    """读取所有地图点位并发布到 /humanoid/map/points"""
    points = read_all_map_points()
    resp = {"command": "map_points", "data": points}
    common.publish(common.TOPIC_MAP_POINTS, resp, qos=0)
    print(f"  [地图] 已发布 {len(points)} 个地图点位到 {common.TOPIC_MAP_POINTS}")


# ═══════════════════════════════════════════════════════════
#  地图点位保存
# ═══════════════════════════════════════════════════════════

def handle_save_point(data):
    """保存当前底盘位姿为地图点位到数据库

    data: {"name": "point_name"}
    """
    save_name = data.get("name", "unnamed") if isinstance(data, dict) else "unnamed"

    # 获取当前底盘位姿
    pose = common.nav.get_current_pose()
    pos = pose.get("position", [0, 0, 0])
    ori = pose.get("orientation", [0, 0, 0, 1])

    position = [round(pos[0], 6), round(pos[1], 6), round(pos[2], 6)]
    orientation = [round(ori[0], 6), round(ori[1], 6), round(ori[2], 6), round(ori[3], 6)]

    db.save_map_point(save_name, position, orientation, source="local")
    print(f"  [地图] 地图点位已保存到数据库: {save_name}")


# ═══════════════════════════════════════════════════════════
#  SLAM 建图控制
# ═══════════════════════════════════════════════════════════

def handle_start_mapping():
    """开始SLAM建图"""
    global _is_mapping
    try:
        common.slam.start_mapping()
        _is_mapping = True
        print("  [SLAM] 开始建图")
        publish_slam_state()
    except Exception as e:
        print(f"  [SLAM] 开始建图失败: {e}")


def handle_stop_mapping():
    """结束SLAM建图（保存地图）"""
    global _is_mapping
    try:
        common.slam.stop_mapping()
        _is_mapping = False
        print("  [SLAM] 结束建图，地图已保存")
        publish_slam_state()
        publish_maps_list()
    except Exception as e:
        print(f"  [SLAM] 结束建图失败: {e}")


def handle_save_map():
    """保存当前地图（调用 stop_mapping 完成保存）"""
    global _is_mapping
    try:
        common.slam.stop_mapping()
        _is_mapping = False
        print("  [SLAM] 地图已保存")
        publish_slam_state()
        publish_maps_list()
    except Exception as e:
        print(f"  [SLAM] 保存地图失败: {e}")


def publish_slam_state():
    """发布当前SLAM状态到 /humanoid/map/info"""
    resp = {"command": "slam_state", "data": {"is_mapping": _is_mapping}}
    common.publish(common.TOPIC_MAP_INFO, resp, qos=0)


# ═══════════════════════════════════════════════════════════
#  地图列表管理
# ═══════════════════════════════════════════════════════════

def publish_maps_list():
    """读取并发布所有地图列表到 /humanoid/map/info"""
    try:
        all_maps = common.gmap.get_all_map()
        maps_data = []
        for m in all_maps:
            mid = m.id
            mname = m.name
            is_curr = m.is_curr_map
            maps_data.append({"id": mid, "name": mname, "is_current": is_curr})
        resp = {"command": "maps", "data": maps_data}
        common.publish(common.TOPIC_MAP_INFO, resp, qos=0)
        print(f"  [地图] 已发布 {len(maps_data)} 个地图")
    except Exception as e:
        print(f"  [地图] 读取地图列表失败: {e}")


def handle_read_maps():
    """处理读取地图列表命令"""
    publish_maps_list()
    publish_slam_state()


def handle_switch_map(map_id):
    """切换到指定地图"""
    try:
        mid = int(map_id)
        common.gmap.switch_map(mid)
        print(f"  [地图] 已切换到地图: {mid}")
        common.nav.list_waypoints()
        publish_maps_list()
    except Exception as e:
        print(f"  [地图] 切换地图失败: {e}")


# ═══════════════════════════════════════════════════════════
#  命令处理
# ═══════════════════════════════════════════════════════════

def handle_control(payload):
    """处理 /humanoid/map/control 命令

    Parameters
    ----------
    payload : dict
        命令消息，如 {"command": "read_points"}
    """
    cmd = payload.get("command")
    data = payload.get("data")

    if cmd in ("read_points", "read_map_points"):
        handle_read_points()
    elif cmd in ("save_point", "save_map_point"):
        handle_save_point(data)
    elif cmd == "start_mapping":
        handle_start_mapping()
    elif cmd == "stop_mapping":
        handle_stop_mapping()
    elif cmd == "save_map":
        handle_save_map()
    elif cmd == "read_maps":
        handle_read_maps()
    elif cmd == "switch_map":
        map_id = data.get("map_id") if isinstance(data, dict) else data
        if map_id:
            handle_switch_map(map_id)
    else:
        print(f"[地图] 未知命令: {cmd}")
