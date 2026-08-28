#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
status.py — 状态组件

职责：
  - 持续发布机器人状态到 /humanoid/status/data
    状态包含：关节角、左右末端位姿、底盘位姿
  - 按需发布激光雷达点云到 /humanoid/status/cloud
  - 接收 /humanoid/status/control 控制命令：
      start_cloud    开始发布点云
      stop_cloud     停止发布点云

消息格式（/humanoid/status/data，发布）：
{
  "timestamp": "2026-07-14 15:00:00",
  "joints": {"idx01_body_joint1": 0.123, ...},
  "left_ee": {"position": [x,y,z], "orientation": [x,y,z,w]},
  "right_ee": {"position": [x,y,z], "orientation": [x,y,z,w]},
  "chassis": {"x": 0.0, "y": 0.0, "yaw": 0.0, "loc_state": 2, "loc_confidence": 0.95}
}
"""

import os
import time
import json
import math
import threading

import numpy as np
import agibot_gdk

import common

# ── 配置 ───────────────────────────────────────────────────
LEFT_NAME = "arm_l_end_link"
RIGHT_NAME = "arm_r_end_link"

# 发布周期（秒）
STATUS_INTERVAL = 0.5
CLOUD_INTERVAL = 1.0

# 点云降采样
DOWNSAMPLE_STEP = 4
MAX_DISTANCE = 30.0

LIDAR_TYPES = [
    (agibot_gdk.LidarType.kLidarFront, "前部雷达"),
    (agibot_gdk.LidarType.kLidarBack,  "后部雷达"),
]

# 点云发布开关
_cloud_enabled = False
_cloud_lock = threading.Lock()


def is_cloud_enabled():
    with _cloud_lock:
        return _cloud_enabled


def set_cloud_enabled(flag):
    global _cloud_enabled
    with _cloud_lock:
        _cloud_enabled = flag


# ═══════════════════════════════════════════════════════════
#  状态读取
# ═══════════════════════════════════════════════════════════

def read_joint_states(robot):
    """读取所有关节状态，返回 {关节名: 位置} 字典"""
    joint_states = robot.get_joint_states()
    joints = {}
    for state in joint_states['states']:
        joints[state['name']] = round(state['motor_position'], 6)
    return joints


def find_pose_by_name(status, target_name):
    """从 motion_control_status 中按名称查找末端位姿"""
    for i, frame_name in enumerate(status.frame_names):
        if frame_name == target_name:
            pose = status.frame_poses[i]
            return {
                "position": [
                    round(pose.position.x, 6),
                    round(pose.position.y, 6),
                    round(pose.position.z, 6),
                ],
                "orientation": [
                    round(pose.orientation.x, 6),
                    round(pose.orientation.y, 6),
                    round(pose.orientation.z, 6),
                    round(pose.orientation.w, 6),
                ],
            }
    return None


def read_end_effector_poses(robot):
    """读取左右手末端坐标"""
    status = robot.get_motion_control_status()
    left = find_pose_by_name(status, LEFT_NAME)
    right = find_pose_by_name(status, RIGHT_NAME)
    return left, right


def read_chassis_pose(slam):
    """读取底盘在地图中的位姿（X, Y, 旋转角）"""
    try:
        odom = slam.get_odom_info()
        pos = odom.pose.pose.position
        ori = odom.pose.pose.orientation
        # 四元数转 yaw（绕 Z 轴旋转角）
        w, x, y, z = ori.w, ori.x, ori.y, ori.z
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return {
            "x": round(pos.x, 6),
            "y": round(pos.y, 6),
            "z": round(pos.z, 6),
            "yaw": round(yaw, 6),
            "loc_state": odom.loc_state,
            "loc_confidence": round(odom.loc_confidence, 4),
        }
    except Exception as e:
        print(f"[状态] 读取底盘位姿失败: {e}")
        return None


def build_status_message():
    """构建状态 JSON 消息"""
    joints = read_joint_states(common.robot)
    left_ee, right_ee = read_end_effector_poses(common.robot)
    chassis = read_chassis_pose(common.slam)
    return {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "joints": joints,
        "left_ee": left_ee,
        "right_ee": right_ee,
        "chassis": chassis,
    }


# ═══════════════════════════════════════════════════════════
#  点云读取
# ═══════════════════════════════════════════════════════════

def parse_pointcloud(pointcloud):
    """解析 PointCloud 为 (N, 4) numpy 数组 [x, y, z, intensity]"""
    if not hasattr(pointcloud, 'data'):
        return None
    try:
        if isinstance(pointcloud.data, np.ndarray):
            data = pointcloud.data.astype(np.uint8)
        else:
            data = np.frombuffer(pointcloud.data, dtype=np.uint8)

        if pointcloud.point_step <= 0:
            return None

        num_points = len(data) // pointcloud.point_step
        data = data[:num_points * pointcloud.point_step]
        data = data.reshape((num_points, pointcloud.point_step))

        channels = {}
        for field in pointcloud.fields:
            if field.name in ('x', 'y', 'z', 'intensity'):
                slc = data[:, field.offset:field.offset + 4]
                channels[field.name] = np.ascontiguousarray(slc).view(np.float32)

        if 'x' in channels and 'y' in channels and 'z' in channels:
            xs, ys, zs = channels['x'], channels['y'], channels['z']
            intens = channels.get('intensity', np.zeros(num_points, dtype=np.float32))
            return np.column_stack([xs, ys, zs, intens])
        return None
    except Exception as e:
        print(f"[点云] 解析失败: {e}")
        return None


def build_cloud_message():
    """读取前后雷达点云，合并降采样后构建 MQTT 消息"""
    all_points = []
    front_count = 0
    back_count = 0
    latest_ts = 0

    for lidar_type, lidar_name in LIDAR_TYPES:
        pointcloud = common.lidar.get_latest_pointcloud(lidar_type, 1000.0)
        if pointcloud is None:
            continue

        pts = parse_pointcloud(pointcloud)
        if pts is None or len(pts) == 0:
            continue

        dist = np.sqrt(pts[:, 0] ** 2 + pts[:, 1] ** 2 + pts[:, 2] ** 2)
        mask = dist < MAX_DISTANCE
        pts = pts[mask]
        pts = pts[::DOWNSAMPLE_STEP]

        count = len(pts)
        if lidar_type == agibot_gdk.LidarType.kLidarFront:
            front_count = count
        else:
            back_count = count

        for i in range(count):
            all_points.append([
                round(float(pts[i, 0]), 3),
                round(float(pts[i, 1]), 3),
                round(float(pts[i, 2]), 3),
            ])

        if pointcloud.timestamp_ns > latest_ts:
            latest_ts = pointcloud.timestamp_ns

    if not all_points:
        return None

    return {
        "timestamp": latest_ts,
        "count": len(all_points),
        "front_count": front_count,
        "back_count": back_count,
        "points": all_points,
    }


# ═══════════════════════════════════════════════════════════
#  发布循环
# ═══════════════════════════════════════════════════════════

def publishing_loop():
    """状态+点云发布主循环（在独立线程中运行）"""
    print("[状态] 发布线程已启动")
    last_cloud_time = 0.0

    while True:
        try:
            now = time.time()

            # 发布状态
            try:
                msg = build_status_message()
                common.publish(common.TOPIC_STATUS_DATA, msg, qos=0)
                chassis_str = ""
                if msg.get("chassis"):
                    c = msg["chassis"]
                    chassis_str = f", chassis=({c['x']:.2f},{c['y']:.2f},{c['yaw']:.2f})"
                # print(f"[状态] joints={len(msg['joints'])}个{chassis_str}")
            except Exception as e:
                print(f"[状态] 发布失败: {e}")

            # 发布点云（仅在启用时）
            if is_cloud_enabled() and (now - last_cloud_time) >= CLOUD_INTERVAL:
                try:
                    cloud_msg = build_cloud_message()
                    if cloud_msg:
                        common.publish(common.TOPIC_STATUS_CLOUD, cloud_msg, qos=0)
                        print(f"[点云] 点数={cloud_msg['count']}, "
                              f"前={cloud_msg['front_count']}, 后={cloud_msg['back_count']}")
                    else:
                        print("[点云] 未获取到数据")
                except Exception as e:
                    print(f"[点云] 发布失败: {e}")
                last_cloud_time = now

            time.sleep(STATUS_INTERVAL)
        except Exception as e:
            print(f"[状态] 循环异常: {e}")
            time.sleep(1.0)


# ═══════════════════════════════════════════════════════════
#  命令处理
# ═══════════════════════════════════════════════════════════

def handle_control(payload):
    """处理 /humanoid/status/control 命令（点云开关）

    Parameters
    ----------
    payload : dict
        命令消息，如 {"command": "start_cloud"}
    """
    cmd = payload.get("command", "").lower()

    if cmd == "start_cloud":
        set_cloud_enabled(True)
        print("[状态] 开始发布点云")
    elif cmd == "stop_cloud":
        set_cloud_enabled(False)
        print("[状态] 停止发布点云")
    else:
        print(f"[状态] 未知命令: {cmd}")


def start_publishing_thread():
    """启动状态发布线程（由 main.py 在初始化时调用）"""
    t = threading.Thread(target=publishing_loop, daemon=True)
    t.start()
    print("[状态] 发布线程已启动")
