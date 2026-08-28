#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
common.py — 共享基础设施

提供：
  - GDK 全局对象（robot / interaction / camera / slam / lidar）的初始化与释放
  - MQTT 客户端封装（统一连接、订阅、发布）
  - 全局状态管理（idle / busy）
  - 统一的目录路径常量

所有组件模块（camera.py / joints.py / status.py / commands.py / map.py / programs.py）
都通过本模块共享同一份 GDK 对象和 MQTT 客户端，避免重复初始化。
"""

import os
import sys
import time
import json
import threading

import agibot_gdk
import paho.mqtt.client as mqtt

# ── 路径常量 ───────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)

# 数据目录
DATAS_DIR = os.path.join(PROJECT_DIR, "datas")
JOINTS_DIR = os.path.join(DATAS_DIR, "joints")
POSITIONS_DIR = os.path.join(DATAS_DIR, "positions")
MAP_POINTS_DIR = os.path.join(DATAS_DIR, "map_points")
IMAGE_SAVE_DIR = os.path.join(PROJECT_DIR, "images")
DETECT_SAVE_DIR = os.path.join(PROJECT_DIR, "detect")

# runtime 程序目录
RUNTIME_DIR = os.path.join(PROJECT_DIR, "runtime")
PROGRAMS_DIR = os.path.join(RUNTIME_DIR, "programs")
MAIN_PY = os.path.join(RUNTIME_DIR, "main.py")

# ── MQTT 主题结构（遵循 SKILL.md 规则）─────────────────────
# 4 个主分组 + 2 个扩展分组（map / programs）
TOPIC_CAMERA_DATA    = "/humanoid/camera/data"       # 服务端发布相机帧
TOPIC_CAMERA_CONTROL = "/humanoid/camera/control"    # 客户端控制相机

TOPIC_JOINTS_DATA    = "/humanoid/joints/data"       # 服务端发布关节数据列表
TOPIC_JOINTS_CONTROL = "/humanoid/joints/control"    # 客户端控制关节运动
TOPIC_JOINTS_SAVE    = "/humanoid/joints/save"       # 客户端保存关节/位姿

TOPIC_STATUS_DATA    = "/humanoid/status/data"       # 服务端发布机器人状态
TOPIC_STATUS_CONTROL = "/humanoid/status/control"    # 客户端控制状态/点云
TOPIC_STATUS_CLOUD   = "/humanoid/status/cloud"      # 服务端发布点云

TOPIC_COMMANDS_DATA  = "/humanoid/commands/data"     # 客户端发送动作命令
TOPIC_COMMANDS_DONE  = "/humanoid/commands/done"     # 服务端发布完成通知

TOPIC_MAP_POINTS     = "/humanoid/map/points"        # 服务端发布地图点位
TOPIC_MAP_CONTROL    = "/humanoid/map/control"       # 客户端控制地图点位
TOPIC_MAP_INFO       = "/humanoid/map/info"          # 服务端发布地图列表/SLAM状态

TOPIC_PROGRAMS_CONTROL = "/humanoid/programs/control"  # 客户端控制程序调试
TOPIC_PROGRAMS_STEP    = "/humanoid/programs/step"     # 服务端发布执行步骤
TOPIC_PROGRAMS_CODES   = "/humanoid/programs/codes"    # 服务端发布 main.py 代码内容
TOPIC_PROGRAMS_FILES   = "/humanoid/programs/files"    # 服务端发布文件列表
TOPIC_PROGRAMS_FILE_CONTENT = "/humanoid/programs/file_content"  # 服务端发布指定文件内容
TOPIC_PROGRAMS_UPLOAD_RESULT = "/humanoid/programs/upload_result"  # 服务端发布上传结果
TOPIC_PROGRAMS_DELETE_RESULT = "/humanoid/programs/delete_result"  # 服务端发布删除结果

TOPIC_MODBUS_DATA      = "/humanoid/modbus/data"       # 服务端发布 Modbus 数据
TOPIC_MODBUS_CONTROL   = "/humanoid/modbus/control"    # 客户端控制 Modbus 读写

# ── MQTT 连接配置 ─────────────────────────────────────────
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_CLIENT_ID = "humanoid_server"

# ── GDK 全局对象 ──────────────────────────────────────────
robot = None
interaction = None
camera = None
slam = None
gmap = None               # GDK 地图管理对象
lidar = None
ee_controller = None      # 末端执行器控制器
nav = None                # 底盘导航控制器

_gdk_ready = False


def init_gdk():
    """初始化 GDK 并创建所有全局对象。

    依次创建：Robot / Interaction / Camera / Slam / Lidar，
    以及基于 Robot 的末端控制器和底盘控制器。
    """
    global robot, interaction, camera, slam, gmap, lidar, ee_controller, nav, _gdk_ready

    if _gdk_ready:
        return

    # 让组件模块能找到 chassis_controller / offset_move_common
    if SCRIPT_DIR not in sys.path:
        sys.path.insert(0, SCRIPT_DIR)

    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("[GDK] 初始化失败")
        sys.exit(1)
    print("[GDK] 初始化成功")

    robot = agibot_gdk.Robot()
    interaction = agibot_gdk.Interaction()
    camera = agibot_gdk.Camera()
    slam = agibot_gdk.Slam()
    gmap = agibot_gdk.Map()
    lidar = agibot_gdk.Lidar()
    time.sleep(2)  # 等待 DDS 连接就绪

    # 末端执行器相对移动控制器
    from offset_move_common import EndEffectorController
    ee_controller = EndEffectorController(robot)

    # 底盘导航控制器（内部会读取地图导航点）
    from chassis_controller import RobotController
    nav = RobotController()
    nav.list_waypoints()

    _gdk_ready = True
    print("[GDK] 全局对象创建完成: Robot/Interaction/Camera/Slam/Map/Lidar/EE/Nav")


def release_gdk():
    """释放 GDK 资源"""
    global _gdk_ready
    if not _gdk_ready:
        return
    if camera is not None:
        try:
            camera.close_camera()
        except Exception:
            pass
    if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
        print("[GDK] 释放失败")
    else:
        print("[GDK] 释放成功")
    _gdk_ready = False


# ── 状态管理 ───────────────────────────────────────────────
# 任意时刻只能执行一个命令，执行期间 state="busy"，新命令将被拒绝
_state = "idle"
_state_lock = threading.Lock()


def get_state():
    with _state_lock:
        return _state


def set_state(new_state):
    global _state
    with _state_lock:
        _state = new_state


# ── MQTT 客户端封装 ───────────────────────────────────────
mqtt_client = None


def setup_mqtt(on_connect_cb, on_message_cb):
    """创建并连接 MQTT 客户端。

    Parameters
    ----------
    on_connect_cb : callable(client, userdata, flags, rc, properties)
        连接成功回调，通常在此订阅各主题
    on_message_cb : callable(client, userdata, msg)
        消息到达回调，由 main.py 统一分发
    """
    global mqtt_client

    mqtt_client = mqtt.Client(
        client_id=MQTT_CLIENT_ID,
        callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
    )
    mqtt_client.on_connect = on_connect_cb
    mqtt_client.on_message = on_message_cb
    mqtt_client.reconnect_delay_set(min_delay=2, max_delay=30)
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    print(f"[MQTT] 已连接 {MQTT_BROKER}:{MQTT_PORT}")


def publish(topic, payload, qos=0):
    """发布 JSON 消息到指定主题"""
    if mqtt_client is None:
        return
    try:
        if not isinstance(payload, str):
            payload = json.dumps(payload, ensure_ascii=False)
        mqtt_client.publish(topic, payload, qos=qos)
    except Exception as e:
        print(f"[MQTT] 发布失败 {topic}: {e}")


def publish_done(command=""):
    """命令执行完成后向 /humanoid/commands/done 发布完成通知"""
    publish(TOPIC_COMMANDS_DONE, {"command": "done", "cmd": command}, qos=2)
