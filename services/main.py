#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py — Humanoid 机器人控制服务（单一入口）

本文件是整个机器人控制服务的唯一入口，启动后会：
  1. 初始化 GDK（创建 Robot / Camera / Slam / Lidar 等全局对象）
  2. 连接 MQTT broker
  3. 订阅所有控制主题
  4. 启动相机流发布线程和状态发布线程
  5. 进入 MQTT 消息循环，分发命令到各组件

组件模块（均由 main.py 导入并调度）：
  - common.py     共享基础设施（GDK 初始化、MQTT 客户端、状态管理）
  - camera.py     相机数据发布与控制（/humanoid/camera/）
  - joints.py     关节运动控制与数据持久化（/humanoid/joints/）
  - status.py     机器人状态与点云发布（/humanoid/status/）
  - commands.py   动作命令处理（tts/grab/go 等，/humanoid/commands/）
  - map.py        地图点位管理（/humanoid/map/）
  - programs.py   runtime 程序调试（/humanoid/programs/）

MQTT 主题结构（遵循 SKILL.md 规则）：
  /humanoid/camera/data         服务端发布相机帧
  /humanoid/camera/control      客户端控制相机（start/stop/save_photo/detect）
  /humanoid/joints/data         服务端发布关节数据列表
  /humanoid/joints/control      客户端控制关节运动（WBC/arms/head/...）
  /humanoid/joints/save         客户端保存关节/位姿数据
  /humanoid/status/data         服务端发布机器人状态
  /humanoid/status/control      客户端控制状态/点云
  /humanoid/status/cloud        服务端发布点云
  /humanoid/commands/data       客户端发送动作命令（tts/grab/go/...）
  /humanoid/commands/done       服务端发布完成通知
  /humanoid/map/points          服务端发布地图点位
  /humanoid/map/control         客户端控制地图点位
  /humanoid/programs/control    客户端控制程序调试
  /humanoid/programs/step       服务端发布执行步骤
  /humanoid/programs/codes      服务端发布代码内容
  /humanoid/programs/files      服务端发布文件列表
"""

import os
import sys
import json

# 确保能导入同目录下的组件模块
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import common
import data as db
import camera
import joints
import status
import commands
import map as map_module
import programs
import modbus


# ═══════════════════════════════════════════════════════════
#  MQTT 回调
# ═══════════════════════════════════════════════════════════

def on_connect(client, userdata, flags, rc, properties=None):
    """连接成功后订阅所有控制主题"""
    if rc == 0:
        print(f"\n[MQTT] 已连接到 {common.MQTT_BROKER}:{common.MQTT_PORT}")
        # 订阅所有控制主题
        client.subscribe(common.TOPIC_CAMERA_CONTROL, qos=0)
        client.subscribe(common.TOPIC_JOINTS_CONTROL, qos=2)
        client.subscribe(common.TOPIC_JOINTS_SAVE, qos=0)
        client.subscribe(common.TOPIC_STATUS_CONTROL, qos=0)
        client.subscribe(common.TOPIC_COMMANDS_DATA, qos=2)
        client.subscribe(common.TOPIC_MAP_CONTROL, qos=0)
        client.subscribe(common.TOPIC_PROGRAMS_CONTROL, qos=0)
        client.subscribe(common.TOPIC_MODBUS_CONTROL, qos=0)
        print("[MQTT] 已订阅所有控制主题:")
        print(f"  - {common.TOPIC_CAMERA_CONTROL}")
        print(f"  - {common.TOPIC_JOINTS_CONTROL}")
        print(f"  - {common.TOPIC_JOINTS_SAVE}")
        print(f"  - {common.TOPIC_STATUS_CONTROL}")
        print(f"  - {common.TOPIC_COMMANDS_DATA}")
        print(f"  - {common.TOPIC_MAP_CONTROL}")
        print(f"  - {common.TOPIC_PROGRAMS_CONTROL}")
        print(f"  - {common.TOPIC_MODBUS_CONTROL}")
        print("-" * 60)
    else:
        print(f"[MQTT] 连接失败，返回码: {rc}")


def on_message(client, userdata, msg):
    """收到 MQTT 消息时按主题分发到对应组件"""
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
    except Exception as e:
        print(f"[MQTT] 解析失败: {e}，原始: {msg.payload}")
        return

    topic = msg.topic

    # 按主题分发到对应组件
    if topic == common.TOPIC_CAMERA_CONTROL:
        # 相机控制命令（start/stop/save_photo/detect）
        camera.handle_control(payload)

    elif topic == common.TOPIC_JOINTS_CONTROL:
        # 关节运动命令（WBC/arms/head/joint 等）—— 需要 busy/idle 状态保护
        cmd = payload.get("command")
        print(f"\n[命令] joints/control: {cmd}")
        if common.get_state() == "busy":
            print(f"[命令] 有命令正在执行，拒绝: {cmd}")
            return
        common.set_state("busy")
        try:
            joints.handle_control(payload)
        except Exception as e:
            print(f"[命令] 执行异常: {e}")
        finally:
            common.set_state("idle")
            common.publish_done(cmd)

    elif topic == common.TOPIC_JOINTS_SAVE:
        # 数据持久化命令（save_joints/save_position/read/update/delete）
        cmd = payload.get("command")
        print(f"\n[命令] joints/save: {cmd}")
        joints.handle_save(payload)

    elif topic == common.TOPIC_STATUS_CONTROL:
        # 状态/点云控制命令（start_cloud/stop_cloud）
        status.handle_control(payload)

    elif topic == common.TOPIC_COMMANDS_DATA:
        # 动作命令（tts/grab/go/go_rel/offset_move/cam_head）
        cmd = payload.get("command")
        print(f"\n[命令] commands/data: {cmd}")
        commands.handle_control(payload)

    elif topic == common.TOPIC_MAP_CONTROL:
        # 地图点位命令（read_points/save_point）
        cmd = payload.get("command")
        print(f"\n[命令] map/control: {cmd}")
        map_module.handle_control(payload)

    elif topic == common.TOPIC_PROGRAMS_CONTROL:
        # 程序调试命令（run/debug/next/stop/copy/codes/read_files）
        cmd = payload.get("command")
        print(f"\n[命令] programs/control: {cmd}")
        programs.handle_control(payload)

    elif topic == common.TOPIC_MODBUS_CONTROL:
        # Modbus 读写命令（read/write）
        modbus.handle_control(payload)

    else:
        print(f"[MQTT] 未识别的主题: {topic}")


# ═══════════════════════════════════════════════════════════
#  主入口
# ═══════════════════════════════════════════════════════════

def main():
    print("#" * 60)
    print("#   Humanoid 机器人控制服务 - 启动   #")
    print("#" * 60)
    print(f"joints 目录    : {common.JOINTS_DIR}")
    print(f"positions 目录 : {common.POSITIONS_DIR}")
    print(f"map_points 目录: {common.MAP_POINTS_DIR}")
    print(f"images 目录    : {common.IMAGE_SAVE_DIR}")
    print(f"detect 目录    : {common.DETECT_SAVE_DIR}")
    print(f"programs 目录  : {common.PROGRAMS_DIR}")
    print()

    # 1. 初始化 GDK（创建 Robot / Camera / Slam / Lidar / EE / Nav）
    common.init_gdk()

    # 2. 初始化内存 SQLite 数据库（从硬盘 JSON 文件导入数据）
    db.init_db()

    # 3. 确保数据目录存在
    os.makedirs(common.DATAS_DIR, exist_ok=True)
    os.makedirs(common.IMAGE_SAVE_DIR, exist_ok=True)
    os.makedirs(common.DETECT_SAVE_DIR, exist_ok=True)
    os.makedirs(common.PROGRAMS_DIR, exist_ok=True)

    # 4. 设置并连接 MQTT
    common.setup_mqtt(on_connect, on_message)

    # 5. 启动相机流发布线程（默认不发布，等待 start 命令）
    camera.start_streaming_thread()

    # 6. 启动状态发布线程（持续发布机器人状态）
    status.start_publishing_thread()

    # 7. 启动 Modbus 轮询线程（按配置 rate 周期读取寄存器）
    modbus.start_polling_thread()

    print("\n[主循环] 服务已就绪，等待命令...")
    print("-" * 60)

    # 6. 进入 MQTT 消息循环
    try:
        common.mqtt_client.loop_forever()
    except KeyboardInterrupt:
        print("\n[退出] 用户中断")
    except Exception as e:
        print(f"[错误] {e}")
    finally:
        try:
            common.mqtt_client.disconnect()
        except Exception:
            pass
        common.release_gdk()
        print("服务已停止")


if __name__ == "__main__":
    main()
