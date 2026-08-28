#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""camera.py — 相机采集模块

功能:
  - capture_color(): 拍摄头部彩色图 (GDK 优先, MQTT 回退)
  - capture_color_and_depth(): 拍摄彩色+深度图 (MQTT save_photo + 文件轮询)
  - capture_color_with_warmup(): 连拍两次丢弃首张 (刷新相机缓冲)
  - cleanup_old_images(): 清理旧图片避免堆积

参考: /home/agi/wzd/chassis_correct_all.py:121-334
"""
import base64
import glob
import json
import os
import sys
import time

import cv2
import numpy as np

# ── 全局状态 ──
_gdk_camera = None
_config = None


def configure(cfg):
    """注入配置 (由 main.py 在启动时调用)

    cfg 应包含 common 段的:
      - mqtt_broker, mqtt_port
      - image_save_dir
      - gdk_services_dir
      - warmup_wait
    """
    global _config
    _config = cfg


def _get(key, default=None):
    """从配置读取值"""
    if _config is None:
        return default
    return _config.get(key, default)


# ═══════════════════════════════════════════════════════════
#  GDK 相机接口初始化
# ═══════════════════════════════════════════════════════════

def _init_gdk():
    """初始化 GDK 相机接口 (延迟初始化, 单例)

    Returns: agibot_gdk.Camera 实例, 失败返回 None
    """
    global _gdk_camera
    if _gdk_camera is not None:
        return _gdk_camera

    gdk_dir = _get("gdk_services_dir")
    if gdk_dir and gdk_dir not in sys.path:
        sys.path.insert(0, gdk_dir)
    try:
        import agibot_gdk
        _gdk_camera = agibot_gdk.Camera()
        print("[GDK] 相机接口初始化成功")
        return _gdk_camera
    except Exception as e:
        print(f"[GDK] 初始化失败: {e}, 将回退到 MQTT 方式")
        return None


# ═══════════════════════════════════════════════════════════
#  彩色图采集
# ═══════════════════════════════════════════════════════════

def capture_color(timeout=10.0):
    """拍摄头部彩色相机图像 (优先 GDK, 回退 MQTT)

    Args:
        timeout: MQTT 模式超时秒数

    Returns:
        bgr 图像 (numpy array, H×W×3)

    Raises:
        RuntimeError: 超时未收到相机数据
    """
    broker = _get("mqtt_broker", "localhost")
    port = _get("mqtt_port", 1883)

    # ── 路径 1: GDK 直拍 ──
    cam = _init_gdk()
    if cam is not None:
        try:
            import agibot_gdk
            img = cam.get_latest_image(agibot_gdk.CameraType.kHeadColor, timeout * 1000.0)
            if img is not None and img.data is not None:
                encoding = getattr(img, "encoding", None)
                color_format = getattr(img, "color_format", None)
                raw = img.data

                # JPEG 编码
                if encoding == agibot_gdk.Encoding.JPEG:
                    nparr = np.frombuffer(raw, dtype=np.uint8)
                    bgr = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    if bgr is not None:
                        return bgr

                # 原始格式 (RGB/BGR/GRAY)
                elif color_format in (agibot_gdk.ColorFormat.RGB,
                                       agibot_gdk.ColorFormat.BGR,
                                       agibot_gdk.ColorFormat.GRAY8):
                    nparr = np.frombuffer(raw, dtype=np.uint8)
                    if color_format == agibot_gdk.ColorFormat.RGB:
                        rgb = nparr.reshape((img.height, img.width, 3))
                        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    elif color_format == agibot_gdk.ColorFormat.BGR:
                        return nparr.reshape((img.height, img.width, 3))
                    else:
                        gray = nparr.reshape((img.height, img.width))
                        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        except Exception as e:
            print(f"[GDK] 拍照失败, 回退 MQTT: {e}")

    # ── 路径 2: MQTT 订阅相机流 ──
    import paho.mqtt.client as mqtt

    received = {"img": None}
    topic_ctrl = "/humanoid/camera/control"
    topic_data = "/humanoid/camera/data"

    def on_connect(client, userdata, flags, rc, properties=None):
        if rc == 0:
            client.subscribe(topic_data, qos=0)
            client.publish(topic_ctrl, json.dumps({"command": "start"}), qos=0)

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
            b64 = payload.get("head_color")
            if b64:
                buf = base64.b64decode(b64)
                nparr = np.frombuffer(buf, dtype=np.uint8)
                bgr = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                if bgr is not None:
                    received["img"] = bgr
                    client.disconnect()
        except Exception:
            pass

    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port, keepalive=60)

    t_start = time.time()
    while received["img"] is None and time.time() - t_start < timeout:
        client.loop(timeout=0.2)
    try:
        client.publish(topic_ctrl, json.dumps({"command": "stop"}), qos=0)
        client.disconnect()
    except Exception:
        pass

    if received["img"] is None:
        raise RuntimeError(f"在 {timeout}s 内未收到相机数据")
    return received["img"]


def capture_color_with_warmup():
    """连拍两次丢弃第一张 (刷新相机缓冲, 防止取到旧图)"""
    warmup = _get("warmup_wait", 0.2)
    capture_color()  # 第一张丢弃
    time.sleep(warmup)
    return capture_color()


# ═══════════════════════════════════════════════════════════
#  彩色 + 深度图采集 (MQTT save_photo + 文件轮询)
# ═══════════════════════════════════════════════════════════

def _read_latest_color_and_depth(existing_color, existing_depth, tried_depth=None):
    """读取最新的彩色图和深度图文件

    Args:
        existing_color: 之前的 color 文件集合
        existing_depth: 之前的 depth 文件集合
        tried_depth: 已尝试过但异常的 depth 文件集合 (跳过)

    Returns:
        (color_bgr, depth_2d): 任一为 None 表示未取到
    """
    if tried_depth is None:
        tried_depth = set()

    image_save_dir = _get("image_save_dir", "/data/wxf/wxf0721/images")
    color_files = sorted(glob.glob(os.path.join(image_save_dir, "kHeadColor_*.jpg")))
    depth_files = sorted(glob.glob(os.path.join(image_save_dir, "kHeadDepth_raw_*.raw")))

    new_color = [f for f in color_files if f not in existing_color]
    new_depth = [f for f in depth_files if f not in existing_depth and f not in tried_depth]

    color_bgr = None
    depth_2d = None

    if new_color:
        color_bgr = cv2.imread(new_color[-1])
    if new_depth:
        raw = np.fromfile(new_depth[-1], dtype=np.uint16)
        # 尺寸校验: 期望 400*640=256000 个 uint16
        if raw.size != 400 * 640:
            print(f"[WARN] 深度图尺寸异常: {raw.size} != {400 * 640}, 跳过: {new_depth[-1]}")
            depth_2d = None
        else:
            depth_2d = raw.reshape((400, 640))

    return color_bgr, depth_2d


def capture_color_and_depth(timeout=15.0):
    """单次拍照获取彩色+深度图

    通过 MQTT 发送 save_photo 命令, 轮询 image_save_dir 文件出现。
    改进: 每3秒重发一次 save_photo 防止丢命令; 深度帧异常时自动请求新帧。

    Returns:
        (color_bgr, depth_2d): 任一为 None 表示拍照失败

    参考: chassis_correct_all.py:275-327
    """
    broker = _get("mqtt_broker", "localhost")
    port = _get("mqtt_port", 1883)
    image_save_dir = _get("image_save_dir", "/data/wxf/wxf0721/images")

    existing_c = set(glob.glob(os.path.join(image_save_dir, "kHeadColor_*.jpg")))
    existing_d = set(glob.glob(os.path.join(image_save_dir, "kHeadDepth_raw_*.raw")))

    import paho.mqtt.client as mqtt
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    try:
        client.connect(broker, port, keepalive=10)
        client.loop_start()
        client.publish("/humanoid/camera/control",
                       json.dumps({"command": "save_photo",
                                   "cameras": ["kHeadColor", "kHeadDepth"]}), qos=0)
    except Exception as e:
        print(f"[相机] MQTT 连接失败: {e}")

    t_start = time.time()
    color_bgr, depth_2d = None, None
    tried_depth = set()
    last_resend = t_start

    while time.time() - t_start < timeout:
        color_bgr, depth_2d = _read_latest_color_and_depth(existing_c, existing_d, tried_depth)
        if color_bgr is not None and depth_2d is not None:
            break

        # depth 异常: 标记最新文件以跳过, 等待新文件
        if depth_2d is None:
            depth_files = sorted(glob.glob(os.path.join(image_save_dir, "kHeadDepth_raw_*.raw")))
            new_d_now = [f for f in depth_files if f not in existing_d and f not in tried_depth]
            if new_d_now:
                tried_depth.add(new_d_now[-1])

        # 每3秒重发一次 save_photo (防止丢命令或请求新帧替代异常帧)
        now = time.time()
        if now - last_resend >= 3.0:
            try:
                client.publish("/humanoid/camera/control",
                               json.dumps({"command": "save_photo",
                                           "cameras": ["kHeadColor", "kHeadDepth"]}), qos=0)
            except Exception:
                pass
            last_resend = now
        time.sleep(0.1)  # 100ms 轮询

    try:
        client.loop_stop()
        client.disconnect()
    except Exception:
        pass

    return color_bgr, depth_2d


# ═══════════════════════════════════════════════════════════
#  旧图清理
# ═══════════════════════════════════════════════════════════

def cleanup_old_images(keep_n=20):
    """清理旧图片避免堆积 (保留最近 N 个)

    每次纠偏循环都会生成 kHeadColor_*.jpg 和 kHeadDepth_raw_*.raw 文件,
    不清理会导致"读取最新文件"逻辑受旧文件干扰, 且占用磁盘。
    """
    image_save_dir = _get("image_save_dir", "/data/wxf/wxf0721/images")
    if not os.path.isdir(image_save_dir):
        return

    for pattern in ["kHeadColor_*.jpg", "kHeadDepth_raw_*.raw"]:
        files = sorted(glob.glob(os.path.join(image_save_dir, pattern)))
        if len(files) > keep_n:
            for f in files[:-keep_n]:
                try:
                    os.remove(f)
                except Exception:
                    pass


# ═══════════════════════════════════════════════════════════
#  深度图辅助
# ═══════════════════════════════════════════════════════════

def get_depth_at_point(depth_2d, cx, cy, r=5):
    """取深度图中指定坐标附近的深度值 (逐步扩大搜索半径, 中位数, mm)

    Args:
        depth_2d: 深度图 (H×W uint16, 单位 mm)
        cx, cy: 像素坐标
        r: 初始搜索半径

    Returns:
        深度值 mm, 无有效值返回 0.0
    """
    h, w = depth_2d.shape
    for radius in (r, r * 2, r * 4, r * 8, r * 16):
        y0, y1 = max(0, cy - radius), min(h, cy + radius + 1)
        x0, x1 = max(0, cx - radius), min(w, cx + radius + 1)
        region = depth_2d[y0:y1, x0:x1]
        valid = region[(region > 0) & (region < 10000)]
        if len(valid) >= 5:
            return float(np.median(valid))
    return 0.0
