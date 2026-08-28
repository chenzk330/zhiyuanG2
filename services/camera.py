#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera.py — 相机组件

职责：
  - 持续读取 G2 机器人的 4 路相机（头部彩色/深度、左右手腕彩色）
  - 将图像 base64 编码后发布到 /humanoid/camera/data
  - 接收 /humanoid/camera/control 控制命令：
      start          开始发布相机流
      stop           停止发布相机流
      save_photo     保存指定相机图片到 images/
      detect         拍摄头部彩深图并发送给 YOLO 检测服务

消息格式（/humanoid/camera/data，发布）：
{
  "timestamp": 1782975716895377276,
  "head_color": "<base64 jpeg>",
  "head_depth": "<base64 jpeg>",
  "left_wrist": "<base64 jpeg>",
  "right_wrist": "<base64 jpeg>"
}

消息格式（/humanoid/camera/control，订阅）：
  {"command": "start"}
  {"command": "stop"}
  {"command": "save_photo", "cameras": ["kHeadColor", "kHeadDepth"]}
  {"command": "detect", "yolo": "wxf.pt"}
"""

import os
import time
import json
import base64
import socket
import threading

import agibot_gdk

try:
    import cv2
    import numpy as np
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

import common

# ── 相机配置 ───────────────────────────────────────────────
# 4 路相机：(输出字段名, CameraType 枚举, 中文名)
CAMERA_LIST = [
    ("head_color", agibot_gdk.CameraType.kHeadColor,     "头部彩色"),
    ("head_depth", agibot_gdk.CameraType.kHeadDepth,     "头部深度"),
    ("left_wrist", agibot_gdk.CameraType.kHandLeftColor, "左手腕"),
    ("right_wrist", agibot_gdk.CameraType.kHandRightColor, "右手腕"),
]

# 相机名称字符串 → CameraType 枚举（用于 save/detect 命令的 cameras 字段）
CAMERA_NAME_MAP = {
    "kHeadColor":      agibot_gdk.CameraType.kHeadColor,
    "kHeadDepth":      agibot_gdk.CameraType.kHeadDepth,
    "kHandLeftColor":  agibot_gdk.CameraType.kHandLeftColor,
    "kHandRightColor": agibot_gdk.CameraType.kHandRightColor,
}

# 采集周期（秒）
LOOP_INTERVAL = 0.8

# JPEG 编码质量
JPEG_QUALITY = 60

# YOLO TCP 服务配置（cam_head / detect 命令使用）
YOLO_TCP_HOST = "10.2.236.7"
YOLO_TCP_PORT = 9998
YOLO_RECV_TIMEOUT = 60.0

# 发布开关（线程持续运行，按开关决定是否发布）
_publishing = False
_publish_lock = threading.Lock()

# 持续拍照开关
_continuous_capturing = False
_continuous_lock = threading.Lock()
_continuous_interval = 0.8
_continuous_count = 0


def is_publishing():
    with _publish_lock:
        return _publishing


def set_publishing(flag):
    global _publishing
    with _publish_lock:
        _publishing = flag


def is_continuous_capturing():
    with _continuous_lock:
        return _continuous_capturing


def set_continuous_capturing(flag):
    global _continuous_capturing, _continuous_count
    with _continuous_lock:
        _continuous_capturing = flag
        if not flag:
            _continuous_count = 0


# ═══════════════════════════════════════════════════════════
#  图像编码
# ═══════════════════════════════════════════════════════════

def encode_image(image, key):
    """把 GDK Image 编码为 base64 字符串

    - 已压缩格式（JPEG/PNG）：直接 base64
    - 深度图：转伪彩色后 JPEG 编码
    - 未压缩彩色：用 cv2 重新编码为 JPEG
    """
    if image is None or not hasattr(image, 'data') or image.data is None:
        return None

    raw = image.data
    encoding = getattr(image, 'encoding', None)

    # 已是压缩格式，直接 base64
    if encoding == agibot_gdk.Encoding.JPEG:
        return base64.b64encode(bytes(raw)).decode("ascii")
    if encoding == agibot_gdk.Encoding.PNG:
        return base64.b64encode(bytes(raw)).decode("ascii")

    if not HAS_CV2:
        return base64.b64encode(bytes(raw)).decode("ascii")

    try:
        color_format = getattr(image, 'color_format', None)

        if key == "head_depth":
            # 深度图转伪彩色
            if len(raw) == image.width * image.height * 2:
                depth = np.frombuffer(raw, dtype=np.uint16).reshape((image.height, image.width))
            elif len(raw) == image.width * image.height:
                depth = np.frombuffer(raw, dtype=np.uint8).reshape((image.height, image.width))
            else:
                depth = np.frombuffer(raw, dtype=np.uint16)
                if depth.size == image.width * image.height:
                    depth = depth.reshape((image.height, image.width))
                else:
                    return None

            valid = depth > 0
            if np.any(valid):
                mn, mx = depth[valid].min(), depth[valid].max()
                if mx > mn:
                    norm = ((depth.astype(np.float32) - mn) / (mx - mn) * 255).astype(np.uint8)
                else:
                    norm = np.zeros_like(depth, dtype=np.uint8)
            else:
                norm = np.zeros_like(depth, dtype=np.uint8)
            colored = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
        else:
            # 彩色图
            nparr = np.frombuffer(raw, dtype=np.uint8)
            if color_format == agibot_gdk.ColorFormat.RGB:
                img = nparr.reshape((image.height, image.width, 3))
                colored = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif color_format == agibot_gdk.ColorFormat.BGR:
                colored = nparr.reshape((image.height, image.width, 3))
            elif color_format == agibot_gdk.ColorFormat.GRAY8:
                gray = nparr.reshape((image.height, image.width))
                colored = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            else:
                colored = nparr.reshape((image.height, image.width, 3))

        ok, buf = cv2.imencode('.jpg', colored, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if ok:
            return base64.b64encode(buf).decode("ascii")
    except Exception as e:
        print(f"[相机] 编码失败 {key}: {e}")

    return None


# ═══════════════════════════════════════════════════════════
#  相机流发布循环
# ═══════════════════════════════════════════════════════════

def streaming_loop():
    """相机流发布主循环（在独立线程中运行）

    线程始终按 LOOP_INTERVAL 周期运行，
    - 当 is_publishing() 为 True 时读取并发布相机数据
    - 当 is_continuous_capturing() 为 True 时每隔 0.5 秒保存一张头部彩色照片
    """
    print("[相机] 发布线程已启动，等待 start 命令")
    last_capture_time = 0
    while True:
        try:
            t0 = time.time()

            if is_publishing():
                _capture_and_publish()

            if is_continuous_capturing():
                now = time.time()
                if now - last_capture_time >= _continuous_interval:
                    _save_continuous_head_color()
                    last_capture_time = now

            elapsed = time.time() - t0
            sleep_time = min(LOOP_INTERVAL, _continuous_interval)
            if elapsed < sleep_time:
                time.sleep(sleep_time - elapsed)
        except Exception as e:
            print(f"[相机] 循环异常: {e}")
            time.sleep(1.0)


def _capture_and_publish():
    """采集 4 路相机并发布到 /humanoid/camera/data"""
    msg = {"timestamp": int(time.time() * 1e9)}
    for key, cam_type, cam_name in CAMERA_LIST:
        try:
            img = common.camera.get_latest_image(cam_type, 1000.0)
            b64 = encode_image(img, key) if img is not None else None
        except Exception as e:
            b64 = None
            print(f"  [{cam_name}] 读取异常: {e}")
        if b64:
            msg[key] = b64
        else:
            print(f"  [{cam_name}] 无数据")

    common.publish(common.TOPIC_CAMERA_DATA, msg, qos=0)


# ═══════════════════════════════════════════════════════════
#  图片保存
# ═══════════════════════════════════════════════════════════

def save_camera_images(camera_names):
    """保存指定相机的图片到 images/ 目录

    Parameters
    ----------
    camera_names : list[str]
        相机名称列表，如 ["kHeadColor", "kHeadDepth"]
    """
    os.makedirs(common.IMAGE_SAVE_DIR, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    saved_files = []

    for name in camera_names:
        cam_type = CAMERA_NAME_MAP.get(name)
        if cam_type is None:
            print(f"  [保存] 未知相机名: {name}")
            continue

        try:
            img = common.camera.get_latest_image(cam_type, 1000.0)
        except Exception as e:
            print(f"  [保存] {name} 读取异常: {e}")
            continue

        if img is None or img.data is None:
            print(f"  [保存] {name} 无数据")
            continue

        fname = _save_single_image(img, name, timestamp)
        if fname:
            saved_files.append(fname)

    print(f"[保存] 完成，共 {len(saved_files)} 个文件 → {common.IMAGE_SAVE_DIR}")
    return saved_files


def _save_single_image(img, name, timestamp):
    """保存单张相机图片，返回保存的文件名"""
    if name == "kHeadDepth":
        return _save_depth_image(img, name, timestamp)

    encoding = getattr(img, 'encoding', None)
    if encoding == agibot_gdk.Encoding.JPEG:
        jpg_name = f"{name}_{timestamp}.jpg"
        with open(os.path.join(common.IMAGE_SAVE_DIR, jpg_name), "wb") as f:
            f.write(img.data)
        print(f"  [保存] {jpg_name}")
        return jpg_name

    if HAS_CV2:
        try:
            nparr = np.frombuffer(img.data, dtype=np.uint8)
            color_format = getattr(img, 'color_format', None)
            if color_format == agibot_gdk.ColorFormat.RGB:
                bgr = cv2.cvtColor(nparr.reshape((img.height, img.width, 3)), cv2.COLOR_RGB2BGR)
            else:
                bgr = nparr.reshape((img.height, img.width, 3))
            jpg_name = f"{name}_{timestamp}.jpg"
            cv2.imwrite(os.path.join(common.IMAGE_SAVE_DIR, jpg_name), bgr)
            print(f"  [保存] {jpg_name}")
            return jpg_name
        except Exception as e:
            print(f"  [保存] {name} 编码失败: {e}")

    raw_name = f"{name}_{timestamp}.raw"
    with open(os.path.join(common.IMAGE_SAVE_DIR, raw_name), "wb") as f:
        f.write(img.data)
    print(f"  [保存] {raw_name}")
    return raw_name


def _save_depth_image(img, name, timestamp):
    """保存深度图：原始 uint16 + 伪彩色 jpg"""
    # 原始数据
    raw_name = f"{name}_raw_{timestamp}.raw"
    with open(os.path.join(common.IMAGE_SAVE_DIR, raw_name), "wb") as f:
        f.write(img.data)
    print(f"  [保存] {raw_name}")

    if not HAS_CV2:
        return raw_name

    try:
        depth_array = np.frombuffer(img.data, dtype=np.uint16).reshape((img.height, img.width))
        valid_mask = depth_array > 0
        if np.any(valid_mask):
            mn, mx = depth_array[valid_mask].min(), depth_array[valid_mask].max()
        else:
            mn, mx = 0, 1
        if mx > mn:
            normalized = ((depth_array - mn) / (mx - mn) * 255).astype(np.uint8)
        else:
            normalized = np.zeros_like(depth_array, dtype=np.uint8)
        depth_colored = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
        cv2.putText(depth_colored, f"Depth: {mn}-{mx}mm", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        jpg_name = f"{name}_{timestamp}.jpg"
        cv2.imwrite(os.path.join(common.IMAGE_SAVE_DIR, jpg_name), depth_colored)
        print(f"  [保存] {jpg_name}")
        return jpg_name
    except Exception as e:
        print(f"  [保存] 深度伪彩色失败: {e}")
        return raw_name


def _save_continuous_head_color():
    """持续拍照模式：只保存头部彩色相机图片，带序号"""
    global _continuous_count
    try:
        os.makedirs(common.IMAGE_SAVE_DIR, exist_ok=True)
        img = common.camera.get_latest_image(agibot_gdk.CameraType.kHeadColor, 1000.0)
        if img is None or img.data is None:
            return

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        _continuous_count += 1
        seq = f"{_continuous_count:04d}"

        encoding = getattr(img, 'encoding', None)
        if encoding == agibot_gdk.Encoding.JPEG:
            jpg_name = f"cap_{timestamp}_{seq}.jpg"
            with open(os.path.join(common.IMAGE_SAVE_DIR, jpg_name), "wb") as f:
                f.write(img.data)
        elif HAS_CV2:
            nparr = np.frombuffer(img.data, dtype=np.uint8)
            color_format = getattr(img, 'color_format', None)
            if color_format == agibot_gdk.ColorFormat.RGB:
                bgr = cv2.cvtColor(nparr.reshape((img.height, img.width, 3)), cv2.COLOR_RGB2BGR)
            else:
                bgr = nparr.reshape((img.height, img.width, 3))
            jpg_name = f"cap_{timestamp}_{seq}.jpg"
            cv2.imwrite(os.path.join(common.IMAGE_SAVE_DIR, jpg_name), bgr)
        else:
            raw_name = f"cap_{timestamp}_{seq}.raw"
            with open(os.path.join(common.IMAGE_SAVE_DIR, raw_name), "wb") as f:
                f.write(img.data)
        print(f"  [连拍] 第{_continuous_count}张: {jpg_name}")
    except Exception as e:
        print(f"  [连拍] 保存失败: {e}")


# ═══════════════════════════════════════════════════════════
#  YOLO 目标检测
# ═══════════════════════════════════════════════════════════

def run_yolo_detect(model_name):
    """拍摄头部彩深图 → 保存图片 → TCP 发给 YOLO 服务 → 保存检测结果

    Parameters
    ----------
    model_name : str
        YOLO 模型文件名，如 "wxf.pt"
    """
    # 1. 拍摄头部彩色 + 深度
    color_img = _capture_head_image(agibot_gdk.CameraType.kHeadColor, "彩色")
    depth_img = _capture_head_image(agibot_gdk.CameraType.kHeadDepth, "深度")

    if color_img is None or color_img.data is None or depth_img is None or depth_img.data is None:
        print("[YOLO] 彩色或深度图未获取到，跳过检测")
        return None

    color_bytes = color_img.data
    depth_bytes = depth_img.data

    # 2. 保存图片
    os.makedirs(common.IMAGE_SAVE_DIR, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    _save_detect_images(color_img, depth_img, ts)

    # 3. base64 编码 + 构造请求
    rgb_b64 = base64.b64encode(color_bytes).decode("ascii")
    depth_b64 = base64.b64encode(depth_bytes).decode("ascii")

    payload = {
        "command": "detect",
        "rgb": rgb_b64,
        "depth": depth_b64,
        "model": model_name,
    }
    message = json.dumps(payload, ensure_ascii=False) + "\n"
    print(f"[YOLO] 请求: rgb={len(rgb_b64)}, depth={len(depth_b64)}, model={model_name}")

    # 4. TCP 发送并接收回复
    result = _tcp_detect_request(message)
    if result is None:
        return None

    # 5. 保存检测结果到 detect/
    _save_detect_result(result, ts)
    return result


def _capture_head_image(cam_type, label):
    """拍摄单张头部相机图像"""
    try:
        img = common.camera.get_latest_image(cam_type, 1000.0)
        if img is not None and img.data is not None:
            print(f"[YOLO] {label}图: {img.width}x{img.height}")
            return img
        print(f"[YOLO] 未获取到{label}图像")
    except Exception as e:
        print(f"[YOLO] {label}图读取异常: {e}")
    return None


def _save_detect_images(color_img, depth_img, ts):
    """保存检测用的彩色图和深度伪彩色图"""
    rgb_name = f"P{ts}_RGB.jpg"
    depth_name = f"P{ts}_DEPTH.jpg"

    # 保存彩色图
    try:
        encoding = getattr(color_img, 'encoding', None)
        if encoding == agibot_gdk.Encoding.JPEG:
            with open(os.path.join(common.IMAGE_SAVE_DIR, rgb_name), "wb") as f:
                f.write(color_img.data)
        elif HAS_CV2:
            nparr = np.frombuffer(color_img.data, dtype=np.uint8)
            color_format = getattr(color_img, 'color_format', None)
            if color_format == agibot_gdk.ColorFormat.RGB:
                bgr = cv2.cvtColor(nparr.reshape((color_img.height, color_img.width, 3)), cv2.COLOR_RGB2BGR)
            else:
                bgr = nparr.reshape((color_img.height, color_img.width, 3))
            cv2.imwrite(os.path.join(common.IMAGE_SAVE_DIR, rgb_name), bgr)
        else:
            with open(os.path.join(common.IMAGE_SAVE_DIR, rgb_name), "wb") as f:
                f.write(color_img.data)
        print(f"[YOLO] 彩色图已保存: {rgb_name}")
    except Exception as e:
        print(f"[YOLO] 彩色图保存失败: {e}")

    # 保存深度伪彩色图
    try:
        if HAS_CV2:
            depth_array = np.frombuffer(depth_img.data, dtype=np.uint16).reshape((depth_img.height, depth_img.width))
            valid_mask = depth_array > 0
            if np.any(valid_mask):
                mn, mx = depth_array[valid_mask].min(), depth_array[valid_mask].max()
            else:
                mn, mx = 0, 1
            if mx > mn:
                normalized = ((depth_array - mn) / (mx - mn) * 255).astype(np.uint8)
            else:
                normalized = np.zeros_like(depth_array, dtype=np.uint8)
            depth_colored = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
            cv2.putText(depth_colored, f"Depth: {mn}-{mx}mm", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imwrite(os.path.join(common.IMAGE_SAVE_DIR, depth_name), depth_colored)
            print(f"[YOLO] 深度图已保存: {depth_name}")
        else:
            with open(os.path.join(common.IMAGE_SAVE_DIR, depth_name), "wb") as f:
                f.write(depth_img.data)
            print(f"[YOLO] 深度图(原始)已保存: {depth_name}")
    except Exception as e:
        print(f"[YOLO] 深度图保存失败: {e}")


def _tcp_detect_request(message):
    """通过 TCP 发送检测请求并接收回复"""
    sock = None
    try:
        print(f"[YOLO] 连接 {YOLO_TCP_HOST}:{YOLO_TCP_PORT} ...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(YOLO_RECV_TIMEOUT)
        sock.connect((YOLO_TCP_HOST, YOLO_TCP_PORT))
        sock.sendall(message.encode("utf-8"))
        print("[YOLO] 报文已发送，等待回复...")

        received = b""
        while True:
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                print("[YOLO] 接收超时")
                break
            if not chunk:
                break
            received += chunk
            if b"\n" in chunk:
                break

        if not received:
            print("[YOLO] 未收到回复")
            return None

        print(f"[YOLO] 收到回复，长度={len(received)} 字节")
        try:
            return json.loads(received.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            print(f"[YOLO] 回复非合法 JSON: {e}")
            return {"raw": received.decode("utf-8", errors="replace")}
    except Exception as e:
        print(f"[YOLO] TCP 通信失败: {e}")
        return None
    finally:
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass


def _save_detect_result(result, ts):
    """保存检测结果到 detect/ 目录"""
    os.makedirs(common.DETECT_SAVE_DIR, exist_ok=True)

    # 保存带时间戳的结果
    ts_path = os.path.join(common.DETECT_SAVE_DIR, f"D{ts}.json")
    with open(ts_path, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print(f"[YOLO] 检测结果已保存: D{ts}.json")

    # 覆盖最新结果
    latest_path = os.path.join(common.DETECT_SAVE_DIR, "detect.json")
    with open(latest_path, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print(f"[YOLO] 最新结果已覆盖: detect.json")


# ═══════════════════════════════════════════════════════════
#  命令处理
# ═══════════════════════════════════════════════════════════

def handle_control(payload):
    """处理 /humanoid/camera/control 命令

    Parameters
    ----------
    payload : dict
        解析后的命令消息，如 {"command": "start"}
    """
    cmd = payload.get("command", "").lower()

    if cmd == "start":
        set_publishing(True)
        print("[相机] 开始发布")

    elif cmd == "stop":
        set_publishing(False)
        print("[相机] 停止发布")

    elif cmd == "start_continuous_capture":
        set_continuous_capturing(True)
        print("[相机] 开始持续拍照（头部彩色，间隔0.5秒）")

    elif cmd == "stop_continuous_capture":
        set_continuous_capturing(False)
        print("[相机] 停止持续拍照")

    elif cmd == "save_photo":
        cameras = payload.get("cameras", [])
        if not cameras:
            print("[相机] save_photo 缺少 cameras 字段")
            return
        print(f"[相机] 保存图片: {cameras}")
        # 在子线程中执行保存，避免阻塞
        t = threading.Thread(target=_run_save, args=(cameras,), daemon=True)
        t.start()

    elif cmd == "detect":
        yolo_model = payload.get("yolo", "")
        if not yolo_model:
            print("[相机] detect 缺少 yolo 字段")
            return
        print(f"[相机] YOLO 检测: model={yolo_model}")
        t = threading.Thread(target=_run_detect, args=(yolo_model,), daemon=True)
        t.start()

    else:
        print(f"[相机] 未知命令: {cmd}")


def _run_save(cameras):
    """在子线程中执行保存图片，完成后发送 done 通知"""
    try:
        save_camera_images(cameras)
    except Exception as e:
        print(f"[相机] 保存异常: {e}")
    finally:
        common.publish_done("save_photo")


def _run_detect(model_name):
    """在子线程中执行 YOLO 检测，完成后发送 done 通知"""
    try:
        run_yolo_detect(model_name)
    except Exception as e:
        print(f"[相机] 检测异常: {e}")
    finally:
        common.publish_done("detect")


def start_streaming_thread():
    """启动相机流发布线程（由 main.py 在初始化时调用）"""
    t = threading.Thread(target=streaming_loop, daemon=True)
    t.start()
    print("[相机] 发布线程已启动")
