#!/usr/bin/env python3
"""
detect_server.py
TCP 服务端 (端口 9998)，融合 YOLO 检测/分割 + 深度采样, 支持多模型按需加载。

协议 (裸 JSON, 括号配平分帧):
  - client/server 均直接发送 UTF-8 JSON 文本, 末尾以 '\\n' 分隔
  - 一条消息的边界用 "{/} / [/]" 括号配平 + 字符串感知判定 (base64 不含这些字符, 安全)

请求报文 (client -> server):
  {"cmd": "detect", "rgb": "<base64 jpg>", "depth": "<base64 raw uint16>", "model": "wxf.pt"}
  model 字段可选, 指定 .pt 文件名 (如 "wxf.pt" / "7.14.pt"); 缺省用 DEFAULT_MODEL。
  首次使用某模型时懒加载并缓存, 后续请求复用。

处理:
  1. base64 -> 存 rgb.jpg / depth.raw
  2. 用指定模型做 YOLO 推理:
       - detect 任务 (如 wxf.pt): 直接用 boxes.xyxy
       - segment 任务 (如 7.14.pt): 同样用 boxes.xyxy (mask 外接框), 并额外画 mask 轮廓
     取置信度最高的 a box 与 b box, 计算 box 中心点
  3. 计算 a/b 中心连线、中点、水平偏移、斜率
  4. 从 depth.raw 采样深度 (a中心 / b中心 / 中点)
  5. 保存标注图 + 结果 JSON, 并把结果 JSON 回发给 client

响应报文 (server -> client): 裸 JSON + '\\n', 字段见 make_result()
"""
import base64
import json
import os
import socket
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import paho.mqtt.client as mqtt

from ultralytics import YOLO

# ===================== 配置 =====================
HOST = "0.0.0.0"
PORT = 9998

BASE_DIR = Path(__file__).resolve().parent
MODEL_DIR = BASE_DIR              # 模型 .pt 文件所在目录
DEFAULT_MODEL = "wxf.pt"          # 请求未指定 model 时使用
RGB_PATH = str(BASE_DIR / "rgb.jpg")
DEPTH_RAW_PATH = str(BASE_DIR / "depth.raw")
DEPTH_SHAPE = (400, 640)          # 深度图尺寸 (H, W), 不匹配时自动推断
DEPTH_OFFSET = 12                 # 深度采样相对中心的纵向偏移 (参考 demo.py)

# 输出
OUT_RGB = str(BASE_DIR / "server_result_rgb.jpg")
OUT_RESULT_JSON = str(BASE_DIR / "server_result.json")

# MQTT 配置 (发布标注结果图 base64)
MQTT_HOST = "10.2.236.6"
MQTT_PORT = 1883
MQTT_TOPIC = "/minth/g2/camera/detect"

# 模型缓存 {模型名(相对/绝对): YOLO}, 按需懒加载
_models = {}
_models_lock = threading.Lock()


def resolve_model_path(model_name: str) -> str:
    """解析模型路径, 支持纯文件名 (如 'wxf.pt') 或绝对/相对路径。"""
    if not model_name:
        model_name = DEFAULT_MODEL
    p = Path(model_name)
    if not p.is_absolute():
        p = MODEL_DIR / p
    return str(p)


def get_model(model_name: str) -> YOLO:
    """按模型名懒加载并缓存; 线程安全。返回 YOLO 实例。"""
    key = model_name or DEFAULT_MODEL
    with _models_lock:
        m = _models.get(key)
        if m is not None:
            return m
        path = resolve_model_path(key)
        if not os.path.exists(path):
            raise FileNotFoundError(f"模型文件不存在: {path}")
        print(f"[server] 加载模型: {key} -> {path}")
        m = YOLO(path)
        print(f"[server] 模型就绪: {key} task={m.task} names={m.names}")
        _models[key] = m
        return m


# ===================== 深度读取 (移植自 demo.py) =====================
def load_depth_from_raw(raw_path: str, shape=None):
    if not os.path.exists(raw_path):
        return None
    raw_bytes = open(raw_path, "rb").read()
    total = len(raw_bytes)
    n_pixels = total // 2
    if shape is not None:
        H, W = shape
        if n_pixels != H * W:
            return _auto_reshape(raw_bytes)
        return np.frombuffer(raw_bytes, dtype=np.uint16).reshape((H, W))
    return _auto_reshape(raw_bytes)


def _auto_reshape(raw_bytes: bytes):
    n_pixels = len(raw_bytes) // 2
    common_resolutions = [
        (400, 640), (480, 640), (480, 848), (360, 640),
        (720, 1280), (240, 424), (400, 848), (720, 960),
    ]
    for H, W in common_resolutions:
        if H * W == n_pixels:
            return np.frombuffer(raw_bytes, dtype=np.uint16).reshape((H, W))
    side = int(np.sqrt(n_pixels))
    return np.frombuffer(raw_bytes, dtype=np.uint16).reshape((side, side))


def get_depth_at_pixel(depth_raw, x, y, search_radius=10):
    h, w = depth_raw.shape[:2]
    if 0 <= x < w and 0 <= y < h:
        val = depth_raw[y, x]
        if val > 0:
            return float(val)
    for r in range(1, search_radius + 1):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if abs(dx) + abs(dy) != r:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h:
                    val = depth_raw[ny, nx]
                    if val > 0:
                        return float(val)
    return -1.0


def get_average_depth(depth_raw, x, y, radius=5):
    h, w = depth_raw.shape[:2]
    depths = []
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            if dx * dx + dy * dy <= radius * radius:
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h:
                    val = depth_raw[ny, nx]
                    if val > 0:
                        depths.append(val)
    if not depths:
        return -1.0
    return float(np.mean(depths))


# ===================== YOLO 检测 + 计算 (box 中心点) =====================
def detect_and_compute(rgb_path: str, model_name: str):
    """用指定模型检测, 取置信度最高的 a box 与 b box, 返回 (img, 信息字典, 错误字符串)。
    detect 与 segment 任务统一用 box 中心点逻辑; segment 额外画 mask 轮廓。"""
    model = get_model(model_name)
    t0 = time.time()
    results = model(rgb_path, verbose=False)
    infer_time = time.time() - t0

    r0 = results[0]
    img = r0.orig_img.copy()
    img_h, img_w = img.shape[:2]
    img_center_x = img_w / 2.0
    boxes = r0.boxes
    names = r0.names
    is_seg = (model.task == "segment") and (r0.masks is not None)

    def collect(target_label):
        cid = None
        for k, v in names.items():
            if v == target_label:
                cid = k
                break
        out = []
        if cid is None:
            return out
        for i, box in enumerate(boxes):
            if int(box.cls[0]) != cid:
                continue
            conf = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cx = float((x1 + x2) / 2)
            cy = float((y1 + y2) / 2)
            entry = {"x1": float(x1), "y1": float(y1), "x2": float(x2), "y2": float(y2),
                     "cx": cx, "cy": cy, "conf": conf, "idx": i}
            out.append(entry)
        out.sort(key=lambda d: d["conf"], reverse=True)
        return out

    boxes_a = collect("a")
    boxes_b = collect("b")

    info = {
        "model": model_name or DEFAULT_MODEL,
        "task": model.task,
        "image_size": {"height": int(img_h), "width": int(img_w)},
        "image_center_x": img_center_x,
        "boxes_a": boxes_a,
        "boxes_b": boxes_b,
        "inference_time_sec": round(infer_time, 4),
    }

    # 需要 a 与 b 各至少 1 个
    if not boxes_a or not boxes_b:
        return img, info, f"检测不全: a={len(boxes_a)}, b={len(boxes_b)}"

    a = boxes_a[0]
    b = boxes_b[0]
    cx1, cy1 = a["cx"], a["cy"]
    cx2, cy2 = b["cx"], b["cy"]

    # —— 画 box ——
    cv2.rectangle(img, (int(a["x1"]), int(a["y1"])), (int(a["x2"]), int(a["y2"])), (255, 0, 0), 2)
    cv2.rectangle(img, (int(b["x1"]), int(b["y1"])), (int(b["x2"]), int(b["y2"])), (0, 255, 0), 2)
    cv2.putText(img, f'a {a["conf"]:.2f}', (int(a["x1"]), int(a["y1"]) - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    cv2.putText(img, f'b {b["conf"]:.2f}', (int(b["x1"]), int(b["y1"]) - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # —— segment 任务额外画 mask 轮廓 ——
    if is_seg:
        try:
            masks_xy = r0.masks.xy  # list[ndarray(N,2)]
            for ent, color in ((a, (255, 0, 0)), (b, (0, 255, 0))):
                idx = ent.get("idx")
                if idx is None or idx >= len(masks_xy):
                    continue
                poly = masks_xy[idx].astype(np.int32).reshape(-1, 1, 2)
                cv2.polylines(img, [poly], isClosed=True, color=color, thickness=2)
        except Exception:
            pass

    # —— 中心连线 (红) + 两个中心点 (黄) ——
    cv2.line(img, (int(cx1), int(cy1)), (int(cx2), int(cy2)), (0, 0, 255), 2)
    cv2.circle(img, (int(cx1), int(cy1)), 5, (0, 255, 255), -1)
    cv2.circle(img, (int(cx2), int(cy2)), 5, (0, 255, 255), -1)

    # —— 线段中点 (蓝) ——
    line_center_x = (cx1 + cx2) / 2
    line_center_y = (cy1 + cy2) / 2
    cv2.circle(img, (int(line_center_x), int(line_center_y)), 8, (255, 0, 0), -1)
    cv2.putText(img, f'({line_center_x:.1f}, {line_center_y:.1f})',
                (int(line_center_x) + 10, int(line_center_y) - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    # —— 水平偏移 ——
    h_offset = line_center_x - img_center_x
    cv2.line(img, (int(img_center_x), 0), (int(img_center_x), img_h), (0, 255, 0), 1)
    cv2.line(img, (int(img_center_x), int(line_center_y)),
             (int(line_center_x), int(line_center_y)), (255, 255, 0), 2)
    cv2.putText(img, f'h_offset: {h_offset:.1f}px',
                (int(min(img_center_x, line_center_x)) + 5, int(line_center_y) - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    # —— 斜率 ——
    dx = cx2 - cx1
    dy = cy2 - cy1
    angle_rad = float(np.arctan2(dy, dx))
    slope = float("inf") if abs(dx) < 1e-6 else float(dy / dx)
    cv2.putText(img, f'slope: {"inf" if np.isinf(slope) else f"{slope:.2f}"}',
                (max(int((cx1 + cx2) / 2) - 80, 10), max(int((cy1 + cy2) / 2) - 15, 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    info.update({
        "a": a,
        "b": b,
        "a_center": [round(cx1, 2), round(cy1, 2)],
        "b_center": [round(cx2, 2), round(cy2, 2)],
        "line_center": [round(line_center_x, 2), round(line_center_y, 2)],
        "horizontal_offset_px": round(h_offset, 2),
        "direction": "偏右" if h_offset > 0 else ("偏左" if h_offset < 0 else "居中"),
        "slope": None if np.isinf(slope) else round(slope, 4),
        "angle_rad": round(angle_rad, 4),
        "angle_deg": round(float(np.degrees(angle_rad)), 2),
    })
    return img, info, None


def sample_depth(depth_raw, info):
    """从深度图采样 a中心 / b中心 / 中点 的深度, 返回深度信息字典。"""
    if depth_raw is None:
        return None
    cx1, cy1 = info["a_center"]
    cx2, cy2 = info["b_center"]
    mcx, mcy = info["line_center"]

    depth_a_center = get_depth_at_pixel(depth_raw, int(cx1), int(cy1))
    depth_b_center = get_depth_at_pixel(depth_raw, int(cx2), int(cy2))
    depth_mid = get_average_depth(depth_raw, int(mcx), int(mcy), radius=5)

    # 参考 demo.py: a 中心下偏 DEPTH_OFFSET, b 中心下偏 DEPTH_OFFSET 的邻域平均
    a_off_x, a_off_y = int(cx1), int(cy1) + DEPTH_OFFSET
    b_off_x, b_off_y = int(cx2), int(cy2) + DEPTH_OFFSET
    depth_a_offset = get_average_depth(depth_raw, a_off_x, a_off_y, radius=2)
    depth_b_offset = get_average_depth(depth_raw, b_off_x, b_off_y, radius=2)

    return {
        "depth_shape": list(depth_raw.shape),
        "a_center_mm": round(depth_a_center, 1),
        "b_center_mm": round(depth_b_center, 1),
        "midpoint_mm": round(depth_mid, 1),
        "a_offset_mm": round(depth_a_offset, 1),
        "a_offset_pixel": [a_off_x, a_off_y],
        "b_offset_mm": round(depth_b_offset, 1),
        "b_offset_pixel": [b_off_x, b_off_y],
    }


# ===================== MQTT 发布 =====================
def publish_result_image_mqtt(img_path: str):
    """读取图片文件 -> base64 编码 -> 发布到 MQTT topic。同步阻塞, 失败不影响主流程。"""
    try:
        with open(img_path, "rb") as f:
            b64 = base64.b64encode(f.read()).decode("ascii")
        payload = json.dumps({"image": b64, "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")}, ensure_ascii=False)
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        client.connect(MQTT_HOST, MQTT_PORT, keepalive=30)
        client.loop_start()                       # 启动网络循环线程, 否则 publish 不会真正发出
        info = client.publish(MQTT_TOPIC, payload, qos=0)
        info.wait_for_publish(timeout=5)          # 等待真正发送完成
        client.loop_stop()
        client.disconnect()
        print(f"[mqtt] 已发布标注图 base64 ({len(payload)} bytes) -> {MQTT_TOPIC}")
    except Exception as e:
        print(f"[mqtt] 发布失败 (不影响主流程): {e}")


# ===================== 结果组装 =====================
def make_result(req, info, depth_info, err):
    res = {
        "cmd": "detect",
        "status": "error" if err else "ok",
        "error": err,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "model": info.get("model"),
        "task": info.get("task"),
        "rgb_path": RGB_PATH,
        "depth_raw_path": DEPTH_RAW_PATH,
        "image_size": info.get("image_size"),
        "inference_time_sec": info.get("inference_time_sec"),
        "counts": {"a": len(info.get("boxes_a", [])), "b": len(info.get("boxes_b", []))},
    }
    if not err:
        res.update({
            "a_box": {
                "xyxy": [round(info["a"]["x1"], 2), round(info["a"]["y1"], 2),
                         round(info["a"]["x2"], 2), round(info["a"]["y2"], 2)],
                "conf": round(info["a"]["conf"], 4),
                "center": info["a_center"],
            },
            "b_box": {
                "xyxy": [round(info["b"]["x1"], 2), round(info["b"]["y1"], 2),
                         round(info["b"]["x2"], 2), round(info["b"]["y2"], 2)],
                "conf": round(info["b"]["conf"], 4),
                "center": info["b_center"],
            },
            "line_center": info["line_center"],
            "image_center_x": round(info["image_center_x"], 2),
            "horizontal_offset_px": info["horizontal_offset_px"],
            "direction": info["direction"],
            "slope": info["slope"],
            "angle_rad": info["angle_rad"],
            "angle_deg": info["angle_deg"],
            "depth": depth_info,
        })
    return res


# ===================== TCP 分帧 (裸 JSON) =====================
# 协议: client/server 均直接发送 UTF-8 JSON 文本, 不带长度前缀。
# 一条消息的边界用 "括号配平 + 字符串感知" 判定:
#   跟踪 {/[ 与 }/] 的嵌套深度, 并忽略出现在字符串内的括号;
#   当深度从 >0 回到 0 时, 说明一个完整 JSON 对象已读完。
# 安全性: base64 字母表 (A-Za-z0-9+/=) 不含 { } [ ] " \, 故不会干扰配平。
MAX_MSG = 256 * 1024 * 1024  # 256MB 上限


def recv_json_bytes(sock: socket.socket) -> bytes | None:
    """读取一条完整 JSON 文本(字节); 连接关闭返回 None。"""
    buf = bytearray()
    depth = 0
    in_str = False
    escape = False
    while True:
        chunk = sock.recv(65536)
        if not chunk:
            return None if not buf else bytes(buf)  # 连接关闭
        buf.extend(chunk)
        if len(buf) > MAX_MSG:
            raise ValueError(f"消息超过上限 {MAX_MSG} 字节")
        # 只扫描本块新增字节, 状态跨块延续
        for b in chunk:
            c = b  # 比较 ASCII 字节值, base64/UTF-8 高字节不会命中分隔符
            if in_str:
                if escape:
                    escape = False
                elif c == 0x5C:  # 反斜杠 \
                    escape = True
                elif c == 0x22:  # 引号 "
                    in_str = False
            else:
                if c == 0x22:   # "
                    in_str = True
                elif c == 0x7B or c == 0x5B:  # { 或 [
                    depth += 1
                elif c == 0x7D or c == 0x5D:  # } 或 ]
                    depth -= 1
                    if depth == 0:
                        return bytes(buf)
        # 否则继续 recv


def send_msg(sock: socket.socket, payload: bytes):
    """发送裸 JSON 文本, 末尾追加换行作为友好分隔 (json.loads 会忽略)。"""
    sock.sendall(payload + b"\n")


# ===================== 请求处理 =====================
def handle_client(conn: socket.socket, addr):
    print(f"[server] 连接来自 {addr}")
    try:
        while True:
            data = recv_json_bytes(conn)
            if data is None:
                break
            try:
                req = json.loads(data.decode("utf-8"))
            except Exception as e:
                resp = {"cmd": "?", "status": "error", "error": f"JSON 解析失败: {e}"}
                send_msg(conn, json.dumps(resp, ensure_ascii=False).encode("utf-8"))
                continue

            cmd = req.get("cmd", "")
            if cmd != "detect":
                resp = {"cmd": cmd, "status": "error", "error": f"未知 cmd: {cmd}"}
                send_msg(conn, json.dumps(resp, ensure_ascii=False).encode("utf-8"))
                continue

            try:
                # 1. base64 -> 文件
                rgb_b64 = req.get("rgb", "")
                depth_b64 = req.get("depth", "")
                model_name = req.get("model", "") or DEFAULT_MODEL
                with open(RGB_PATH, "wb") as f:
                    f.write(base64.b64decode(rgb_b64))
                with open(DEPTH_RAW_PATH, "wb") as f:
                    f.write(base64.b64decode(depth_b64))
                print(f"[server] 已保存 rgb.jpg / depth.raw (来自 {addr}, model={model_name})")

                # 2. YOLO 检测 + box 中心计算 (按 model 字段选择/懒加载模型)
                img, info, err = detect_and_compute(RGB_PATH, model_name)

                # 3. 深度采样
                depth_info = None
                if not err:
                    depth_raw = load_depth_from_raw(DEPTH_RAW_PATH, DEPTH_SHAPE)
                    depth_info = sample_depth(depth_raw, info)
                    # 在图上标深度 (a/b/中点)
                    if depth_raw is not None:
                        cx1, cy1 = info["a_center"]
                        cx2, cy2 = info["b_center"]
                        mcx, mcy = info["line_center"]
                        cv2.putText(img, f'a:{depth_info["a_center_mm"]:.0f}mm',
                                    (int(cx1) + 6, int(cy1) + 18),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        cv2.putText(img, f'b:{depth_info["b_center_mm"]:.0f}mm',
                                    (int(cx2) + 6, int(cy2) + 18),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.putText(img, f'mid:{depth_info["midpoint_mm"]:.0f}mm',
                                    (int(mcx) + 10, int(mcy) + 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)


                
                

                # 5. 组装结果
                resp = make_result(req, info, depth_info, err)
                with open(OUT_RESULT_JSON, "w", encoding="utf-8") as f:
                    json.dump(resp, f, ensure_ascii=False, indent=2)
                print(f"[server] 处理完成 status={resp['status']} -> {OUT_RESULT_JSON}")


            except Exception as e:
                resp = {"cmd": "detect", "status": "error", "error": f"处理异常: {e}"}

            # 6. 回发 client
            send_msg(conn, json.dumps(resp, ensure_ascii=False).encode("utf-8"))
            cv2.imwrite(OUT_RGB, img)
            publish_result_image_mqtt(OUT_RGB)            
    except Exception as e:
        print(f"[server] 连接异常 {addr}: {e}")
    finally:
        conn.close()
        print(f"[server] 连接关闭 {addr}")


def main():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(8)
    print(f"[server] 监听 {HOST}:{PORT}  (裸 JSON 协议: 括号配平分帧)")
    print(f"[server] 默认模型: {DEFAULT_MODEL} (按请求 model 字段懒加载, 例如 wxf.pt / 7.14.pt)")
    try:
        while True:
            conn, addr = srv.accept()
            t = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
            t.start()
    except KeyboardInterrupt:
        print("\n[server] 退出")
    finally:
        srv.close()


if __name__ == "__main__":
    main()
