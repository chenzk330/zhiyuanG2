#!/usr/bin/env python3
"""
yolo_depth.py
融合 YOLO 检测 + 从 head_depth.raw 读取深度值：
1. 对 head.jpg 做 YOLO 检测，找到两个点 a/b，画线
2. 计算线的中心点、中心线与图像中线的水平偏移像素、斜率
3. 从 head_depth.raw 读取深度图，采样：
   - 点 a 左侧 5 个像素处的深度值
   - 点 b 右侧 5 个像素处的深度值
4. 输出标注后的图像及深度信息
"""

import cv2
import numpy as np
import os
import sys
import json
import time

from ultralytics import YOLO

# ===================== 全局配置 =====================
IMG_PATH = 'head.jpg'                    # 输入的 RGB 图像
DEPTH_RAW_PATH = 'head_depth.raw'        # 深度原始数据文件
DEPTH_SHAPE = (400, 640)                 # 深度图尺寸 (H, W)，根据实际情况修改（当前文件 512000B = 400x640）
MODEL_PATH = sys.argv[1] if len(sys.argv) > 1 else '06131557.pt'
DEPTH_OFFSET = int(sys.argv[2]) if len(sys.argv) > 2 else 12
SAVE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "images")
os.makedirs(SAVE_DIR, exist_ok=True)

# ===================== 从 .raw 文件读取深度 =====================

def load_depth_from_raw(raw_path: str, shape: tuple = None) -> np.ndarray | None:
    """
    从 head_depth.raw 加载深度数据。
    返回 uint16 数组 (H, W)，失败返回 None。
    shape: (H, W)，不指定时自动推断。
    """
    if not os.path.exists(raw_path):
        print(f"❌ 深度文件不存在: {raw_path}")
        return None

    raw_bytes = open(raw_path, "rb").read()
    total = len(raw_bytes)
    # uint16 = 2 bytes per pixel
    n_pixels = total // 2

    if shape is not None:
        H, W = shape
        if n_pixels != H * W:
            print(f"⚠️ 文件大小 ({total}B = {n_pixels}px) 与指定尺寸 {shape} ({H*W}px) 不匹配，"
                  f"尝试自动推断...")
            # 自动推断
            return _auto_reshape(raw_bytes)
        depth = np.frombuffer(raw_bytes, dtype=np.uint16).reshape((H, W))
    else:
        depth = _auto_reshape(raw_bytes)

    print(f"深度图尺寸: {depth.shape}, dtype={depth.dtype}")
    valid = depth[depth > 0]
    if len(valid) > 0:
        print(f"深度范围: {valid.min()} ~ {valid.max()} mm")
    else:
        print("⚠️ 深度图全部为 0（无效）")
    return depth


def _auto_reshape(raw_bytes: bytes) -> np.ndarray | None:
    """自动推断深度图宽高（常见分辨率）"""
    n_pixels = len(raw_bytes) // 2
    # 常见深度相机分辨率
    common_resolutions = [
        (400, 640),   # 当前实际尺寸
        (480, 640),   # 最常见的 VGA
        (480, 848),   # 广角
        (360, 640),
        (720, 1280),
        (240, 424),
        (400, 848),   # 其他可能
        (720, 960),
    ]
    for H, W in common_resolutions:
        if H * W == n_pixels:
            print(f"自动推断为 {H}x{W}")
            return np.frombuffer(raw_bytes, dtype=np.uint16).reshape((H, W))

    # 尝试按正方形/接近正方形分配
    side = int(np.sqrt(n_pixels))
    print(f"⚠️ 无法匹配常见分辨率，尝试 {side}x{side}...")
    return np.frombuffer(raw_bytes, dtype=np.uint16).reshape((side, side))


# ===================== YOLO 检测部分 (基于 yolo.py) =====================

def yolo_detect(image_path: str):
    """YOLO 检测，返回原始图像、检测框信息、图像尺寸"""
    start_time = time.time()
    model = YOLO(MODEL_PATH)
    load_time = time.time() - start_time
    print(f"模型加载耗时: {load_time:.4f} 秒")

    # 测量推理时间
    start_time = time.time()
    results = model(image_path)
    inference_time = time.time() - start_time
    print(f"推理耗时: {inference_time:.4f} 秒")
    results[0].save(filename='result.jpg')

    img = results[0].orig_img.copy()
    img_h, img_w = img.shape[:2]
    img_center_x = img_w / 2.0
    boxes = results[0].boxes
    names = results[0].names

    print(f"图像尺寸: {img_h} x {img_w}")
    print(f"检测到的所有类别: {names}")
    print(f"总检测框数: {len(boxes)}")

    # 按类别收集框 (center_x, center_y, x1, y1, x2, y2, conf)
    def collect_boxes_by_class(target_label: str):
        class_id = None
        for cid, cname in names.items():
            if cname == target_label:
                class_id = cid
                break
        collected = []
        if class_id is None:
            return collected
        for box in boxes:
            cls_id = int(box.cls[0])
            if cls_id != class_id:
                continue
            conf = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            collected.append((cx, cy, x1, y1, x2, y2, conf))
        collected.sort(key=lambda x: x[6], reverse=True)
        return collected

    boxes_a = collect_boxes_by_class('a')
    boxes_b = collect_boxes_by_class('b')
    boxes_c = collect_boxes_by_class('c')
    boxes_d = collect_boxes_by_class('d')

    print(f"检测到 a={len(boxes_a)}, b={len(boxes_b)}, c={len(boxes_c)}, d={len(boxes_d)}")

    # ========== 策略选择两个画线点 ==========
    pt1 = pt2 = None

    if len(boxes_a) >= 1 and len(boxes_b) >= 1:
        pt1, pt2 = boxes_a[0], boxes_b[0]
        print("策略: 使用最高置信度的 a 和 b 画线")
    elif len(boxes_b) >= 2:
        pt1, pt2 = boxes_b[0], boxes_b[1]
        print("策略: 使用最高置信度的 2 个 b 画线")
    elif len(boxes_a) >= 2:
        pt1, pt2 = boxes_a[0], boxes_a[1]
        print("策略: 使用最高置信度的 2 个 a 画线")
    elif len(boxes_c) >= 1 and len(boxes_d) >= 1:
        pt1, pt2 = boxes_c[0], boxes_d[0]
        print("策略: 使用最高置信度的 c 和 d 画线")
    elif len(boxes_c) >= 2:
        pt1, pt2 = boxes_c[0], boxes_c[1]
        print("策略: 使用最高置信度的 2 个 c 画线")
    elif len(boxes_d) >= 2:
        pt1, pt2 = boxes_d[0], boxes_d[1]
        print("策略: 使用最高置信度的 2 个 d 画线")
    else:
        print(f"❌ 无法满足任何画线条件")
        return None, None, None, None, None, None

    return (img, img_h, img_w, img_center_x, pt1, pt2,
            boxes_a, boxes_b, boxes_c, boxes_d)


def get_label(pt, ba, bb, bc, bd):
    if pt in ba: return 'a'
    if pt in bb: return 'b'
    if pt in bc: return 'c'
    if pt in bd: return 'd'
    return '?'


def label_color(label):
    return {'a': (255, 0, 0), 'b': (0, 255, 0), 'c': (0, 0, 255), 'd': (0, 165, 255)}.get(label, (128, 128, 128))


def draw_and_calculate(img, img_h, img_w, img_center_x, pt1, pt2,
                       boxes_a, boxes_b, boxes_c, boxes_d):
    """在图上标注、画线、计算偏移和斜率"""
    cx1, cy1, x1_1, y1_1, x1_2, y1_2, conf1 = pt1
    cx2, cy2, x2_1, y2_1, x2_2, y2_2, conf2 = pt2

    label1 = get_label(pt1, boxes_a, boxes_b, boxes_c, boxes_d)
    label2 = get_label(pt2, boxes_a, boxes_b, boxes_c, boxes_d)

    print(f"点1: label={label1}, conf={conf1:.4f}, center=({cx1:.1f}, {cy1:.1f})")
    print(f"点2: label={label2}, conf={conf2:.4f}, center=({cx2:.1f}, {cy2:.1f})")

    color1 = label_color(label1)
    color2 = label_color(label2)

    color1 = label_color(label1)
    color2 = label_color(label2)

    # —— 画检测框 ——
    cv2.rectangle(img, (int(x1_1), int(y1_1)), (int(x1_2), int(y1_2)), color1, 2)
    cv2.rectangle(img, (int(x2_1), int(y2_1)), (int(x2_2), int(y2_2)), color2, 2)
    cv2.putText(img, f'{label1} {conf1:.2f}', (int(x1_1), int(y1_1)-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color1, 2)
    cv2.putText(img, f'{label2} {conf2:.2f}', (int(x2_1), int(y2_1)-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color2, 2)

    # —— 画中心连线（红色） ——
    cv2.line(img, (int(cx1), int(cy1)), (int(cx2), int(cy2)), (0, 0, 255), 2)

    # —— 线的中心点 ——
    line_center_x = (cx1 + cx2) / 2
    line_center_y = (cy1 + cy2) / 2
    cv2.circle(img, (int(line_center_x), int(line_center_y)), 8, (255, 0, 0), -1)
    cv2.putText(img, f'({line_center_x:.1f}, {line_center_y:.1f})',
                (int(line_center_x)+10, int(line_center_y)-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    # —— 画两个中心点 ——
    cv2.circle(img, (int(cx1), int(cy1)), 5, (0, 255, 255), -1)
    cv2.circle(img, (int(cx2), int(cy2)), 5, (0, 255, 255), -1)

    # ===== 水平偏移量 =====
    h_offset = line_center_x - img_center_x
    print(f"\n===== 偏移计算 =====")
    print(f"线段中心点: ({line_center_x:.2f}, {line_center_y:.2f})")
    print(f"图像垂直中线 x: {img_center_x:.2f}")
    print(f"水平偏移量: {h_offset:.2f} px ({'偏右' if h_offset > 0 else '偏左' if h_offset < 0 else '居中'})")

    # 画图像中线（绿色虚线）
    cv2.line(img, (int(img_center_x), 0), (int(img_center_x), img_h), (0, 255, 0), 1)
    # 画水平偏移线
    cv2.line(img, (int(img_center_x), int(line_center_y)),
             (int(line_center_x), int(line_center_y)), (255, 255, 0), 2)
    cv2.circle(img, (int(img_center_x), int(line_center_y)), 5, (0, 255, 0), -1)
    cv2.putText(img, f'h_offset: {h_offset:.1f}px',
                (int(min(img_center_x, line_center_x))+5, int(line_center_y)-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    # ===== 斜率 =====
    dx = cx2 - cx1
    dy = cy2 - cy1
    angle_rad = np.arctan2(dy, dx)
    print(f"\n===== 斜率计算 =====")
    print(f"斜率线与水平线的夹角: {angle_rad:.4f} rad ({np.degrees(angle_rad):.2f} deg)")

    if abs(dx) < 1e-6:
        slope = float('inf')
        slope_text = 'slope: inf (vertical)'
    else:
        slope = dy / dx
        slope_text = f'slope: {slope:.2f}'
    print(f"线的斜率: {slope:.4f}")

    angle_text = f'angle: {angle_rad:.4f} rad ({np.degrees(angle_rad):.1f} deg)'
    cv2.putText(img, slope_text,
                (max(int((cx1+cx2)/2)-80, 10), max(int((cy1+cy2)/2)-15, 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.putText(img, angle_text,
                (max(int((cx1+cx2)/2)-80, 10), max(int((cy1+cy2)/2)+20, 30)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 200), 2)

    # 画水平参考线
    ref_len = np.sqrt(dx**2 + dy**2)
    cv2.line(img, (int(cx1), int(cy1)),
             (int(cx1 + ref_len), int(cy1)), (180, 180, 180), 1)

    return {
        'pt1': (cx1, cy1),
        'pt2': (cx2, cy2),
        'label1': label1,
        'label2': label2,
        'line_center': (line_center_x, line_center_y),
        'img_center_x': img_center_x,
        'h_offset': h_offset,
        'slope': slope,
        'angle_rad': angle_rad,
        'angle_deg': np.degrees(angle_rad),
    }


# ===================== 深度值采样 =====================

def get_depth_at_pixel(depth_raw: np.ndarray, x: int, y: int, search_radius: int = 10) -> float:
    """
    获取深度图 (x, y) 位置的深度值 (mm)。
    如果坐标越界或深度为0（无效），则在周围 search_radius 范围内搜索最近的有效深度点。
    search_radius: 搜索半径，默认为10像素
    """
    h, w = depth_raw.shape[:2]
    
    # 首先尝试获取目标点深度
    if 0 <= x < w and 0 <= y < h:
        val = depth_raw[y, x]
        if val > 0:
            return float(val)
    
    # 目标点无效，进行邻近搜索（BFS方式）
    for r in range(1, search_radius + 1):
        # 搜索半径为 r 的菱形区域
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if abs(dx) + abs(dy) != r:  # 只搜索当前半径的边缘
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h:
                    val = depth_raw[ny, nx]
                    if val > 0:
                        return float(val)
    
    # 搜索范围内没有找到有效深度
    return -1.0


def get_average_depth(depth_raw: np.ndarray, x: int, y: int, radius: int = 5) -> float:
    """
    获取深度图 (x, y) 位置周围半径 radius 范围内的平均深度值 (mm)。
    只计算有效深度点（值 > 0）的平均值。
    radius: 搜索半径，默认为5像素
    """
    h, w = depth_raw.shape[:2]
    
    depths = []
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            # 计算到中心点的距离，只保留在半径范围内的点
            if dx * dx + dy * dy <= radius * radius:
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h:
                    val = depth_raw[ny, nx]
                    if val > 0:
                        depths.append(val)
    
    if len(depths) == 0:
        return -1.0
    
    return float(np.mean(depths))


# ===================== 主流程 =====================

def main():
    start_time_main = time.time()
    print("=" * 60)
    print("YOLO + head_depth.raw 深度采样")
    print("=" * 60)

    # 1. YOLO 检测
    print("\n[1] 运行 YOLO 检测...")
    result = yolo_detect(IMG_PATH)
    if result[0] is None:
        print("❌ YOLO 检测失败，退出")
        return

    (img, img_h, img_w, img_center_x, pt1, pt2,
     boxes_a, boxes_b, boxes_c, boxes_d) = result

    # 2. 画线、计算偏移和斜率
    print("\n[2] 画线 & 计算偏移 & 斜率...")
    calc = draw_and_calculate(img, img_h, img_w, img_center_x, pt1, pt2,
                               boxes_a, boxes_b, boxes_c, boxes_d)
    cx1, cy1 = calc['pt1']
    cx2, cy2 = calc['pt2']
    label1 = calc['label1']
    label2 = calc['label2']

    # 3. 保存 YOLO 标注结果
    out_rgb = 'yolo_depth_rgb.jpg'
    cv2.imwrite(out_rgb, img)
    print(f"\n✅ RGB 标注结果已保存: {out_rgb}")

    # 4. 从 head_depth.raw 读取深度
    print("\n[3] 从 head_depth.raw 读取深度数据...")
    depth_raw = load_depth_from_raw(DEPTH_RAW_PATH, DEPTH_SHAPE)

    if depth_raw is None:
        print("⚠️ 无深度数据，跳过深度采样")
        return

    # 5. 深度采样
    print("\n[4] 深度值采样...")

    # 点 a 左侧 DEPTH_OFFSET 个像素: (cx1 - DEPTH_OFFSET, cy1)，读取周围半径5像素的平均深度
    sample_a_left_x = int(cx1) 
    sample_a_left_y = int(cy1)+ DEPTH_OFFSET
    depth_a_left = get_average_depth(depth_raw, sample_a_left_x, sample_a_left_y, radius=2)

    # 点 b 右侧 DEPTH_OFFSET 个像素: (cx2 + DEPTH_OFFSET, cy2)，读取周围半径5像素的平均深度
    sample_b_right_x = int(cx2) 
    sample_b_right_y = int(cy2)+ DEPTH_OFFSET
    depth_b_right = get_average_depth(depth_raw, sample_b_right_x, sample_b_right_y, radius=2)

    # a 和 b 的中心点，读取周围半径5像素的平均深度
    sample_center_x = int((cx1 + cx2) / 2)
    sample_center_y = int((cy1 + cy2) / 2)
    depth_center = get_average_depth(depth_raw, sample_center_x, sample_center_y, radius=5)

    # 中心点对比
    depth_a_center = get_depth_at_pixel(depth_raw, int(cx1), int(cy1))
    depth_b_center = get_depth_at_pixel(depth_raw, int(cx2), int(cy2))

    print(f"\n===== 深度采样结果 =====")
    print(f"点 {label1} 中心 ({cx1:.1f}, {cy1:.1f}) 深度: {depth_a_center:.1f} mm")
    print(f"点 {label1} 左侧 {DEPTH_OFFSET}px ({sample_a_left_x}, {sample_a_left_y}) 周围半径5像素平均深度: {depth_a_left:.1f} mm")
    print(f"点 {label2} 中心 ({cx2:.1f}, {cy2:.1f}) 深度: {depth_b_center:.1f} mm")
    print(f"点 {label2} 右侧 {DEPTH_OFFSET}px ({sample_b_right_x}, {sample_b_right_y}) 周围半径5像素平均深度: {depth_b_right:.1f} mm")
    print(f"a-b 中心点 ({sample_center_x}, {sample_center_y}) 周围半径5像素平均深度: {depth_center:.1f} mm")

    # ===== 生成伪彩色深度图 =====
    valid_mask = depth_raw > 0
    if np.any(valid_mask):
        min_d = depth_raw[valid_mask].min()
        max_d = depth_raw[valid_mask].max()
        if max_d > min_d:
            normalized = ((depth_raw - min_d) / (max_d - min_d) * 255).astype(np.uint8)
        else:
            normalized = np.zeros_like(depth_raw, dtype=np.uint8)
    else:
        normalized = np.zeros_like(depth_raw, dtype=np.uint8)
    depth_colored = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)

    # 保存深度图
    cv2.imwrite('yolo_depth_depth.jpg', depth_colored)
    print(f"✅ 深度图已保存: yolo_depth_depth.jpg")

    # 6. 在 RGB 图上标注深度采样点
    # 点 a 左侧 DEPTH_OFFSET px（紫色圆点 + 箭头）
    if 0 <= sample_a_left_x < img_w and 0 <= sample_a_left_y < img_h:
        cv2.circle(img, (sample_a_left_x, sample_a_left_y), 5, (255, 0, 255), -1)
        cv2.putText(img, f'L{DEPTH_OFFSET}:{depth_a_left:.0f}mm',
                    (sample_a_left_x - 65, sample_a_left_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
        cv2.arrowedLine(img, (int(cx1), int(cy1)),
                        (sample_a_left_x, sample_a_left_y),
                        (255, 0, 255), 1, tipLength=0.3)

    # 点 b 右侧 DEPTH_OFFSET px（青色圆点 + 箭头）
    if 0 <= sample_b_right_x < img_w and 0 <= sample_b_right_y < img_h:
        cv2.circle(img, (sample_b_right_x, sample_b_right_y), 5, (255, 255, 0), -1)
        cv2.putText(img, f'R{DEPTH_OFFSET}:{depth_b_right:.0f}mm',
                    (sample_b_right_x + 5, sample_b_right_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        cv2.arrowedLine(img, (int(cx2), int(cy2)),
                        (sample_b_right_x, sample_b_right_y),
                        (255, 255, 0), 1, tipLength=0.3)

    # a-b 中心点（绿色圆点）
    if 0 <= sample_center_x < img_w and 0 <= sample_center_y < img_h:
        cv2.circle(img, (sample_center_x, sample_center_y), 6, (0, 255, 0), -1)
        cv2.putText(img, f'Center:{depth_center:.0f}mm',
                    (sample_center_x + 10, sample_center_y + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    out_rgb_with_depth = 'yolo_depth_rgb_with_depth.jpg'
    cv2.imwrite(out_rgb_with_depth, img)
    print(f"✅ 深度采样标注 RGB 已保存: {out_rgb_with_depth}")

    # 7. 在深度图上标注采样点
    depth_marked = depth_colored.copy()
    if 0 <= sample_a_left_x < img_w and 0 <= sample_a_left_y < img_h:
        cv2.circle(depth_marked, (sample_a_left_x, sample_a_left_y), 5, (255, 0, 255), -1)
        cv2.putText(depth_marked, f'{label1}_L{DEPTH_OFFSET}:{depth_a_left:.0f}mm',
                    (sample_a_left_x - 75, sample_a_left_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
    if 0 <= sample_b_right_x < img_w and 0 <= sample_b_right_y < img_h:
        cv2.circle(depth_marked, (sample_b_right_x, sample_b_right_y), 5, (255, 255, 0), -1)
        cv2.putText(depth_marked, f'{label2}_R{DEPTH_OFFSET}:{depth_b_right:.0f}mm',
                    (sample_b_right_x + 5, sample_b_right_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
    cv2.circle(depth_marked, (int(cx1), int(cy1)), 5, (0, 255, 255), -1)
    cv2.circle(depth_marked, (int(cx2), int(cy2)), 5, (0, 255, 255), -1)
    
    # a-b 中心点
    if 0 <= sample_center_x < img_w and 0 <= sample_center_y < img_h:
        cv2.circle(depth_marked, (sample_center_x, sample_center_y), 6, (0, 255, 0), -1)
        cv2.putText(depth_marked, f'Center:{depth_center:.0f}mm',
                    (sample_center_x + 10, sample_center_y + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imwrite('yolo_depth_depth_marked.jpg', depth_marked)
    print(f"✅ 深度图标注已保存: yolo_depth_depth_marked.jpg")

    # ===== 汇总 =====
    print("\n" + "=" * 60)
    print("📊 最终汇总")
    print("=" * 60)
    print(f"YOLO 模型: {MODEL_PATH}")
    print(f"输入图像: {IMG_PATH}")
    print(f"深度数据: {DEPTH_RAW_PATH} ({depth_raw.shape})")
    print(f"检测点: {label1}({calc['pt1']}), {label2}({calc['pt2']})")
    print(f"偏移信息:")
    print(f"  - 线段中心点: ({calc['line_center'][0]:.1f}, {calc['line_center'][1]:.1f})")
    print(f"  - 图像垂直中线 x: {calc['img_center_x']:.1f}")
    print(f"  - 水平偏移量: {calc['h_offset']:.1f} px")
    print(f"斜率信息:")
    print(f"  - 斜率: {calc['slope']:.4f}")
    print(f"  - 与水平夹角: {calc['angle_deg']:.2f} deg ({calc['angle_rad']:.4f} rad)")
    print(f"深度信息:")
    print(f"  - 点 {label1} 中心深度: {depth_a_center:.1f} mm")
    print(f"  - 点 {label1} 左侧 {DEPTH_OFFSET}px 周围半径5像素平均深度: {depth_a_left:.1f} mm")
    print(f"  - 点 {label2} 中心深度: {depth_b_center:.1f} mm")
    print(f"  - 点 {label2} 右侧 {DEPTH_OFFSET}px 周围半径5像素平均深度: {depth_b_right:.1f} mm")
    print(f"  - a-b 中心点 ({sample_center_x}, {sample_center_y}) 周围半径5像素平均深度: {depth_center:.1f} mm")
    print(f"输出文件:")
    print(f"  - {out_rgb}")
    print(f"  - {out_rgb_with_depth}")
    print(f"  - yolo_depth_depth.jpg")
    print(f"  - yolo_depth_depth_marked.jpg")
    print(f"  - result.jpg (YOLO 默认输出)")
    print("=" * 60)

    # ===== 输出诊断结果到 JSON =====
    result_json_path = 'yolo_depth_result.json'
    result_data = {
        "model_path": MODEL_PATH,
        "image_path": IMG_PATH,
        "depth_raw_path": DEPTH_RAW_PATH,
        "depth_offset_px": DEPTH_OFFSET,
        "depth_shape": list(depth_raw.shape),
        "image_size": {"height": img_h, "width": img_w},
        "detection": {
            "point1": {
                "label": label1,
                "center": [round(float(cx1), 2), round(float(cy1), 2)],
            },
            "point2": {
                "label": label2,
                "center": [round(float(cx2), 2), round(float(cy2), 2)],
            },
        },
        "offset": {
            "line_center": [round(float(calc['line_center'][0]), 2), round(float(calc['line_center'][1]), 2)],
            "image_center_x": round(float(calc['img_center_x']), 2),
            "horizontal_offset_px": round(float(calc['h_offset']), 2),
            "direction": "偏右" if calc['h_offset'] > 0 else ("偏左" if calc['h_offset'] < 0 else "居中"),
        },
        "slope": {
            "slope": None if np.isinf(calc['slope']) else round(float(calc['slope']), 4),
            "angle_rad": round(float(calc['angle_rad']), 4),
            "angle_deg": round(float(calc['angle_deg']), 2),
        },
        "depth": {
            "point1_center_mm": round(float(depth_a_center), 1),
            "point1_left_offset_mm": round(float(depth_a_left), 1),
            "point1_left_sample_pixel": [int(sample_a_left_x), int(sample_a_left_y)],
            "point2_center_mm": round(float(depth_b_center), 1),
            "point2_right_offset_mm": round(float(depth_b_right), 1),
            "point2_right_sample_pixel": [int(sample_b_right_x), int(sample_b_right_y)],
            "center_mm": round(float(depth_center), 1),
            "center_sample_pixel": [int(sample_center_x), int(sample_center_y)],
        },
        "output_files": [
            out_rgb,
            out_rgb_with_depth,
            "yolo_depth_depth.jpg",
            "yolo_depth_depth_marked.jpg",
            "result.jpg",
        ],
    }

    with open(result_json_path, "w", encoding="utf-8") as f:
        json.dump(result_data, f, ensure_ascii=False, indent=2)
    print(f"✅ 诊断结果已保存: {result_json_path}")

    inference_time = time.time() - start_time_main
    print(f"总耗时: {inference_time:.4f} 秒")

if __name__ == '__main__':
    main()
