#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""draw.py — 绘图工具模块

功能:
  - draw_top_bar(): 图像顶部添加信息栏
  - draw_geometry(): 绘制 a/b 连线、中点、中垂线、图像中心线
  - draw_annotated(): 根据检测框绘制标注图
  - save_step_final(): 保存纠偏阶段结果图

参考: /home/agi/wzd/chassis_correct_all.py:577-638
"""
import os
import time

import cv2
import numpy as np

# ── 默认颜色 (BGR), 可被 correct.py 覆盖 ──
COLOR_LINE_AB = (0, 255, 0)
COLOR_MID = (0, 0, 255)
COLOR_PERP = (255, 0, 0)
COLOR_CENTER = (255, 255, 0)
COLOR_TEXT = (255, 255, 255)

# ── 图像中心 x (用于左右纠偏目标) ──
LR_TARGET_X = 320


def configure(cfg):
    """注入颜色配置 (由 main.py 启动时调用)

    cfg 应包含 colors 段和 lr.target_x
    """
    global COLOR_LINE_AB, COLOR_MID, COLOR_PERP, COLOR_CENTER, COLOR_TEXT, LR_TARGET_X

    colors = cfg.get("colors", {})
    def _rgb_to_bgr(rgb_list):
        """[r,g,b] → (b,g,r) tuple"""
        if isinstance(rgb_list, (list, tuple)) and len(rgb_list) == 3:
            r, g, b = rgb_list
            return (int(b), int(g), int(r))
        return None

    c = _rgb_to_bgr(colors.get("line_ab", [0, 255, 0]))
    if c: COLOR_LINE_AB = c
    c = _rgb_to_bgr(colors.get("mid_point", [0, 0, 255]))
    if c: COLOR_MID = c
    c = _rgb_to_bgr(colors.get("perp_line", [255, 0, 0]))
    if c: COLOR_PERP = c
    c = _rgb_to_bgr(colors.get("center", [255, 255, 0]))
    if c: COLOR_CENTER = c
    c = _rgb_to_bgr(colors.get("text", [255, 255, 255]))
    if c: COLOR_TEXT = c


def set_lr_target_x(x):
    """设置左右纠偏目标 x (由 correct.py 初始化时调用)"""
    global LR_TARGET_X
    LR_TARGET_X = x


# ═══════════════════════════════════════════════════════════
#  顶部信息栏
# ═══════════════════════════════════════════════════════════

def draw_top_bar(img, lines):
    """在图像顶部添加数据栏 (支持 1~N 行)

    Args:
        img: BGR 图像
        lines: 信息文本列表, 如 ["FB iter:1 delta:+12mm", "gain:1.0"]

    Returns:
        添加了顶部栏的新图像 (不修改原图)
    """
    h, w = img.shape[:2]
    n = max(len(lines), 1)
    bar_h = 24 * n + 12
    bar = np.zeros((bar_h, w, 3), dtype=np.uint8)
    for i, line in enumerate(lines):
        cv2.putText(bar, line, (12, 24 + i * 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_TEXT, 2)
    return np.vstack([bar, img])


# ═══════════════════════════════════════════════════════════
#  几何标注
# ═══════════════════════════════════════════════════════════

def draw_geometry(img, best, mid_x, mid_y, info_lines):
    """绘制 a/b 连线、中点、中垂线、图像中心竖直线

    Args:
        img: BGR 图像 (会被修改, 建议传入 copy)
        best: 检测结果 dict, 包含 best["a"]["cx"] 等
        mid_x, mid_y: a/b 中点坐标
        info_lines: 顶部信息栏文本列表

    Returns:
        添加了几何元素和顶部栏的图像
    """
    h, w = img.shape[:2]
    a_cx, a_cy = best["a"]["cx"], best["a"]["cy"]
    b_cx, b_cy = best["b"]["cx"], best["b"]["cy"]

    # a/b 连线
    cv2.line(img, (int(a_cx), int(a_cy)), (int(b_cx), int(b_cy)), COLOR_LINE_AB, 2)

    # 中点
    cv2.circle(img, (int(mid_x), int(mid_y)), 6, COLOR_MID, -1)

    # 中垂线 (与 a/b 连线垂直)
    dx, dy = b_cx - a_cx, b_cy - a_cy
    length = max(np.sqrt(dx * dx + dy * dy), 1e-6)
    perp_x, perp_y = -dy / length, dx / length
    L = max(h, w)
    cv2.line(img, (int(mid_x - perp_x * L), int(mid_y - perp_y * L)),
             (int(mid_x + perp_x * L), int(mid_y + perp_y * L)), COLOR_PERP, 1)

    # 图像中心竖直线 (左右纠偏目标)
    cv2.line(img, (LR_TARGET_X, 0), (LR_TARGET_X, h), COLOR_CENTER, 1)

    return draw_top_bar(img, info_lines)


# ═══════════════════════════════════════════════════════════
#  检测框标注
# ═══════════════════════════════════════════════════════════

def draw_annotated(img_bgr, detections):
    """根据检测框列表绘制标注图

    Args:
        img_bgr: 原始 BGR 图像
        detections: 检测结果列表, 每项包含 cls, conf, x1, y1, x2, y2

    Returns:
        标注后的图像副本 (不修改原图)
    """
    annotated = img_bgr.copy()
    color_map = {"a": (0, 255, 0), "b": (0, 165, 255)}
    for det in detections:
        cls = det.get("cls", "?")
        color = color_map.get(cls, (255, 0, 0))
        x1, y1 = int(det["x1"]), int(det["y1"])
        x2, y2 = int(det["x2"]), int(det["y2"])
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
        label = f"{cls} {det.get('conf', 0):.2f}"
        cv2.putText(annotated, label, (x1, max(y1 - 5, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return annotated


# ═══════════════════════════════════════════════════════════
#  结果图保存
# ═══════════════════════════════════════════════════════════

def save_step_final(output_dir, prefix, iteration, img, annotated, info_lines, status=""):
    """保存纠偏阶段最终结果图 (不论成功失败都输出)

    Args:
        output_dir: 输出目录
        prefix: 文件名前缀, 如 "01_fb"
        iteration: 当前迭代次数
        img: 原始彩色图 (annotated 为 None 时使用)
        annotated: YOLO 标注图 (优先使用)
        info_lines: 顶部信息栏文本列表
        status: 状态后缀, 如 "no_ab", "depth_invalid", "max_iter" (空=成功收敛)

    Returns:
        保存的文件路径, 或 None (无图像可保存)
    """
    ts = time.strftime("%H%M%S")
    base = annotated if annotated is not None else img
    if base is None:
        print(f"[保存] ⚠ 无图像可保存 ({prefix} iter{iteration})")
        return None

    final_img = base.copy()
    if info_lines:
        final_img = draw_top_bar(final_img, info_lines)

    status_suffix = f"_{status}" if status else ""
    final_path = os.path.join(output_dir, f"{prefix}_iter{iteration}_{ts}{status_suffix}.jpg")
    cv2.imwrite(final_path, final_img)
    print(f"[保存] 结果图已保存: {final_path}")
    return final_path
