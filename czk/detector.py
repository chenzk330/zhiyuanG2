#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""detector.py — YOLO 推理模块 (辉羲 RPU 芯片)

设计要点 (避免 wzd 双版本冲突):
  - RhinoInfer 作为纯推理引擎使用, 只调用其 infer() 方法返回 detections 列表
  - detect_ab() 逻辑在本模块统一实现, 不调用 RhinoInfer.detect_ab
  - 位置回退采用 chassis_correct_all.py 的更宽松版本 (y_diff_factor=2.0)
  - 多阈值回退: 0.25 → 0.15 → 0.10

参考:
  - /home/agi/wzd/rhino_infer.py (RhinoInfer 推理引擎)
  - /home/agi/wzd/chassis_correct_all.py:418-541 (detect_ab 逻辑, 取其更宽松版本)
"""
import os
import sys

# ── 加载本目录的 rhino_infer 模块 ──
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
from rhino_infer import RhinoInfer

# ── 全局配置 ──
_config = None


def configure(cfg):
    """注入配置 (由 main.py 启动时调用)

    cfg 应包含:
      - yolo.imgsz, conf, min_conf, iou
      - yolo.conf_fallback (多阈值回退序列)
      - yolo.position_fallback (位置回退参数)
    """
    global _config
    _config = cfg


def _get(key, default=None):
    if _config is None:
        return default
    return _config.get(key, default)


# ═══════════════════════════════════════════════════════════
#  模型加载
# ═══════════════════════════════════════════════════════════

def load_model(model_ref_path):
    """加载辉羲 RPU 推理模型

    Args:
        model_ref_path: .ref 模型文件绝对路径

    Returns:
        RhinoInfer 实例

    Raises:
        FileNotFoundError: 模型或推理工具不存在
    """
    if not os.path.isfile(model_ref_path):
        raise FileNotFoundError(f"REF 模型不存在: {model_ref_path}")

    print(f"[推理] 加载辉羲 RPU 模型: {model_ref_path}")
    engine = RhinoInfer(model_ref_path)
    print(f"[推理] ✓ 辉羲 RPU 已就绪 (含预热)")
    return engine


# ═══════════════════════════════════════════════════════════
#  统一 detect_ab 实现
# ═══════════════════════════════════════════════════════════

def _build_best(detections):
    """从 detections 列表中按类别取最高置信度, 构建 best_dict

    Args:
        detections: [{cls, conf, x1, y1, x2, y2, cx, cy}, ...]

    Returns:
        best: {"a": {...}, "b": {...}} (每个类别取最高 conf)
    """
    best = {}
    for det in detections:
        cls = det["cls"]
        if cls not in best or det["conf"] > best[cls]["conf"]:
            best[cls] = det
    return best


def _position_fallback(all_detections, min_conf, y_diff_factor, x_dist_min):
    """位置回退: a/b 缺失时, 取 top2 高置信度框水平排列判定

    判定条件:
      - top2 的 conf 均 >= min_conf
      - y_diff < max(avg_h * y_diff_factor, 60)  (两框近似水平排列)
      - x_dist > x_dist_min  (水平距离足够远)

    左=a, 右=b

    参考: chassis_correct_all.py:510-528 (更宽松版本, y_diff_factor=2.0)
    """
    if len(all_detections) < 2:
        return None

    sorted_by_conf = sorted(all_detections, key=lambda b: b["conf"], reverse=True)
    top2 = sorted_by_conf[:2]

    if top2[0]["conf"] < min_conf or top2[1]["conf"] < min_conf:
        return None

    x0, y0 = top2[0]["cx"], top2[0]["cy"]
    x1, y1 = top2[1]["cx"], top2[1]["cy"]
    avg_h = ((top2[0]["y2"] - top2[0]["y1"]) + (top2[1]["y2"] - top2[1]["y1"])) / 2
    y_diff = abs(y0 - y1)
    x_dist = abs(x0 - x1)

    # 放宽: 相对阈值 (factor 倍框高) 或 绝对阈值 (60px) 取大者
    y_threshold = max(avg_h * y_diff_factor, 60.0)
    if y_diff < y_threshold and x_dist > x_dist_min:
        left = top2[0] if x0 < x1 else top2[1]
        right = top2[1] if x0 < x1 else top2[0]
        return {"a": dict(left, cls="a"), "b": dict(right, cls="b")}
    return None


def detect_ab(model, img_bgr, conf=None, strict_ab=True, min_boxes=1):
    """YOLO 检测 a/b, 返回 (best_dict, annotated) 或 (None, annotated)

    统一实现, 不依赖 RhinoInfer.detect_ab 方法。

    Args:
        model: RhinoInfer 实例 (调用其 infer 方法)
        img_bgr: BGR 图像
        conf: 置信度阈值 (None 则用配置默认值 0.25)
        strict_ab: True=严格要求 a+b 同时存在 (角度纠偏用);
                    False=允许回退到 top2 (左右纠偏) 或 single (前后纠偏)
        min_boxes: strict_ab=False 回退时最少需要的检测框数 (FB=1, LR=2)

    Returns:
        (best, annotated):
          best: {"a": {...}, "b": {...}} 或 None
          annotated: 标注图 (即使检测失败也返回)

    ★ 多阈值回退: conf → 0.15 → 0.10 (不低于 min_conf)
    ★ 最低置信度过滤: conf < min_conf 的检测框直接丢弃
    ★ 位置回退: a/b 缺失且 strict_ab=True 时, top2 水平排列则左=a右=b
    """
    # 读取配置
    default_conf = _get("conf", 0.25)
    min_conf = _get("min_conf", 0.10)
    conf_fallback = _get("conf_fallback", [0.25, 0.15, 0.10])
    pf_cfg = _get("position_fallback", {})
    pf_enabled = pf_cfg.get("enabled", True)
    pf_min_conf = pf_cfg.get("min_conf", 0.25)
    pf_y_factor = pf_cfg.get("y_diff_factor", 2.0)
    pf_x_min = pf_cfg.get("x_dist_min", 30)

    if conf is None:
        conf = default_conf

    # 构建多阈值回退序列 (去重, 不低于 min_conf)
    conf_thresholds = []
    candidates = list(conf_fallback) if conf_fallback else [conf, 0.15, min_conf]
    if conf not in candidates:
        candidates.insert(0, conf)
    for c in candidates:
        if c >= min_conf and c not in conf_thresholds:
            conf_thresholds.append(c)
    if min_conf not in conf_thresholds:
        conf_thresholds.append(min_conf)

    last_detections = []
    last_annotated = None

    for try_conf in conf_thresholds:
        # 调用辉羲 RPU 推理引擎
        detections = model.infer(img_bgr, conf_threshold=try_conf)
        last_detections = detections

        # 绘制标注图
        import draw as _draw
        last_annotated = _draw.draw_annotated(img_bgr, detections)

        # 调试输出
        if detections:
            box_info = ", ".join([
                f"{d['cls']}={d['conf']:.2f}@({d['cx']:.0f},{d['cy']:.0f})"
                for d in detections
            ])
            print(f"  [检测 conf={try_conf}] {len(detections)}框: {box_info}")
        else:
            print(f"  [检测 conf={try_conf}] 0框")

        # 构建 best (按类别取最高 conf)
        best = _build_best(detections)
        if "a" in best and "b" in best:
            return best, last_annotated

    # ── 位置回退 (strict_ab=True): top2 高置信度框水平排列判定 ──
    if strict_ab and pf_enabled and len(last_detections) >= 2:
        fallback_best = _position_fallback(last_detections, pf_min_conf,
                                           pf_y_factor, pf_x_min)
        if fallback_best is not None:
            top2 = sorted(last_detections, key=lambda b: b["conf"], reverse=True)[:2]
            print(f"  [位置回退] 2个高置信度框水平排列 "
                  f"(conf={top2[0]['conf']:.2f}/{top2[1]['conf']:.2f}), 左=a 右=b")
            return fallback_best, last_annotated

    # ── 回退: topN / single (仅 strict_ab=False 时) ──
    if not strict_ab and len(last_detections) >= min_boxes:
        sorted_boxes = sorted(last_detections, key=lambda b: b["conf"], reverse=True)
        if len(sorted_boxes) >= 2:
            return {"a": sorted_boxes[0], "b": sorted_boxes[1]}, last_annotated
        else:
            only = sorted_boxes[0]
            print(f"  [检测回退] 仅检测到1个目标 ({only['cls']} conf={only['conf']:.2f}), a/b 复用同一框")
            return {"a": only, "b": only}, last_annotated

    return None, last_annotated
