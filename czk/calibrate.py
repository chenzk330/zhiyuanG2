#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""calibrate.py — 底盘横移标定脚本

标定 "像素 ↔ 米" 线性关系, 用于左右纠偏的 PX_TO_METER 系数。

流程:
  1. 在起点拍照 → 记录 a/b 中点 x₀
  2. 底盘按预设序列横移 → 每个位置拍照 → 记录中点 xᵢ
  3. 回到起点
  4. 最小二乘拟合 Δpx = k * Δy_m + b
  5. PX_TO_METER = 1/k

默认行为:
  - 不带 --rebuild 时, 只读取并显示当前 config/correct.yaml 中的 PX_TO_METER 值
  - --rebuild 时执行完整标定流程, 完成后自动写回 yaml

参考: /home/agi/wzd/chassis_lr_calibrate.py
"""
import argparse
import csv
import json
import os
import sys
import time

import cv2
import numpy as np

# 加载本目录模块
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import camera
import detector
import chassis

# ── 默认标定参数 ──
DEFAULT_OFFSETS = [-0.2, -0.1, 0.0, 0.1, 0.2]  # 横移偏移序列 (米), 正=左
DEFAULT_SETTLE_TIME = 1.5  # 移动后稳定时间


# ═══════════════════════════════════════════════════════════
#  YAML 读写
# ═══════════════════════════════════════════════════════════

def _load_yaml(path):
    """加载 YAML 配置文件 (使用 PyYAML)"""
    import yaml
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _update_px_to_meter_in_yaml(yaml_path, new_value, timestamp):
    """更新 yaml 中的全局 px_to_meter 值 (保留注释, 只改值和注释)"""
    with open(yaml_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    updated = False
    for i, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith("px_to_meter:") and not stripped.startswith("#"):
            lines[i] = f"  px_to_meter: {new_value:.6f}       # m/px (标定更新于 {timestamp})\n"
            updated = True
            break

    if updated:
        with open(yaml_path, "w", encoding="utf-8") as f:
            f.writelines(lines)
        print(f"[标定] 已更新 {yaml_path} 中的 px_to_meter = {new_value:.6f}")
    else:
        print(f"[标定] ⚠ 未找到 px_to_meter 配置项, 未更新 yaml")


def update_scene_px_to_meter(yaml_path, scene_name, new_value, timestamp, target_depth=None):
    """更新 yaml 中 scenes.{scene}.px_to_meter_override (场景级标定值)

    在场景块中查找/新增 px_to_meter_override 行。
    若提供 target_depth, 同步写入 px_to_meter_calib_depth, 用于后续判定"目标
    深度是否改变"以决定是否跳过重新标定。
    """
    with open(yaml_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    scene_indent = None      # 场景块的缩进 (如 "  pick:" → 2空格)
    scene_block_indent = None  # 场景属性的缩进 (如 "    target_depth:" → 4空格)
    scene_start = -1
    scene_end = len(lines)

    # 1. 找到目标场景块的范围
    for i, line in enumerate(lines):
        stripped = line.lstrip()
        indent_len = len(line) - len(stripped)
        if stripped.startswith(f"{scene_name}:") and indent_len == 2:
            scene_start = i
            scene_indent = indent_len
            scene_block_indent = indent_len + 2
            break

    if scene_start < 0:
        print(f"[标定] ⚠ 未找到场景 {scene_name}, 无法写入 px_to_meter_override")
        return False

    # 2. 找场景块结束 (下一个同级场景或顶级key)
    for j in range(scene_start + 1, len(lines)):
        line = lines[j]
        stripped = line.lstrip()
        indent_len = len(line) - len(stripped)
        if stripped and not stripped.startswith("#") and indent_len <= scene_indent and stripped != f"{scene_name}:":
            scene_end = j
            break

    indent = " " * scene_block_indent

    # 3. 更新 px_to_meter_override
    px_pos = None
    for k in range(scene_start + 1, scene_end):
        stripped = lines[k].strip()
        if stripped.startswith("px_to_meter_override:"):
            px_pos = k
            break
    px_line = f"{indent}px_to_meter_override: {new_value:.6f}   # m/px (场景标定于 {timestamp})\n"
    if px_pos is not None:
        lines[px_pos] = px_line
    else:
        lines.insert(scene_end, px_line)
        scene_end += 1

    # 4. 同步写入 px_to_meter_calib_depth (标定时目标深度)
    if target_depth is not None:
        calib_pos = None
        for k in range(scene_start + 1, scene_end):
            stripped = lines[k].strip()
            if stripped.startswith("px_to_meter_calib_depth:"):
                calib_pos = k
                break
        calib_line = f"{indent}px_to_meter_calib_depth: {target_depth}   # 标定时的目标深度mm (目标深度改变时自动重新标定)\n"
        if calib_pos is not None:
            lines[calib_pos] = calib_line
        else:
            lines.insert(scene_end, calib_line)

    with open(yaml_path, "w", encoding="utf-8") as f:
        f.writelines(lines)
    print(f"[标定] 已更新 scenes.{scene_name}.px_to_meter_override = {new_value:.6f}" +
          (f", calib_depth={target_depth}" if target_depth is not None else ""))
    return True


# ═══════════════════════════════════════════════════════════
#  标定主流程
# ═══════════════════════════════════════════════════════════

def run_calibration(model, g2, offsets, output_dir, settle_time):
    """执行标定流程

    Returns:
        list: [(offset_m, mid_x, mid_y, img_path), ...] 或 None
    """
    records = []
    cumulative_y = 0.0

    print(f"\n{'=' * 60}")
    print(f"开始标定: {len(offsets)} 个点位")
    print(f"偏移序列 (米): {offsets}")
    print(f"{'=' * 60}")

    # 起点拍照
    print(f"\n[起点] 拍照中...")
    img0 = camera.capture_color()
    best, annotated = detector.detect_ab(model, img0, strict_ab=True)
    if best is None:
        print(f"[起点] ✗ 未检测到 a/b, 终止标定")
        return None

    mid_x0 = (best["a"]["cx"] + best["b"]["cx"]) / 2
    mid_y0 = (best["a"]["cy"] + best["b"]["cy"]) / 2
    print(f"[起点] 中点=({mid_x0:.1f}, {mid_y0:.1f})")

    # 保存起点图
    raw_path = os.path.join(output_dir, "calib_0_origin.jpg")
    cv2.imwrite(raw_path, img0)
    if annotated is not None:
        ann_path = os.path.join(output_dir, "calib_0_origin_annotated.jpg")
        cv2.imwrite(ann_path, annotated)

    records.append((0.0, mid_x0, mid_y0, raw_path))

    # 按序列横移
    for i, offset in enumerate(offsets, 1):
        if offset == 0.0:
            records.append((0.0, mid_x0, mid_y0, raw_path))
            print(f"\n[{i}/{len(offsets)}] offset=0.0m  (复用起点)")
            continue

        print(f"\n[{i}/{len(offsets)}] 目标: 横移 {offset:+.3f} m ...")

        # 回原点
        if abs(cumulative_y) > 1e-6:
            print(f"[{i}] 回原点 (反向 {-cumulative_y:+.3f} m)...")
            chassis.move_chassis(g2, dy_m=-cumulative_y)
            time.sleep(settle_time)
            cumulative_y = 0.0

        # 移动到目标
        print(f"[{i}] 从原点移动到 {offset:+.3f} m...")
        ok = chassis.move_chassis(g2, dy_m=offset)
        if not ok:
            print(f"[{i}] ✗ 横移失败, 跳过")
            continue
        cumulative_y = offset
        time.sleep(settle_time)

        # 拍照
        print(f"[{i}] 拍照中...")
        img = camera.capture_color()
        best, annotated = detector.detect_ab(model, img, strict_ab=True)
        if best is None:
            print(f"[{i}] ✗ 未检测到 a/b, 跳过")
            cv2.imwrite(os.path.join(output_dir, f"calib_{i}_fail_offset{offset:+.3f}.jpg"), img)
            continue

        mid_x = (best["a"]["cx"] + best["b"]["cx"]) / 2
        mid_y = (best["a"]["cy"] + best["b"]["cy"]) / 2
        print(f"[{i}] offset={offset:+.3f}m  中点=({mid_x:.1f}, {mid_y:.1f})")

        raw_path_i = os.path.join(output_dir, f"calib_{i}_offset{offset:+.3f}.jpg")
        cv2.imwrite(raw_path_i, img)
        if annotated is not None:
            ann_path_i = os.path.join(output_dir, f"calib_{i}_offset{offset:+.3f}_annotated.jpg")
            cv2.imwrite(ann_path_i, annotated)

        records.append((offset, mid_x, mid_y, raw_path_i))

    # 回到起点
    if cumulative_y != 0.0:
        print(f"\n[回退] 反向移动 {cumulative_y:.3f} m 回到起点...")
        chassis.move_chassis(g2, dy_m=-cumulative_y)
        time.sleep(settle_time)
        print("[回退] 完成")

    return records


def analyze_and_save(records, output_dir, yaml_path):
    """拟合 Δpx = k * Δy_m + b, 保存结果"""
    if len(records) < 2:
        print(f"\n[分析] 有效数据点不足 ({len(records)}/2), 无法拟合")
        return None

    base = next((r for r in records if abs(r[0]) < 1e-6), records[0])
    base_x = base[1]

    offsets_m = []
    delta_px = []
    for offset_m, mid_x, mid_y, _ in records:
        offsets_m.append(offset_m)
        delta_px.append(mid_x - base_x)

    x = np.array(offsets_m)
    y = np.array(delta_px)
    k, b = np.polyfit(x, y, 1)
    y_pred = k * x + b
    residuals = y - y_pred
    rmse = np.sqrt(np.mean(residuals ** 2))
    ss_res = np.sum(residuals ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0

    px_to_meter = 1.0 / k if abs(k) > 1e-9 else float("inf")
    timestamp = time.strftime("%Y%m%d_%H%M%S")

    print(f"\n{'=' * 60}")
    print(f"标定结果")
    print(f"{'=' * 60}")
    print(f"数据点数: {len(records)}")
    print(f"基准点 (offset=0): mid_x = {base_x:.2f} px")
    print(f"线性拟合: Δpx = {k:.2f} * Δy_m + ({b:.2f})")
    print(f"  斜率 k = {k:.2f} px/m   (底盘左移 1m → 中点 x 增加 {k:.2f} px)")
    print(f"  截距 b = {b:.2f} px")
    print(f"  RMSE   = {rmse:.2f} px")
    print(f"  R²     = {r2:.4f}")
    print(f"")
    print(f"像素 → 米 转换系数 (1/k):")
    print(f"  PX_TO_METER = {px_to_meter:.6f} m/px")
    print(f"  即 1 像素 ≈ {abs(px_to_meter) * 1000:.3f} 毫米")
    print(f"  注: 正负号表示方向, 实际使用时根据 go_rel 的 y 方向定义取负号")
    print(f"{'=' * 60}")

    # 保存 CSV
    csv_path = os.path.join(output_dir, "calibration_data.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "offset_m", "mid_x_px", "mid_y_px", "delta_px_from_base", "image_path"])
        for i, (offset_m, mid_x, mid_y, img_path) in enumerate(records):
            writer.writerow([i, f"{offset_m:.4f}", f"{mid_x:.2f}", f"{mid_y:.2f}",
                             f"{mid_x - base_x:.2f}", img_path])
    print(f"\nCSV 已保存: {csv_path}")

    # 保存结果 JSON
    result = {
        "timestamp": timestamp,
        "num_points": len(records),
        "base_mid_x_px": base_x,
        "fit": {
            "slope_k_px_per_m": float(k),
            "intercept_b_px": float(b),
            "rmse_px": float(rmse),
            "r_squared": float(r2),
        },
        "px_to_meter": float(px_to_meter),
    }
    json_path = os.path.join(output_dir, "calibration_result.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print(f"结果 JSON 已保存: {json_path}")

    # 更新 yaml
    if yaml_path and os.path.exists(yaml_path):
        _update_px_to_meter_in_yaml(yaml_path, float(px_to_meter), timestamp)

    return result


# ═══════════════════════════════════════════════════════════
#  内联标定 (供 correct.py 在 LR 纠偏前调用)
# ═══════════════════════════════════════════════════════════

def run_inline_calibration(model, g2, offsets=None, settle_time=None, output_dir=None):
    """内联标定: 在当前位置标定 px_to_meter, 标定后回到原点

    用于 correct.run_correct 在 FB+YAW 收敛后、LR 之前调用。
    前提: 机器人已处于目标深度 (FB 已收敛)。

    Args:
        model: YOLO 模型
        g2: minth.G2 实例
        offsets: 横移偏移序列(米), 正=左 负=右 (默认 DEFAULT_OFFSETS)
        settle_time: 移动后稳定时间 (默认 DEFAULT_SETTLE_TIME)
        output_dir: 标定图保存目录 (默认 czk/calibration/)

    Returns:
        float: px_to_meter (m/px), 标定失败返回 None
    """
    if offsets is None:
        offsets = DEFAULT_OFFSETS
    if settle_time is None:
        settle_time = DEFAULT_SETTLE_TIME
    if output_dir is None:
        output_dir = os.path.join(_HERE, "calibration")
    os.makedirs(output_dir, exist_ok=True)

    offsets = sorted(set(offsets))

    print(f"\n{'=' * 60}")
    print(f"[LR前标定] 在当前深度标定 px_to_meter")
    print(f"  偏移序列: {offsets} m")
    print(f"  稳定时间: {settle_time}s")
    print(f"{'=' * 60}")

    # 执行标定流程 (起点拍照 → 横移序列 → 回原点)
    records = run_calibration(model, g2, offsets, output_dir, settle_time)
    if not records or len(records) < 2:
        print(f"[LR前标定] ✗ 有效数据点不足 ({len(records) if records else 0}/2)")
        return None

    # 拟合 Δpx = k * Δy_m + b
    base = next((r for r in records if abs(r[0]) < 1e-6), records[0])
    base_x = base[1]
    offsets_m = np.array([r[0] for r in records])
    delta_px = np.array([r[1] - base_x for r in records])

    k, b = np.polyfit(offsets_m, delta_px, 1)
    if abs(k) < 1e-9:
        print(f"[LR前标定] ✗ 斜率过小 (k={k:.4f}), 标定失败")
        return None

    px_to_meter = 1.0 / k

    # 拟合质量
    y_pred = k * offsets_m + b
    residuals = delta_px - y_pred
    rmse = float(np.sqrt(np.mean(residuals ** 2)))
    ss_res = float(np.sum(residuals ** 2))
    ss_tot = float(np.sum((delta_px - np.mean(delta_px)) ** 2))
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0

    print(f"\n[LR前标定] ✓ 标定完成")
    print(f"  数据点: {len(records)}")
    print(f"  斜率 k = {k:.2f} px/m   (底盘左移1m → 中点x增加 {k:.2f}px)")
    print(f"  RMSE  = {rmse:.2f} px")
    print(f"  R²    = {r2:.4f}")
    print(f"  px_to_meter = {px_to_meter:.6f} m/px  (1像素 ≈ {abs(px_to_meter)*1000:.3f}mm)")
    print(f"{'=' * 60}")

    return float(px_to_meter)


# ═══════════════════════════════════════════════════════════
#  主函数
# ═══════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="底盘横移标定: 像素 ↔ 米")
    parser.add_argument("--rebuild", action="store_true",
                        help="执行完整标定流程 (默认只读取当前值)")
    parser.add_argument("--model", default=None,
                        help="YOLO ref 模型路径 (默认用 config 中 pick 场景的模型)")
    parser.add_argument("--output", default=None,
                        help="标定结果输出目录 (默认 /home/agi/czk/calibration)")
    parser.add_argument("--offsets", type=float, nargs="+", default=DEFAULT_OFFSETS,
                        help=f"横移偏移序列(米), 正=左 负=右 (默认: {DEFAULT_OFFSETS})")
    parser.add_argument("--settle", type=float, default=DEFAULT_SETTLE_TIME,
                        help=f"移动后稳定时间(秒) (默认: {DEFAULT_SETTLE_TIME})")
    args = parser.parse_args()

    # 加载配置
    yaml_path = os.path.join(_HERE, "config", "correct.yaml")
    cfg = _load_yaml(yaml_path)

    # 配置各模块
    camera.configure(cfg.get("common", {}))
    detector.configure(cfg.get("yolo", {}))
    chassis.configure(cfg.get("common", {}))

    # 读取当前值
    lr_cfg = cfg.get("lr", {})
    current_px_to_m = lr_cfg.get("px_to_meter", 0.002584)
    print(f"\n[当前配置] PX_TO_METER = {current_px_to_m} m/px")
    print(f"           即 1 像素 ≈ {abs(current_px_to_m) * 1000:.3f} 毫米")

    if not args.rebuild:
        print(f"\n(如需重新标定, 请加 --rebuild 参数)")
        print(f"(标定会移动底盘 ±0.2m, 请确保机器人周围有足够空间)")
        return 0

    # ── 执行标定 ──
    model_path = args.model
    if model_path is None:
        # 用 pick 场景的模型
        scenes = cfg.get("scenes", {})
        pick_scene = scenes.get("pick", {})
        model_rel = pick_scene.get("model_ref", "models/best_new.ref")
        model_path = os.path.join(_HERE, model_rel) if not os.path.isabs(model_rel) else model_rel

    output_dir = args.output or os.path.join(_HERE, "calibration")
    os.makedirs(output_dir, exist_ok=True)

    if not os.path.isfile(model_path):
        raise FileNotFoundError(f"模型文件不存在: {model_path}")

    # 加载模型
    print(f"\n[1] 加载 YOLO 模型: {model_path}")
    model = detector.load_model(model_path)

    # 连接机器人
    print(f"\n[2] 连接机器人...")
    g2 = chassis.setup_minth()
    print(f"    ✓ Minth 已就绪")

    # 偏移序列去重并排序
    offsets = sorted(set(args.offsets))
    print(f"[3] 标定偏移序列: {offsets} m")

    # 执行标定
    records = run_calibration(model, g2, offsets, output_dir, args.settle)

    # 释放连接
    g2.close()

    # 分析并保存
    if records:
        result = analyze_and_save(records, output_dir, yaml_path)
        print(f"\n标定完成. 所有文件保存在: {output_dir}")
        if result:
            print(f"\n★ 新的 PX_TO_METER = {result['px_to_meter']:.6f} m/px")
            print(f"  已自动更新到 config/correct.yaml")
    else:
        print(f"\n标定失败, 未获得有效数据")


if __name__ == "__main__":
    main()
