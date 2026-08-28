#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""correct.py — 三步纠偏核心算法

顺序(默认): 前后(FB) → 角度(YAW) → 左右(LR); 可通过 run_correct(order=...) 自定义

★ 4 层安全机制 (必须完整保留, 不可简化):
  1. FB_MIN_DEPTH: 深度过近立即终止 (防撞架)
  2. safety_abort: 偏差异常增大时终止 (方向错误或外部干扰)
  3. stale_buffer_abort: 移动后深度不变时终止 (疑似取到旧图)
  4. FB_MAX_DEPTH_DIFF: a/b 深度差异常时判定误检

★ 自适应增益: 根据上次移动效果调整本次增益
★ 预移动/预旋转: 克服底盘静摩擦 (小距离时先反向再正向, 净位移不变)

参考: /home/agi/wzd/chassis_correct_all.py:645-1083
"""
import os
import time

import cv2
import numpy as np

import camera
import detector
import draw


# ═══════════════════════════════════════════════════════════
#  参数配置 (由 main.py 启动时注入)
# ═══════════════════════════════════════════════════════════

_fb_cfg = {}
_yaw_cfg = {}
_lr_cfg = {}
_common_cfg = {}


def configure(cfg):
    """注入纠偏参数配置

    cfg 应包含 fb, yaw, lr, common 段
    """
    global _fb_cfg, _yaw_cfg, _lr_cfg, _common_cfg
    _fb_cfg = cfg.get("fb", {})
    _yaw_cfg = cfg.get("yaw", {})
    _lr_cfg = cfg.get("lr", {})
    _common_cfg = cfg.get("common", {})

    # 同步 draw 模块的 LR_TARGET_X
    draw.set_lr_target_x(_lr_cfg.get("target_x", 320))


def _fb(key, default=None):
    return _fb_cfg.get(key, default)


def _yaw(key, default=None):
    return _yaw_cfg.get(key, default)


def _lr(key, default=None):
    return _lr_cfg.get(key, default)


def _common(key, default=None):
    return _common_cfg.get(key, default)


# ═══════════════════════════════════════════════════════════
#  纠偏步骤 1: 前后纠偏 (FB)
# ═══════════════════════════════════════════════════════════

def step_fb_correct(model, g2, target_depth, output_dir, dry_run=False):
    """前后纠偏: 通过深度图对齐目标距离

    流程:
      1. 拍彩色+深度图
      2. YOLO 检测 a/b
      3. 取 a/b 深度均值, 计算 delta = avg_depth - target_depth
      4. 4 层安全检查
      5. 收敛判断 (|delta| < threshold)
      6. 自适应增益调整
      7. 计算移动量 (带增益, 截断到 max_single_move)
      8. 预移动克服静摩擦
      9. 执行底盘运动

    Returns:
        dict: {success, step, converged, reason, color_img, a_depth, b_depth, avg_depth}
    """
    threshold = _fb("threshold", 5)
    max_iter = _fb("max_iter", 5)
    settle_time = _fb("settle_time", 2.0)
    max_single_move = _fb("max_single_move", 0.30)
    min_depth = _fb("min_depth", 400)
    max_depth_diff = _fb("max_depth_diff", 300)
    gain_init = _fb("gain", 1.0)
    gain_min = _fb("gain_min", 0.5)
    gain_max = _fb("gain_max", 3.0)
    pre_move_enabled = _fb("pre_move", True)
    pre_move_m = _fb("pre_move_m", 0.05)
    pre_move_threshold = _fb("pre_move_threshold", 0.02)
    warmup_wait = _common("warmup_wait", 0.2)

    print(f"\n{'=' * 60}")
    print(f"[步骤 1/3] 前后纠偏 (目标深度: {target_depth}mm)")
    print(f"{'=' * 60}")

    curr_gain = gain_init
    prev_delta = None
    prev_move_m = None
    last_move_dir = 0
    last_delta = None
    last_avg_d = None           # 上一次平均深度 (检测旧图缓冲)
    last_move_m_abs = 0.0       # 上一次实际移动距离绝对值

    for iteration in range(1, max_iter + 1):
        print(f"\n--- 前后纠偏 迭代 {iteration}/{max_iter} ---")

        # 移动后预拍照刷新相机缓冲 (第2次迭代起, 防止取到移动前的旧图)
        if iteration > 1:
            print(f"[FB-{iteration}] 预拍照刷新缓冲...")
            camera.capture_color_and_depth()
            time.sleep(warmup_wait)

        # 拍照 (彩色 + 深度, 最多重试3次)
        color_img, depth_2d = None, None
        for attempt in range(3):
            print(f"[FB-{iteration}] 拍照中... (第{attempt + 1}次)")
            color_img, depth_2d = camera.capture_color_and_depth()
            if color_img is not None and depth_2d is not None:
                break
            time.sleep(1.0)

        if color_img is None or depth_2d is None:
            print(f"[FB-{iteration}] ✗ 拍照失败")
            return {"success": False, "step": "fb", "reason": "拍照失败", "color_img": None}

        # YOLO 检测 (允许回退到 single, min_boxes=1)
        best, annotated = detector.detect_ab(model, color_img, strict_ab=False, min_boxes=1)
        if best is None:
            print(f"[FB-{iteration}] ✗ 未检测到 a/b")
            draw.save_step_final(output_dir, "01_fb", iteration, color_img, annotated,
                                 [f"FB iter:{iteration}  STATUS: NO_AB_DETECTED",
                                  f"reason: a/b not found in image"], status="no_ab")
            return {"success": False, "step": "fb", "reason": "未检测到 a/b", "color_img": color_img}

        # 深度值
        a_d = camera.get_depth_at_point(depth_2d, int(best["a"]["cx"]), int(best["a"]["cy"]))
        b_d = camera.get_depth_at_point(depth_2d, int(best["b"]["cx"]), int(best["b"]["cy"]))
        avg_d = (a_d + b_d) / 2
        delta = avg_d - target_depth

        print(f"[FB-{iteration}] a={a_d:.0f}mm b={b_d:.0f}mm avg={avg_d:.0f}mm delta={delta:+.0f}mm")

        # 保存每次迭代标注图
        ts = time.strftime("%H%M%S")
        iter_img = annotated.copy()
        cv2.circle(iter_img, (int(best["a"]["cx"]), int(best["a"]["cy"])), 4, (0, 0, 255), -1)
        cv2.circle(iter_img, (int(best["b"]["cx"]), int(best["b"]["cy"])), 4, (0, 0, 255), -1)
        cv2.putText(iter_img, f"a:{a_d:.0f}mm", (int(best["a"]["cx"]) + 8, int(best["a"]["cy"])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(iter_img, f"b:{b_d:.0f}mm", (int(best["b"]["cx"]) + 8, int(best["b"]["cy"])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        iter_img = draw.draw_top_bar(iter_img, [
            f"FB iter:{iteration} a:{a_d:.0f}mm b:{b_d:.0f}mm avg:{avg_d:.0f}mm delta:{delta:+.0f}mm"])
        cv2.imwrite(os.path.join(output_dir, f"01_fb_iter{iteration}_{ts}.jpg"), iter_img)

        # ── 安全检查 1: 深度值无效 ──
        if a_d == 0 or b_d == 0:
            print(f"[FB-{iteration}] ✗ 深度值无效")
            draw.save_step_final(output_dir, "01_fb", iteration, color_img, annotated,
                                 [f"FB iter:{iteration}  STATUS: DEPTH_INVALID",
                                  f"a:{a_d:.0f}mm b:{b_d:.0f}mm avg:{avg_d:.0f}mm (invalid)"],
                                 status="depth_invalid")
            return {"success": False, "step": "fb", "reason": "深度无效", "color_img": color_img}

        # ── 安全检查 4: a/b 深度差过大 (疑似误检) ──
        depth_diff = abs(a_d - b_d)
        if depth_diff > max_depth_diff:
            print(f"[FB-{iteration}] ⚠ a/b 深度差={depth_diff:.0f}mm > {max_depth_diff}mm, "
                  f"疑似误检 (a={a_d:.0f} b={b_d:.0f})")
            draw.save_step_final(output_dir, "01_fb", iteration, color_img, annotated,
                                 [f"FB iter:{iteration}  STATUS: DEPTH_INCONSISTENT",
                                  f"a:{a_d:.0f}mm b:{b_d:.0f}mm diff:{depth_diff:.0f}mm (false detect?)"],
                                 status="depth_inconsistent")
            return {"success": False, "step": "fb",
                    "reason": f"a/b深度差过大({depth_diff:.0f}mm),疑似误检", "color_img": color_img}

        # ── 安全检查 1: 最小深度 (防止撞架) ──
        if avg_d < min_depth:
            print(f"[FB-{iteration}] ⚠⚠ 安全终止: 深度过近 avg={avg_d:.0f}mm < {min_depth}mm, 立即停止!")
            draw.save_step_final(output_dir, "01_fb", iteration, color_img, annotated,
                                 [f"FB iter:{iteration}  STATUS: TOO_CLOSE_ABORT",
                                  f"a:{a_d:.0f}mm b:{b_d:.0f}mm avg:{avg_d:.0f}mm < {min_depth}mm"],
                                 status="too_close_abort")
            return {"success": False, "step": "fb",
                    "reason": f"深度过近({avg_d:.0f}mm),防止撞架", "color_img": color_img}

        # ── 安全检查 2 & 3: 偏差异常增大 / 旧图缓冲 ──
        curr_dir = 1 if delta > 0 else -1
        if last_move_dir != 0 and last_delta is not None:
            # 偏差异常增大 → 终止
            if abs(delta) - abs(last_delta) > 30:
                print(f"[FB-{iteration}] ⚠⚠ 安全终止: 偏差异常增大")
                draw.save_step_final(output_dir, "01_fb", iteration, color_img, annotated,
                                     [f"FB iter:{iteration}  STATUS: SAFETY_ABORT",
                                      f"a:{a_d:.0f}mm b:{b_d:.0f}mm avg:{avg_d:.0f}mm delta:{delta:+.0f}mm"],
                                     status="safety_abort")
                return {"success": False, "step": "fb", "reason": "安全终止", "color_img": color_img}

            # 移动后深度几乎不变 → 疑似旧图缓冲 → 终止
            if abs(avg_d - last_avg_d) < 10 and last_move_m_abs > 0.05:
                print(f"[FB-{iteration}] ⚠⚠ 安全终止: 移动{last_move_m_abs * 1000:.0f}mm后深度几乎不变 "
                      f"(avg {last_avg_d:.0f}→{avg_d:.0f}mm), 疑似取到旧图")
                draw.save_step_final(output_dir, "01_fb", iteration, color_img, annotated,
                                     [f"FB iter:{iteration}  STATUS: STALE_BUFFER_ABORT",
                                      f"moved {last_move_m_abs * 1000:.0f}mm but depth "
                                      f"{last_avg_d:.0f}→{avg_d:.0f}mm (stale buffer?)"],
                                     status="stale_buffer_abort")
                return {"success": False, "step": "fb",
                        "reason": "疑似旧图缓冲,安全终止", "color_img": color_img}

        # ── 收敛判断 ──
        if abs(delta) < threshold:
            print(f"[FB-{iteration}] ✓ 收敛 (|{delta:.0f}| < {threshold})")
            ts = time.strftime("%H%M%S")
            final_img = annotated.copy()
            cv2.circle(final_img, (int(best["a"]["cx"]), int(best["a"]["cy"])), 4, (0, 0, 255), -1)
            cv2.circle(final_img, (int(best["b"]["cx"]), int(best["b"]["cy"])), 4, (0, 0, 255), -1)
            cv2.putText(final_img, f"{a_d:.0f}mm", (int(best["a"]["cx"]) + 8, int(best["a"]["cy"])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(final_img, f"{b_d:.0f}mm", (int(best["b"]["cx"]) + 8, int(best["b"]["cy"])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            final_img = draw.draw_top_bar(final_img, [
                f"FB iter:{iteration}  a:{a_d:.0f}mm  b:{b_d:.0f}mm",
                f"avg:{avg_d:.0f}mm  target:{target_depth}mm  delta:{delta:+.0f}mm"])
            cv2.imwrite(os.path.join(output_dir, f"01_fb_final_iter{iteration}_{ts}.jpg"), final_img)
            return {"success": True, "step": "fb", "converged": True,
                    "a_depth": a_d, "b_depth": b_d, "avg_depth": avg_d, "color_img": color_img}

        # ── 自适应增益 ──
        if prev_delta is not None and prev_move_m is not None:
            actual_change = prev_delta - delta
            expected_change = prev_move_m * 1000.0  # mm
            if abs(expected_change) > 1.0:
                ratio = actual_change / expected_change
                if abs(ratio) > 0.1:
                    ideal_gain = 1.0 / ratio
                    new_gain = 0.5 * curr_gain + 0.5 * ideal_gain
                    new_gain = max(gain_min, min(gain_max, new_gain))
                    print(f"[FB-{iteration}] 自适应: ratio={ratio:.2f} gain {curr_gain:.2f}→{new_gain:.2f}")
                    curr_gain = new_gain

        # ── 计算移动量 (带增益, 截断) ──
        move_m = delta / 1000.0 * curr_gain
        if abs(move_m) > max_single_move:
            move_m = max_single_move if move_m > 0 else -max_single_move

        print(f"[FB-{iteration}] 移动: {move_m * 1000:+.0f}mm "
              f"({'前进' if move_m > 0 else '后退'}) [gain={curr_gain:.2f}]")

        if dry_run:
            print(f"[FB-{iteration}] (DRY RUN)")
            prev_delta = delta
            prev_move_m = move_m
            continue

        # ── 预移动: 克服静摩擦 (先反向再正向, 净位移不变) ──
        if pre_move_enabled and abs(move_m) < pre_move_threshold:
            pre_m = pre_move_m * (-1 if move_m > 0 else 1)
            print(f"[FB-{iteration}] 小距离预热: 先{'后退' if pre_m < 0 else '前进'}"
                  f"{abs(pre_m) * 1000:.0f}mm")
            import chassis
            chassis.move_chassis(g2, dx_m=pre_m)
            time.sleep(settle_time)
            ok = chassis.move_chassis(g2, dx_m=move_m - pre_m)
        else:
            import chassis
            ok = chassis.move_chassis(g2, dx_m=move_m)

        if not ok:
            draw.save_step_final(output_dir, "01_fb", iteration, color_img, annotated,
                                 [f"FB iter:{iteration}  STATUS: MOVE_FAILED",
                                  f"a:{a_d:.0f}mm b:{b_d:.0f}mm avg:{avg_d:.0f}mm delta:{delta:+.0f}mm"],
                                 status="move_failed")
            return {"success": False, "step": "fb", "reason": "移动失败", "color_img": color_img}

        last_move_dir = curr_dir
        last_delta = delta
        last_avg_d = avg_d
        last_move_m_abs = abs(move_m)
        prev_delta = delta
        prev_move_m = move_m
        time.sleep(settle_time)

    # 达到最大迭代仍未收敛
    draw.save_step_final(output_dir, "01_fb", max_iter, color_img, annotated,
                         [f"FB iter:{max_iter}  STATUS: MAX_ITER_NOT_CONVERGED",
                          f"a:{a_d:.0f}mm b:{b_d:.0f}mm avg:{avg_d:.0f}mm delta:{delta:+.0f}mm"],
                         status="max_iter")
    return {"success": True, "step": "fb", "converged": False, "color_img": color_img}


# ═══════════════════════════════════════════════════════════
#  纠偏步骤 2: 角度纠偏 (YAW)
# ═══════════════════════════════════════════════════════════

def step_yaw_correct(model, g2, output_dir, dry_run=False, reuse_img=None):
    """角度纠偏: 通过 a/b 连线斜率对齐底盘 Yaw (自适应增益)

    目标: a/b 连线斜率 → 0 (连线变水平, 等价中垂线变垂直)

    Returns:
        dict: {success, step, converged, reason, color_img, final_angle}
    """
    threshold_deg = _yaw("threshold_deg", 0.5)
    max_iter = _yaw("max_iter", 5)
    settle_time = _yaw("settle_time", 1.5)
    gain_init = _yaw("gain", 2.0)
    gain_min = _yaw("gain_min", 0.5)
    gain_max = _yaw("gain_max", 3.0)
    max_single_rotation = _yaw("max_single_rotation", 15.0)
    pre_rotate_enabled = _yaw("pre_rotate", True)
    pre_rotate_deg = _yaw("pre_rotate_deg", 15.0)
    pre_rotate_threshold = _yaw("pre_rotate_threshold", 3.0)

    print(f"\n{'=' * 60}")
    print(f"[步骤 2/3] 角度纠偏 (目标: angle → 0°)")
    print(f"{'=' * 60}")

    curr_gain = gain_init
    prev_angle = None
    prev_yaw_deg = None
    img = None

    for iteration in range(1, max_iter + 1):
        print(f"\n--- 角度纠偏 迭代 {iteration}/{max_iter} ---")

        # 拍照: 第1次迭代可复用前一步的彩色图, 失败后重拍
        best, annotated = None, None
        new_photo_count = 0
        max_new_photos = 3
        for attempt in range(max_new_photos + 1):
            if attempt == 0 and iteration == 1 and reuse_img is not None:
                print(f"[YAW-{iteration}] 复用上一步彩色图")
                img = reuse_img
            else:
                new_photo_count += 1
                print(f"[YAW-{iteration}] 拍照中... (第{new_photo_count}次)")
                img = camera.capture_color()

            # YOLO 检测 (严格 a+b)
            best, annotated = detector.detect_ab(model, img, strict_ab=True)
            if best is not None:
                break
            print(f"[YAW-{iteration}] ✗ 未同时检测到 a/b, 重试...")
            time.sleep(0.3)

        if best is None:
            print(f"[YAW-{iteration}] ✗ 多次拍照仍未检测到 a/b")
            draw.save_step_final(output_dir, "02_yaw", iteration, img, annotated,
                                 [f"YAW iter:{iteration}  STATUS: NO_AB_DETECTED",
                                  f"reason: a and b not detected simultaneously"], status="no_ab")
            return {"success": False, "step": "yaw", "reason": "未检测到 a/b", "color_img": img}

        # 计算角度
        a_cx, a_cy = best["a"]["cx"], best["a"]["cy"]
        b_cx, b_cy = best["b"]["cx"], best["b"]["cy"]
        dx = b_cx - a_cx
        slope = (b_cy - a_cy) / dx if abs(dx) > 1e-6 else float("inf")
        angle = np.degrees(np.arctan(slope)) if abs(slope) < 1e6 else 90.0
        mid_x = (a_cx + b_cx) / 2
        mid_y = (a_cy + b_cy) / 2

        print(f"[YAW-{iteration}] slope={slope:+.4f} angle={angle:+.2f}° gain={curr_gain:.2f}")

        # 保存标注图
        ts = time.strftime("%H%M%S")
        iter_img = draw.draw_geometry(annotated.copy(), best, mid_x, mid_y,
                                      [f"YAW iter:{iteration} slope:{slope:+.4f} "
                                       f"angle:{angle:+.2f}deg gain:{curr_gain:.2f}"])
        cv2.imwrite(os.path.join(output_dir, f"02_yaw_iter{iteration}_{ts}.jpg"), iter_img)

        # 收敛判断
        if abs(angle) < threshold_deg:
            print(f"[YAW-{iteration}] ✓ 收敛 (|{angle:.2f}| < {threshold_deg})")
            ts = time.strftime("%H%M%S")
            final_img = draw.draw_geometry(annotated.copy(), best, mid_x, mid_y, [
                f"YAW iter:{iteration}  slope:{slope:+.4f}  angle:{angle:+.2f}deg",
                f"gain:{curr_gain:.2f}  target:0deg"])
            cv2.imwrite(os.path.join(output_dir, f"02_yaw_final_iter{iteration}_{ts}.jpg"), final_img)
            return {"success": True, "step": "yaw", "converged": True,
                    "final_angle": angle, "color_img": img}

        # 自适应增益
        if prev_angle is not None and prev_yaw_deg is not None:
            actual_change = prev_angle - angle
            expected_change = -prev_yaw_deg
            if abs(expected_change) > 0.1:
                ratio = actual_change / expected_change
                if abs(ratio) > 0.01:
                    ideal_gain = 1.0 / ratio
                    new_gain = 0.5 * curr_gain + 0.5 * ideal_gain
                    new_gain = max(gain_min, min(gain_max, new_gain))
                    print(f"[YAW-{iteration}] 自适应: ratio={ratio:.2f} gain {curr_gain:.2f}→{new_gain:.2f}")
                    curr_gain = new_gain

        # 计算旋转量
        delta_yaw_rad = -np.arctan(slope) * curr_gain
        delta_yaw_deg = np.degrees(delta_yaw_rad)
        if abs(delta_yaw_deg) > max_single_rotation:
            delta_yaw_deg = np.sign(delta_yaw_deg) * max_single_rotation
            delta_yaw_rad = np.radians(delta_yaw_deg)

        print(f"[YAW-{iteration}] 旋转: {delta_yaw_deg:+.2f}°")

        if dry_run:
            print(f"[YAW-{iteration}] (DRY RUN)")
            prev_angle = angle
            prev_yaw_deg = delta_yaw_deg
            continue

        # 预旋转: 小角度时先反向大角度旋转, 克服静摩擦
        if pre_rotate_enabled and abs(delta_yaw_deg) < pre_rotate_threshold:
            pre_deg = pre_rotate_deg * (-1 if delta_yaw_deg > 0 else 1)
            print(f"[YAW-{iteration}] [预旋转] 先转 {pre_deg:+.1f}° 再补偿 (克服静摩擦)")
            import chassis
            chassis.move_chassis(g2, yaw_rad=np.radians(pre_deg))
            time.sleep(settle_time)
            delta_yaw_rad = delta_yaw_rad - np.radians(pre_deg)

        import chassis
        ok = chassis.move_chassis(g2, yaw_rad=delta_yaw_rad)
        if not ok:
            draw.save_step_final(output_dir, "02_yaw", iteration, img, annotated,
                                 [f"YAW iter:{iteration}  STATUS: ROTATE_FAILED",
                                  f"slope:{slope:+.4f} angle:{angle:+.2f}deg delta_yaw:{delta_yaw_deg:+.2f}deg"],
                                 status="rotate_failed")
            return {"success": False, "step": "yaw", "reason": "旋转失败", "color_img": img}

        prev_angle = angle
        prev_yaw_deg = delta_yaw_deg
        time.sleep(settle_time)

    draw.save_step_final(output_dir, "02_yaw", max_iter, img, annotated,
                         [f"YAW iter:{max_iter}  STATUS: MAX_ITER_NOT_CONVERGED",
                          f"slope:{slope:+.4f} angle:{angle:+.2f}deg gain:{curr_gain:.2f}"],
                         status="max_iter")
    return {"success": True, "step": "yaw", "converged": False, "color_img": img}


# ═══════════════════════════════════════════════════════════
#  纠偏步骤 3: 左右纠偏 (LR)
# ═══════════════════════════════════════════════════════════

def step_lr_correct(model, g2, output_dir, dry_run=False, reuse_img=None,
                    target_x_override=None, px_to_meter_override=None):
    """左右纠偏: 通过 a/b 中点 x 对齐图像中心

    目标: a/b 中点 x → 目标 x 坐标 (默认 320px), 等价中垂线穿过图像中心

    Args:
        target_x_override: 覆盖配置中的 lr.target_x (场景级别补偿), None=使用全局配置
        px_to_meter_override: 覆盖配置中的 lr.px_to_meter (场景级标定值), None=使用全局配置

    Returns:
        dict: {success, step, converged, reason, color_img, final_mid_x, final_delta_px}
    """
    target_x = target_x_override if target_x_override is not None else _lr("target_x", 320)
    threshold = _lr("threshold", 5)
    max_iter = _lr("max_iter", 5)
    settle_time = _lr("settle_time", 1.5)
    px_to_meter = px_to_meter_override if px_to_meter_override is not None else _lr("px_to_meter", 0.002584)
    gain_init = _lr("gain", 1.0)
    gain_min = _lr("gain_min", 0.5)
    gain_max = _lr("gain_max", 10.0)
    max_single_move = _lr("max_single_move", 0.20)
    pre_move_enabled = _lr("pre_move", True)
    pre_move_m = _lr("pre_move_m", 0.05)
    pre_move_threshold = _lr("pre_move_threshold", 0.01)

    print(f"\n{'=' * 60}")
    print(f"[步骤 3/3] 左右纠偏 (目标: mid_x → {target_x}px)")
    print(f"{'=' * 60}")

    curr_gain = gain_init
    prev_delta_px = None
    prev_move_m = None
    img = None

    for iteration in range(1, max_iter + 1):
        print(f"\n--- 左右纠偏 迭代 {iteration}/{max_iter} ---")

        # 拍照: 第1次迭代可复用前一步的彩色图
        if iteration == 1 and reuse_img is not None:
            print(f"[LR-{iteration}] 复用上一步彩色图")
            img = reuse_img
        else:
            img = camera.capture_color()

        # YOLO 检测 (允许 top2 回退, 至少2个框)
        best, annotated = detector.detect_ab(model, img, strict_ab=False, min_boxes=2)
        if best is None:
            print(f"[LR-{iteration}] ✗ 未检测到 a/b")
            draw.save_step_final(output_dir, "03_lr", iteration, img, annotated,
                                 [f"LR iter:{iteration}  STATUS: NO_AB_DETECTED",
                                  f"reason: a/b not found in image"], status="no_ab")
            return {"success": False, "step": "lr", "reason": "未检测到 a/b", "color_img": img}

        mid_x = (best["a"]["cx"] + best["b"]["cx"]) / 2
        mid_y = (best["a"]["cy"] + best["b"]["cy"]) / 2
        delta_px = mid_x - target_x

        print(f"[LR-{iteration}] mid_x={mid_x:.1f} delta={delta_px:+.1f}px")

        # 保存标注图
        ts = time.strftime("%H%M%S")
        iter_img = draw.draw_geometry(annotated.copy(), best, mid_x, mid_y,
                                      [f"LR iter:{iteration} mid_x:{mid_x:.1f} "
                                       f"delta:{delta_px:+.1f}px gain:{curr_gain:.2f}"])
        cv2.imwrite(os.path.join(output_dir, f"03_lr_iter{iteration}_{ts}.jpg"), iter_img)

        # 收敛判断
        if abs(delta_px) < threshold:
            print(f"[LR-{iteration}] ✓ 收敛 (|{delta_px:.1f}| < {threshold})")
            ts = time.strftime("%H%M%S")
            final_img = draw.draw_geometry(annotated.copy(), best, mid_x, mid_y, [
                f"LR iter:{iteration}  mid_x:{mid_x:.1f}  delta:{delta_px:+.1f}px",
                f"move:{(-delta_px * px_to_meter) * 1000:+.1f}mm  target:{target_x}px"])
            cv2.imwrite(os.path.join(output_dir, f"03_lr_final_iter{iteration}_{ts}.jpg"), final_img)
            return {"success": True, "step": "lr", "converged": True,
                    "final_mid_x": mid_x, "final_delta_px": delta_px, "color_img": img}

        # 自适应增益
        if prev_delta_px is not None and prev_move_m is not None:
            actual_change = prev_delta_px - delta_px
            expected_change = -prev_move_m / px_to_meter
            if abs(expected_change) > 0.5:
                ratio = actual_change / expected_change
                if abs(ratio) > 0.01:
                    ideal_gain = 1.0 / ratio
                    new_gain = 0.5 * curr_gain + 0.5 * ideal_gain
                    new_gain = max(gain_min, min(gain_max, new_gain))
                    print(f"[LR-{iteration}] 自适应: ratio={ratio:.2f} gain {curr_gain:.2f}→{new_gain:.2f}")
                    curr_gain = new_gain

        # 计算移动量 (带增益, 截断到 max_single_move)
        move_m = -delta_px * px_to_meter * curr_gain
        if abs(move_m) > max_single_move:
            print(f"[LR-{iteration}] ⚠ 单步移动 {abs(move_m)*1000:.0f}mm "
                  f"超过 max_single_move={max_single_move*1000:.0f}mm, 已截断")
            move_m = np.sign(move_m) * max_single_move
        print(f"[LR-{iteration}] 移动: {move_m * 1000:+.1f}mm "
              f"({'左' if move_m > 0 else '右'}) [gain={curr_gain:.2f}]")

        if dry_run:
            print(f"[LR-{iteration}] (DRY RUN)")
            prev_delta_px = delta_px
            prev_move_m = move_m
            continue

        # 预横移: 克服静摩擦
        if pre_move_enabled and abs(move_m) < pre_move_threshold:
            pre_m = pre_move_m * (-1 if move_m > 0 else 1)
            print(f"[LR-{iteration}] 小距离预热: 先{'右移' if pre_m < 0 else '左移'}"
                  f"{abs(pre_m) * 1000:.0f}mm")
            import chassis
            chassis.move_chassis(g2, dy_m=pre_m)
            time.sleep(settle_time)
            ok = chassis.move_chassis(g2, dy_m=move_m - pre_m)
        else:
            import chassis
            ok = chassis.move_chassis(g2, dy_m=move_m)

        if not ok:
            draw.save_step_final(output_dir, "03_lr", iteration, img, annotated,
                                 [f"LR iter:{iteration}  STATUS: MOVE_FAILED",
                                  f"mid_x:{mid_x:.1f} delta:{delta_px:+.1f}px move:{move_m * 1000:+.1f}mm"],
                                 status="move_failed")
            return {"success": False, "step": "lr", "reason": "移动失败", "color_img": img}

        prev_delta_px = delta_px
        prev_move_m = move_m
        time.sleep(settle_time)

    draw.save_step_final(output_dir, "03_lr", max_iter, img, annotated,
                         [f"LR iter:{max_iter}  STATUS: MAX_ITER_NOT_CONVERGED",
                          f"mid_x:{mid_x:.1f} delta:{delta_px:+.1f}px target:{target_x}px"],
                         status="max_iter")
    return {"success": True, "step": "lr", "converged": False, "color_img": img}


# ═══════════════════════════════════════════════════════════
#  三步纠偏总控
# ═══════════════════════════════════════════════════════════

def run_correct(model, g2, target_depth, output_dir, dry_run=False,
                skip_fb=False, skip_yaw=False, skip_lr=False,
                lr_target_x_override=None,
                calibrate_before_lr=True, scene_name=None, yaml_path=None,
                lr_px_to_meter_cache=None, order=None):
    """执行三步纠偏总控

    顺序: order 指定, 默认前后 → 角度 → [标定] → 左右 (放货可传 ["lr","yaw","fb"])
    任一步骤异常 (success=False) 则终止后续步骤, 停止机器人运动。

    Args:
        model: RhinoInfer 实例
        g2: minth.G2 实例 (dry_run 时可为 None)
        target_depth: 前后纠偏目标深度 mm
        output_dir: 结果图保存目录
        dry_run: 只检测不移动
        skip_fb/skip_yaw/skip_lr: 跳过对应步骤
        lr_target_x_override: 场景级 LR 目标 x 覆盖 (None=用全局 lr.target_x)
        calibrate_before_lr: LR 前是否在当前深度标定 px_to_meter (dry_run 自动跳过)
        scene_name: 场景名 ("pick"/"place"), 用于标定结果写回 yaml
        yaml_path: correct.yaml 路径, 提供则标定后写回场景级 px_to_meter_override
        lr_px_to_meter_cache: yaml 中缓存的场景级 px_to_meter (dry_run 时使用)
        order: 步骤顺序列表, 如 ["fb","yaw","lr"] (默认) 或 ["lr","yaw","fb"] (放货)

    Returns:
        dict: {"fb": ..., "yaw": ..., "lr": ..., "abort": bool}
    """
    os.makedirs(output_dir, exist_ok=True)
    t_start = time.time()

    if order is None:
        order = ["fb", "yaw", "lr"]
    _step_name = {"fb": "前后", "yaw": "角度", "lr": "左右"}
    steps = [s for s in order
             if not (s == "fb" and skip_fb)
             and not (s == "yaw" and skip_yaw)
             and not (s == "lr" and skip_lr)]

    print(f"\n{'=' * 60}")
    print(f"底盘综合纠偏总控")
    print(f"  顺序: {' → '.join(_step_name[s] for s in steps) or '(空)'}")
    print(f"  目标深度: {target_depth}mm")
    if lr_target_x_override is not None:
        default_tx = _lr("target_x", 320)
        print(f"  LR目标x: {lr_target_x_override}px (场景覆盖, 全局默认{default_tx}px)")
    print(f"  Dry Run: {dry_run}")
    print(f"  执行步骤: {' → '.join(_step_name[s] for s in steps) or '(空)'}")
    print(f"{'=' * 60}")

    # 清理旧图
    camera.cleanup_old_images()

    results = {}
    abort = False
    reuse_img = None

    lr_px_to_meter = None

    # 按 order 顺序执行各步
    for step in order:
        if abort:
            print(f"\n[跳过] {_step_name[step]} (前序纠偏已异常终止)")
            continue

        if step == "fb":
            if skip_fb:
                print(f"\n[跳过] 前后纠偏")
                continue
            r = step_fb_correct(model, g2, target_depth, output_dir, dry_run)
            results["fb"] = r
        elif step == "yaw":
            if skip_yaw:
                print(f"\n[跳过] 角度纠偏")
                continue
            r = step_yaw_correct(model, g2, output_dir, dry_run, reuse_img=reuse_img)
            results["yaw"] = r
        elif step == "lr":
            if skip_lr:
                print(f"\n[跳过] 左右纠偏")
                continue
            # ── LR 前标定: 在当前深度标定 px_to_meter ──
            if calibrate_before_lr and not dry_run:
                print(f"\n{'─' * 60}")
                print(f"[LR前] 在当前深度标定 px_to_meter (场景: {scene_name or '?'})")
                print(f"{'─' * 60}")
                import calibrate as _calib
                calib_dir = os.path.join(output_dir, "calibration")
                lr_px_to_meter = _calib.run_inline_calibration(
                    model, g2, output_dir=calib_dir)
                if lr_px_to_meter is None:
                    print(f"[LR前] ⚠ 标定失败, 回退到配置缓存值")
                    lr_px_to_meter = lr_px_to_meter_cache
                else:
                    # 写回 yaml 场景级
                    if yaml_path and scene_name:
                        ts = time.strftime("%Y%m%d_%H%M%S")
                        try:
                            _calib.update_scene_px_to_meter(
                                yaml_path, scene_name, lr_px_to_meter, ts,
                                target_depth=target_depth)
                        except Exception as e:
                            print(f"[LR前] ⚠ 写回yaml失败: {e}")
            else:
                # dry_run 或不需要标定: 用缓存值
                if lr_px_to_meter_cache is not None:
                    print(f"\n[LR前] 使用yaml缓存 px_to_meter={lr_px_to_meter_cache:.6f} (dry_run跳过标定)")
                    lr_px_to_meter = lr_px_to_meter_cache

            # ── 执行 LR 纠偏 ──
            r = step_lr_correct(model, g2, output_dir, dry_run, reuse_img=reuse_img,
                                target_x_override=lr_target_x_override,
                                px_to_meter_override=lr_px_to_meter)
            results["lr"] = r
        else:
            continue

        reuse_img = r.get("color_img")
        if not r["success"]:
            print(f"\n⚠⚠ {_step_name[step]}纠偏失败: {r.get('reason')}")
            print(f"⚠⚠ 终止后续步骤, 停止机器人运动!")
            abort = True

    if abort:
        print(f"\n{'=' * 60}")
        print(f"⚠⚠ 检测到异常, 已停止所有机器人运动")
        print(f"{'=' * 60}")

    # 总结
    elapsed = time.time() - t_start
    print(f"\n{'=' * 60}")
    print(f"综合纠偏总结 (耗时 {elapsed:.1f}s)")
    print(f"{'=' * 60}")

    if "fb" in results:
        r = results["fb"]
        if r.get("converged"):
            print(f"  前后: ✓ 收敛  a={r.get('a_depth', 0):.0f}mm b={r.get('b_depth', 0):.0f}mm "
                  f"avg={r.get('avg_depth', 0):.0f}mm")
        elif r.get("success"):
            print(f"  前后: ⚠ 未完全收敛")
        else:
            print(f"  前后: ✗ 失败 ({r.get('reason')})")

    if "yaw" in results:
        r = results["yaw"]
        if r.get("converged"):
            print(f"  角度: ✓ 收敛  angle={r.get('final_angle', 0):+.2f}°")
        elif r.get("success"):
            print(f"  角度: ⚠ 未完全收敛")
        else:
            print(f"  角度: ✗ 失败 ({r.get('reason')})")

    if "lr" in results:
        r = results["lr"]
        if r.get("converged"):
            print(f"  左右: ✓ 收敛  mid_x={r.get('final_mid_x', 0):.1f} "
                  f"delta={r.get('final_delta_px', 0):+.1f}px")
        elif r.get("success"):
            print(f"  左右: ⚠ 未完全收敛")
        else:
            print(f"  左右: ✗ 失败 ({r.get('reason')})")

    print(f"{'=' * 60}")

    results["abort"] = abort
    return results
