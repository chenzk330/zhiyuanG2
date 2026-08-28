#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pipeline.py — 流水线总控 (取货→右转90°→放货, 动作人工衔接)

流程:
  1. 取货三步纠偏 (前后 → 角度 → 左右)
  2. ★ 停顿等待人工执行取货动作 (按回车继续)
  3. 原地顺时针旋转 90° (向右转身)
  4. 底盘稳定 3 秒
  5. 放货三步纠偏 (前后 → 角度 → 左右)
  6. ★ 停顿等待人工执行放货动作 (按回车继续)

设计:
  - czk 纯做纠偏 + 旋转, 双臂/夹爪动作由人工衔接
  - 取货点/放货点物理位置夹角 90°, 无需 SLAM 导航点
  - 纠偏完成后打印提示并等待回车, 不自动触发取货/放货动作
  - 任一纠偏环节异常则终止后续流程

用法:
  python pipeline.py                              # 完整流水线 (含LR前标定)
  python pipeline.py --skip-calibration           # 跳过标定, 用yaml缓存的px_to_meter
  python pipeline.py --skip-pick                  # 跳过取货纠偏 (直接旋转→放货)
  python pipeline.py --skip-place                 # 跳过放货纠偏 (取货→旋转后结束)
  python pipeline.py --skip-rotate                # 跳过90°旋转 (调试用)
  python pipeline.py --dry-run                    # 只打印流程, 不实际执行
  python pipeline.py --auto-continue               # 跳过人工停顿 (调试用)
  python pipeline.py --rotate-deg 90               # 自定义旋转角度 (默认90°)
"""
import argparse
import math
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import camera
import detector
import chassis
import draw
import correct
import check_env
from main import load_config, setup_modules


# ═══════════════════════════════════════════════════════════
#  人工衔接停顿
# ═══════════════════════════════════════════════════════════

def manual_hook(message, auto_continue=False):
    """人工衔接停顿: 打印提示并等待回车

    Args:
        message: 提示信息
        auto_continue: True=不等待回车 (调试用)
    """
    print(f"\n{'#' * 60}")
    print(f"# ★ 人工衔接点 ★")
    print(f"# {message}")
    print(f"{'#' * 60}")

    if auto_continue:
        print(f"# (auto-continue 模式, 自动继续)")
        return

    try:
        input(f"\n>>> 完成后按回车继续... ")
    except EOFError:
        print(f"(无交互输入, 自动继续)")
    print(f"[继续]")


# ═══════════════════════════════════════════════════════════
#  场景纠偏执行
# ═══════════════════════════════════════════════════════════

def run_scene_correct(scene_name, cfg, model_cache, g2, dry_run=False,
                      skip_fb=False, skip_yaw=False, skip_lr=False,
                      skip_calibration=False):
    """执行单个场景的三步纠偏 (含 LR 前自动标定)

    Args:
        scene_name: "pick" 或 "place"
        cfg: 完整配置 dict
        model_cache: 模型缓存 dict (避免重复加载, {scene_name: model})
        g2: minth.G2 实例
        dry_run, skip_fb, skip_yaw, skip_lr: 控制参数
        skip_calibration: True=跳过LR前标定, 用yaml缓存的px_to_meter_override

    Returns:
        dict: 纠偏结果 (含 abort 字段)
    """
    scenes = cfg.get("scenes", {})
    scene_cfg = scenes.get(scene_name, {})
    if not scene_cfg:
        print(f"✗ 场景配置不存在: {scene_name}")
        return {"abort": True}

    model_ref = scene_cfg.get("model_ref", f"models/{scene_name}.ref")
    if not os.path.isabs(model_ref):
        model_ref = os.path.join(_HERE, model_ref)
    target_depth = scene_cfg.get("target_depth", 750)
    output_dir = scene_cfg.get("output_dir", f"output/{scene_name}")
    if not os.path.isabs(output_dir):
        output_dir = os.path.join(_HERE, output_dir)
    lr_target_x_override = scene_cfg.get("lr_target_x_override", None)
    lr_px_to_meter_cache = scene_cfg.get("px_to_meter_override", None)

    # yaml 路径 (用于标定结果写回)
    correct_yaml = os.path.join(_HERE, "config", "correct.yaml")

    # 加载模型 (缓存)
    if scene_name not in model_cache:
        print(f"\n[加载模型] {scene_name} 场景: {model_ref}")
        model_cache[scene_name] = detector.load_model(model_ref)
    model = model_cache[scene_name]

    # 执行纠偏 (场景级 LR 目标覆盖 + LR前标定)
    results = correct.run_correct(
        model, g2, target_depth, output_dir, dry_run=dry_run,
        skip_fb=skip_fb, skip_yaw=skip_yaw, skip_lr=skip_lr,
        lr_target_x_override=lr_target_x_override,
        calibrate_before_lr=not skip_calibration,
        scene_name=scene_name,
        yaml_path=correct_yaml,
        lr_px_to_meter_cache=lr_px_to_meter_cache)

    return results


# ═══════════════════════════════════════════════════════════
#  原地旋转 (顺时针 = 负角度)
# ═══════════════════════════════════════════════════════════

def rotate_chassis(g2, deg_cw, settle_s=3.0, dry_run=False):
    """底盘原地旋转

    Args:
        g2: minth.G2 实例 (dry_run 时可为 None)
        deg_cw: 顺时针旋转角度 (度), 正值=顺时针, 负值=逆时针
        settle_s: 旋转后稳定时间 (秒)
        dry_run: 只打印不执行

    Returns:
        bool: 是否成功
    """
    # 顺时针 → yaw_rad 为负
    yaw_rad = -math.radians(deg_cw)
    direction = "顺时针" if deg_cw >= 0 else "逆时针"
    abs_deg = abs(deg_cw)

    print(f"\n{'=' * 60}")
    print(f"[旋转] 原地{direction} {abs_deg:.0f}° (yaw_rad={yaw_rad:+.4f})")
    print(f"{'=' * 60}")

    if dry_run:
        print(f"(DRY RUN: 跳过实际旋转)")
        print(f"旋转后稳定 {settle_s:.1f}s (DRY RUN: 跳过等待)")
        return True

    ok = chassis.move_chassis(g2, yaw_rad=yaw_rad)
    if not ok:
        print(f"✗ 旋转命令执行失败")
        return False

    print(f"✓ 旋转完成, 等待底盘稳定 {settle_s:.1f}s ...")
    time.sleep(settle_s)
    print(f"✓ 底盘稳定, 继续下一步")
    return True


# ═══════════════════════════════════════════════════════════
#  主流程
# ═══════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="流水线总控 (取货纠偏→右转90°→放货纠偏, 动作人工衔接)")
    parser.add_argument("--skip-pick", action="store_true",
                        help="跳过取货纠偏 (直接旋转→放货)")
    parser.add_argument("--skip-place", action="store_true",
                        help="跳过放货纠偏 (取货→旋转后结束)")
    parser.add_argument("--skip-rotate", action="store_true",
                        help="跳过中间 90° 旋转 (调试用)")
    parser.add_argument("--dry-run", action="store_true",
                        help="只打印流程, 不实际执行")
    parser.add_argument("--auto-continue", action="store_true",
                        help="跳过人工停顿 (调试用, 不等待回车)")
    parser.add_argument("--skip-env-check", action="store_true",
                        help="跳过环境检查")
    parser.add_argument("--skip-slam-check", action="store_true",
                        help="环境检查时跳过 SLAM")
    parser.add_argument("--skip-calibration", action="store_true",
                        help="跳过LR前标定, 用yaml缓存的px_to_meter_override")
    parser.add_argument("--rotate-deg", type=float, default=90.0,
                        help="中间旋转角度 (度, 顺时针, 默认90)")
    parser.add_argument("--rotate-settle", type=float, default=3.0,
                        help="旋转后稳定时间 (秒, 默认3)")
    args = parser.parse_args()

    t_total = time.time()

    # ── 加载配置 ──
    correct_yaml = os.path.join(_HERE, "config", "correct.yaml")
    if not os.path.exists(correct_yaml):
        print(f"✗ 配置文件不存在: {correct_yaml}")
        return 1
    cfg = load_config(correct_yaml)
    setup_modules(cfg)

    # ── 环境检查 ──
    if not args.skip_env_check:
        ok = check_env.run_all_checks(skip_slam=args.skip_slam_check)
        if not ok:
            print(f"\n环境检查未通过, 终止. (加 --skip-env-check 可跳过)")
            return 1

    # ── 流程概览 ──
    print(f"\n{'=' * 60}")
    print(f"流水线总控 (取货纠偏→右转{args.rotate_deg:.0f}°→放货纠偏, 动作人工衔接)")
    print(f"  Dry Run: {args.dry_run}")
    print(f"  Auto Continue: {args.auto_continue}")
    print(f"  旋转角度: {args.rotate_deg:.0f}° (顺时针)")
    print(f"  旋转稳定: {args.rotate_settle:.1f}s")
    print(f"  LR前标定: {'跳过(用缓存)' if args.skip_calibration else '每次标定'}")
    parts = []
    if not args.skip_pick:
        parts.append("取货纠偏 → [人工取货]")
    if not args.skip_rotate:
        parts.append(f"右转{args.rotate_deg:.0f}°")
    if not args.skip_place:
        parts.append("放货纠偏 → [人工放货]")
    print(f"  流程: {' → '.join(parts)}")
    print(f"{'=' * 60}")

    # ── 连接机器人 ──
    model_cache = {}
    if args.dry_run:
        print(f"\n[初始化] DRY RUN 模式, 跳过机器人连接")
        g2 = None
    else:
        print(f"\n[初始化] 连接机器人...")
        g2 = chassis.setup_minth()
        print(f"[初始化] ✓ Minth 已就绪")

    try:
        results_log = []

        # ════════════════════════════════════════
        # Phase 1: 取货纠偏
        # ════════════════════════════════════════
        if not args.skip_pick:
            print(f"\n{'#' * 60}")
            print(f"# Phase 1: 取货纠偏")
            print(f"{'#' * 60}")

            r_pick = run_scene_correct("pick", cfg, model_cache, g2, dry_run=args.dry_run,
                                       skip_calibration=args.skip_calibration)
            results_log.append(("取货纠偏", not r_pick.get("abort", True)))

            if r_pick.get("abort"):
                print(f"\n⚠⚠ 取货纠偏异常, 终止整个流水线!")
                _summary(results_log, t_total)
                return 1

            # 人工衔接: 取货动作
            manual_hook("取货纠偏完成, 请手动执行取货动作",
                        auto_continue=args.auto_continue or args.dry_run)
        else:
            print(f"\n[跳过] 取货纠偏")

        # ════════════════════════════════════════
        # Phase 2: 原地右转 90°
        # ════════════════════════════════════════
        if not args.skip_rotate:
            print(f"\n{'#' * 60}")
            print(f"# Phase 2: 原地右转 {args.rotate_deg:.0f}°")
            print(f"{'#' * 60}")

            rot_ok = rotate_chassis(g2, deg_cw=args.rotate_deg,
                                    settle_s=args.rotate_settle,
                                    dry_run=args.dry_run)
            results_log.append((f"右转{args.rotate_deg:.0f}°", rot_ok))

            if not rot_ok and not args.dry_run:
                print(f"\n⚠⚠ 旋转失败, 终止整个流水线!")
                _summary(results_log, t_total)
                return 1
        else:
            print(f"\n[跳过] 中间旋转")

        # ════════════════════════════════════════
        # Phase 3: 放货纠偏
        # ════════════════════════════════════════
        if not args.skip_place:
            print(f"\n{'#' * 60}")
            print(f"# Phase 3: 放货纠偏")
            print(f"{'#' * 60}")

            r_place = run_scene_correct("place", cfg, model_cache, g2, dry_run=args.dry_run,
                                         skip_calibration=args.skip_calibration)
            results_log.append(("放货纠偏", not r_place.get("abort", True)))

            if r_place.get("abort"):
                print(f"\n⚠⚠ 放货纠偏异常, 终止流水线!")
                _summary(results_log, t_total)
                return 1

            # 人工衔接: 放货动作
            manual_hook("放货纠偏完成, 请手动执行放货动作",
                        auto_continue=args.auto_continue or args.dry_run)
        else:
            print(f"\n[跳过] 放货纠偏")

        _summary(results_log, t_total)
        return 0

    except KeyboardInterrupt:
        print(f"\n[中断] 用户中止")
        return 130
    finally:
        if g2 is not None:
            try:
                g2.close()
            except Exception:
                pass


def _summary(results_log, t_start):
    """打印流水线总结"""
    elapsed = time.time() - t_start
    print(f"\n{'=' * 60}")
    print(f"流水线总结 (总耗时 {elapsed:.1f}s)")
    print(f"{'=' * 60}")
    for name, ok in results_log:
        status = "✓ 成功" if ok else "✗ 失败"
        print(f"  {name}: {status}")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        import traceback
        print(f"\n{'=' * 60}")
        print(f"✗ 程序异常崩溃: {type(e).__name__}: {e}")
        print(f"{'=' * 60}")
        traceback.print_exc()
        sys.exit(2)
