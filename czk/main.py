#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""main.py — 单场景纠偏命令行入口

用法:
  # 取货纠偏
  python main.py --scene pick
  python main.py --scene pick --dry-run          # 只检测不移动
  python main.py --scene pick --target-depth 750

  # 放货纠偏
  python main.py --scene place
  python main.py --scene place --target-depth 800

  # 跳过某些步骤
  python main.py --scene pick --skip-fb
  python main.py --scene pick --skip-yaw --skip-lr

  # 先导航到纠偏点再纠偏
  python main.py --scene pick --nav-to 1
"""
import argparse
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


# ═══════════════════════════════════════════════════════════
#  YAML 配置加载
# ═══════════════════════════════════════════════════════════

def load_config(yaml_path):
    """加载 YAML 配置文件 (使用 PyYAML)

    支持: 嵌套 section, 列表, 字符串, 数字, 布尔, 行内注释
    """
    import yaml
    with open(yaml_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def setup_modules(cfg):
    """将配置注入各模块"""
    camera.configure(cfg.get("common", {}))
    detector.configure(cfg.get("yolo", {}))
    chassis.configure(cfg.get("common", {}))
    draw.configure(cfg)
    correct.configure(cfg)


# ═══════════════════════════════════════════════════════════
#  主函数
# ═══════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="单场景三步纠偏 (取货/放货)")
    parser.add_argument("--scene", choices=["pick", "place"], default="pick",
                        help="纠偏场景: pick=取货, place=放货 (默认: pick)")
    parser.add_argument("--target-depth", type=int, default=None,
                        help="前后纠偏目标深度 mm (默认用 config 中场景值)")
    parser.add_argument("--dry-run", action="store_true", help="只检测不移动")
    parser.add_argument("--skip-fb", action="store_true", help="跳过前后纠偏")
    parser.add_argument("--skip-yaw", action="store_true", help="跳过角度纠偏")
    parser.add_argument("--skip-lr", action="store_true", help="跳过左右纠偏")
    parser.add_argument("--nav-to", type=int, default=None,
                        help="纠偏前先导航到指定 SLAM 点位")
    parser.add_argument("--skip-env-check", action="store_true",
                        help="跳过环境前置检查")
    parser.add_argument("--skip-slam-check", action="store_true",
                        help="环境检查时跳过 SLAM odom 检查")
    args = parser.parse_args()

    t_total = time.time()

    # ── 加载配置 ──
    yaml_path = os.path.join(_HERE, "config", "correct.yaml")
    if not os.path.exists(yaml_path):
        print(f"✗ 配置文件不存在: {yaml_path}")
        return 1
    cfg = load_config(yaml_path)
    setup_modules(cfg)

    # 场景配置
    scenes = cfg.get("scenes", {})
    scene_cfg = scenes.get(args.scene, {})
    if not scene_cfg:
        print(f"✗ 场景配置不存在: {args.scene}")
        return 1

    model_ref = scene_cfg.get("model_ref", f"models/{args.scene}.ref")
    if not os.path.isabs(model_ref):
        model_ref = os.path.join(_HERE, model_ref)
    target_depth = args.target_depth or scene_cfg.get("target_depth", 750)
    output_dir = scene_cfg.get("output_dir", f"output/{args.scene}")
    if not os.path.isabs(output_dir):
        output_dir = os.path.join(_HERE, output_dir)
    lr_target_x_override = scene_cfg.get("lr_target_x_override", None)
    lr_px_to_meter_cache = scene_cfg.get("px_to_meter_override", None)
    correct_yaml = os.path.join(_HERE, "config", "correct.yaml")

    # ── 环境检查 ──
    if not args.skip_env_check:
        ok = check_env.run_all_checks(skip_slam=args.skip_slam_check)
        if not ok:
            print(f"\n环境检查未通过, 终止. (加 --skip-env-check 可跳过)")
            return 1

    # ── 流程概览 ──
    print(f"\n{'=' * 60}")
    print(f"单场景三步纠偏")
    print(f"  场景: {args.scene}")
    print(f"  模型: {model_ref}")
    print(f"  目标深度: {target_depth}mm")
    if lr_target_x_override is not None:
        print(f"  LR目标x: {lr_target_x_override}px (场景覆盖)")
    if lr_px_to_meter_cache is not None:
        print(f"  LR缓存px_to_meter: {lr_px_to_meter_cache:.6f} m/px")
    else:
        print(f"  LR缓存px_to_meter: 无 (首次运行将自动标定)")
    print(f"  流程: 前后→角度→[LR前标定]→左右")
    print(f"  输出目录: {output_dir}")
    print(f"  Dry Run: {args.dry_run}")
    print(f"  导航点: {args.nav_to if args.nav_to is not None else '无'}")
    print(f"{'=' * 60}")

    # ── 加载模型 ──
    print(f"\n[初始化] 加载辉羲 RPU 模型...")
    model = detector.load_model(model_ref)

    # ── 连接机器人 ──
    if args.dry_run:
        print(f"[初始化] DRY RUN 模式, 跳过机器人连接")
        g2 = None
    else:
        print(f"[初始化] 连接机器人...")
        g2 = chassis.setup_minth()
        print(f"[初始化] ✓ Minth 已就绪")

    try:
        # ── 导航到纠偏点 ──
        if args.nav_to is not None and g2 is not None:
            print(f"\n[导航] 前往 {args.nav_to} 号点...")
            ok = chassis.go_to_point(g2, args.nav_to)
            if not ok:
                print(f"[导航] ✗ 导航失败, 终止纠偏")
                return 1
            print(f"[导航] ✓ 已到达")
            time.sleep(2.0)

        # ── 三步纠偏 (含LR前标定) ──
        results = correct.run_correct(
            model, g2, target_depth, output_dir, dry_run=args.dry_run,
            skip_fb=args.skip_fb, skip_yaw=args.skip_yaw, skip_lr=args.skip_lr,
            lr_target_x_override=lr_target_x_override,
            calibrate_before_lr=True,
            scene_name=args.scene,
            yaml_path=correct_yaml,
            lr_px_to_meter_cache=lr_px_to_meter_cache)

    except KeyboardInterrupt:
        print(f"\n[中断] 用户中止")
        return 130
    finally:
        if g2 is not None:
            try:
                g2.close()
            except Exception:
                pass

    elapsed = time.time() - t_total
    print(f"\n总耗时: {elapsed:.1f}s")

    # 返回码: 0=正常, 1=纠偏异常, 2=崩溃
    if results.get("abort"):
        return 1
    return 0


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
