#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""chassis.py — 底盘控制模块

功能:
  - setup_minth(): 加载 minth 库并初始化 G2 控制
  - move_chassis(): 底盘相对运动 (dx 前, dy 左, yaw 逆时针)
  - go_to_point(): 导航到 SLAM 地图中的预录制点位
  - run_points(): 按顺序导航到多个点位

参考:
  - /home/agi/wzd/chassis_correct_all.py:561-570 (move_chassis)
  - /home/agi/wzd/chassis_run_012340.py (go_to_point / run_points)
"""
import sys
import time

# ── 全局状态 ──
_config = None
_minth_dir_added = False


def configure(cfg):
    """注入配置 (由 main.py 在启动时调用)

    cfg 应包含 common 段的:
      - mqtt_broker, mqtt_port
      - minth_dir
    """
    global _config
    _config = cfg


def _get(key, default=None):
    if _config is None:
        return default
    return _config.get(key, default)


# ═══════════════════════════════════════════════════════════
#  Minth 初始化
# ═══════════════════════════════════════════════════════════

def setup_minth(timeout=60):
    """加载 minth 模块并初始化 G2 控制

    Args:
        timeout: MQTT 命令超时秒数

    Returns:
        minth.G2 实例

    Raises:
        ImportError: minth 模块无法加载
        Exception: G2 初始化失败 (MQTT 连接失败等)
    """
    global _minth_dir_added
    minth_dir = _get("minth_dir", "/data/wxf/wxf0721/runtime")
    if minth_dir not in sys.path:
        sys.path.insert(0, minth_dir)
        _minth_dir_added = True

    broker = _get("mqtt_broker", "localhost")
    port = _get("mqtt_port", 1883)

    import minth
    g2 = minth.G2(broker=broker, port=port, timeout=timeout)
    return g2


# ═══════════════════════════════════════════════════════════
#  底盘相对运动 (纠偏用)
# ═══════════════════════════════════════════════════════════

def move_chassis(g2, dx_m=0.0, dy_m=0.0, yaw_rad=0.0):
    """底盘相对运动

    Args:
        g2: minth.G2 实例
        dx_m: 前后移动 (正=前进, 米)
        dy_m: 左右移动 (正=左, 米)
        yaw_rad: 旋转 (正=逆时针, 弧度)

    Returns:
        bool: 命令是否成功完成

    参考: chassis_correct_all.py:568-570
    """
    return g2._send_and_wait("go_rel", {"x": dx_m, "y": dy_m, "yaw_rad": yaw_rad})


# ═══════════════════════════════════════════════════════════
#  底盘导航 (SLAM 地图点位)
# ═══════════════════════════════════════════════════════════

def go_to_point(g2, point_num):
    """导航到 SLAM 地图中的预录制点位

    Args:
        g2: minth.G2 实例
        point_num: 地图点位编号 (0, 1, 2, ...)

    Returns:
        bool: 是否成功到达

    说明:
        点位坐标在 SLAM 地图中预录制 (通过 HMI), 本函数只发编号。
        超时由 G2 实例初始化时的 timeout 决定。
    """
    return g2.GO(point_num)


def run_points(g2, points, pause=2.0, skip_fail=False):
    """按顺序导航到多个点位

    Args:
        g2: minth.G2 实例
        points: 点位编号列表, 如 [0, 1, 2, 3, 4, 0]
        pause: 每个点到达后停留时间 (秒)
        skip_fail: True=某点失败也继续; False=遇错即停

    Returns:
        dict: {"total": N, "success": M, "failed_points": [...]}

    参考: chassis_run_012340.py:59-114
    """
    total = len(points)
    success = 0
    failed = []

    print(f"\n{'=' * 60}")
    print(f"开始顺序导航: {' → '.join(str(p) for p in points)}")
    print(f"共 {total} 个点位, 每点停留 {pause:.1f}s")
    print(f"{'=' * 60}")

    t_start = time.time()

    for i, pt in enumerate(points, 1):
        print(f"\n[{i}/{total}] 导航到 {pt} 号点 ...")
        t0 = time.time()
        ok = go_to_point(g2, pt)
        dt = time.time() - t0

        if ok:
            success += 1
            print(f"✓ 到达 {pt} 号点 (耗时 {dt:.1f}s)")
            if pause > 0:
                time.sleep(pause)
        else:
            failed.append(pt)
            print(f"✗ 导航到 {pt} 号点失败 (耗时 {dt:.1f}s)")
            if not skip_fail:
                print(f"! 终止后续导航 (使用 skip_fail=True 可继续)")
                break

    elapsed = time.time() - t_start
    print(f"\n{'=' * 60}")
    print(f"导航完成: 成功 {success}/{total}, 失败 {len(failed)} 个 {failed if failed else ''}")
    print(f"总耗时: {elapsed:.1f}s")
    print(f"{'=' * 60}")

    return {"total": total, "success": success, "failed_points": failed}
