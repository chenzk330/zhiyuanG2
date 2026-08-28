#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
getjointstate.py — 获取机器人当前各轴位姿并保存为 JSON 文件

功能：
  1. 初始化 GDK 并连接机器人
  2. 读取所有关节状态（头部/腰部/双臂/夹爪）
  3. 读取末端执行器位姿（左右手 position + orientation）
  4. 读取末端夹爪开合状态
  5. 将所有数据保存为 JSON 文件，供后续执行程序调用

用法：
  python3 getjointstate.py                          # 保存为 joint_state_<时间戳>.json
  python3 getjointstate.py --name my_pose           # 保存为 my_pose.json
  python3 getjointstate.py --output /path/to/dir    # 指定输出目录
"""

import os
import sys
import json
import time
import argparse

import agibot_gdk

# ── 关节名分组 ─────────────────────────────────────────────
HEAD_JOINT_KEYS = [
    "idx11_head_joint1", "idx12_head_joint2", "idx13_head_joint3",
]
WAIST_JOINT_KEYS = [
    "idx01_body_joint1", "idx02_body_joint2", "idx03_body_joint3",
    "idx04_body_joint4", "idx05_body_joint5",
]
LEFT_ARM_JOINT_KEYS = [
    "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3",
    "idx24_arm_l_joint4", "idx25_arm_l_joint5", "idx26_arm_l_joint6",
    "idx27_arm_l_joint7",
]
RIGHT_ARM_JOINT_KEYS = [
    "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3",
    "idx64_arm_r_joint4", "idx65_arm_r_joint5", "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]
LEFT_GRIPPER_KEY = "idx31_gripper_l_inner_joint1"
RIGHT_GRIPPER_KEY = "idx71_gripper_r_inner_joint1"

# 末端执行器 frame 名称（与 status.py 一致）
LEFT_EE_NAME = "arm_l_end_link"
RIGHT_EE_NAME = "arm_r_end_link"

# GDK 初始化后等待 DDS 建链时间
GDK_INIT_WAIT_S = 2.0


def init_robot():
    """初始化 GDK 并创建 Robot 对象"""
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        raise RuntimeError("GDK 初始化失败")
    print("[GDK] 初始化成功")

    robot = agibot_gdk.Robot()
    time.sleep(GDK_INIT_WAIT_S)
    print("[GDK] Robot 对象就绪")
    return robot


def read_joint_positions(robot):
    """读取所有关节的 motor_position，返回 {关节名: 位置} 扁平字典"""
    joint_states = robot.get_joint_states()
    joints = {}
    for state in joint_states['states']:
        joints[state['name']] = round(state['motor_position'], 6)
    return joints


def read_gripper_states(robot):
    """读取左右夹爪状态"""
    try:
        end_state = robot.get_end_state()
        result = {}
        for side in ['left', 'right']:
            side_state = end_state.get(f'{side}_end_state', {})
            end_states = side_state.get('end_states') or []
            if end_states:
                result[side] = round(float(end_states[0].get('position', 0.0)), 6)
            else:
                result[side] = None
        return result
    except Exception as e:
        print(f"  [警告] 读取夹爪状态失败: {e}")
        return {'left': None, 'right': None}


def read_end_effector_poses(robot):
    """读取左右手末端位姿（position + orientation 四元数）"""
    try:
        status = robot.get_motion_control_status()
    except Exception as e:
        print(f"  [警告] 读取运动控制状态失败: {e}")
        return {'left': None, 'right': None}

    def find_pose(frame_name):
        for i, name in enumerate(status.frame_names):
            if name == frame_name:
                pose = status.frame_poses[i]
                return {
                    'position': [
                        round(pose.position.x, 6),
                        round(pose.position.y, 6),
                        round(pose.position.z, 6),
                    ],
                    'orientation': [
                        round(pose.orientation.x, 6),
                        round(pose.orientation.y, 6),
                        round(pose.orientation.z, 6),
                        round(pose.orientation.w, 6),
                    ],
                }
        return None

    return {
        'left': find_pose(LEFT_EE_NAME),
        'right': find_pose(RIGHT_EE_NAME),
    }


def categorize_joints(joint_dict):
    """将关节按身体部位分类"""
    categorized = {
        'head': {},
        'waist': {},
        'left_arm': {},
        'right_arm': {},
        'left_gripper': {},
        'right_gripper': {},
    }
    for name, pos in joint_dict.items():
        if name in HEAD_JOINT_KEYS:
            categorized['head'][name] = pos
        elif name in WAIST_JOINT_KEYS:
            categorized['waist'][name] = pos
        elif name in LEFT_ARM_JOINT_KEYS:
            categorized['left_arm'][name] = pos
        elif name in RIGHT_ARM_JOINT_KEYS:
            categorized['right_arm'][name] = pos
        elif name == LEFT_GRIPPER_KEY:
            categorized['left_gripper'][name] = pos
        elif name == RIGHT_GRIPPER_KEY:
            categorized['right_gripper'][name] = pos
        else:
            categorized.setdefault('other', {})[name] = pos
    return categorized


def collect_pose_data(robot):
    """收集机器人当前所有位姿数据"""
    print("\n[采集] 正在读取关节状态...")
    joint_positions = read_joint_positions(robot)
    print(f"  关节数量: {len(joint_positions)}")

    print("[采集] 正在读取夹爪状态...")
    grippers = read_gripper_states(robot)
    print(f"  左夹爪: {grippers['left']}, 右夹爪: {grippers['right']}")

    print("[采集] 正在读取末端位姿...")
    ee_poses = read_end_effector_poses(robot)
    if ee_poses['left']:
        print(f"  左臂末端: {ee_poses['left']['position']}")
    if ee_poses['right']:
        print(f"  右臂末端: {ee_poses['right']['position']}")

    # 分类关节数据
    categorized = categorize_joints(joint_positions)

    # 构建完整的 JSON 数据
    pose_data = {
        'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        'joints': joint_positions,
        'joints_by_category': categorized,
        'grippers': grippers,
        'end_effectors': ee_poses,
    }

    return pose_data


def save_pose_data(pose_data, output_dir, filename=None):
    """保存位姿数据到 JSON 文件"""
    os.makedirs(output_dir, exist_ok=True)

    if filename is None:
        filename = f"joint_state_{time.strftime('%Y%m%d_%H%M%S')}.json"

    filepath = os.path.join(output_dir, filename)

    with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(pose_data, f, indent=2, ensure_ascii=False)

    print(f"\n[保存] 位姿数据已保存到: {filepath}")
    return filepath


def main():
    parser = argparse.ArgumentParser(
        description='获取机器人当前各轴位姿并保存为 JSON 文件'
    )
    parser.add_argument(
        '--name', type=str, default=None,
        help='保存的 JSON 文件名（不含扩展名，默认自动生成时间戳）'
    )
    parser.add_argument(
        '--output', type=str, default=None,
        help='输出目录（默认: function/poses/）'
    )
    args = parser.parse_args()

    # 确定输出目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = args.output or os.path.join(script_dir, 'poses')

    robot = None
    try:
        # 1. 初始化机器人
        robot = init_robot()

        # 2. 采集位姿数据
        pose_data = collect_pose_data(robot)

        # 3. 保存 JSON 文件
        filename = f"{args.name}.json" if args.name else None
        filepath = save_pose_data(pose_data, output_dir, filename)

        # 4. 打印摘要
        print(f"\n{'='*60}")
        print("位姿采集完成摘要:")
        print(f"{'='*60}")
        print(f"  时间戳: {pose_data['timestamp']}")
        print(f"  关节总数: {len(pose_data['joints'])}")
        cat = pose_data['joints_by_category']
        for part, joints in cat.items():
            if joints:
                print(f"  {part}: {len(joints)} 个关节")
        print(f"  夹爪: 左={pose_data['grippers']['left']}, 右={pose_data['grippers']['right']}")
        print(f"  文件路径: {filepath}")

    except Exception as e:
        print(f"\n[错误] 程序异常: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        # 释放 GDK 资源
        if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
            print("[GDK] 释放失败")
        else:
            print("[GDK] 释放成功")


if __name__ == "__main__":
    main()
