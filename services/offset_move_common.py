import agibot_gdk
import time
import math

# ─────────────────────────────────────────────
# 坐标系说明（末端）：
#   X+  向前   Y+  向左   Z+  向上
# ─────────────────────────────────────────────

LEFT_NAME  = "arm_l_end_link"
RIGHT_NAME = "arm_r_end_link"

# ── 控制参数 ──────────────────────────────────
MAX_STEP_CM = 0.1    # 单步最大位移（厘米）
LIFETIME    = 0.02   # 指令生命周期（秒）
RATE_HZ     = 50.0   # 发送频率（Hz）

# ═══════════════════════════════════════════════════════════════
#  末端执行器控制器
# ═══════════════════════════════════════════════════════════════

class EndEffectorController:

    def __init__(self, robot):
        self.robot = robot

    # ── 数学与规划工具 ─────────────────────────────────────────────

    @staticmethod
    def slerp(q0, q1, t):
        """四元数球面线性插值 [x, y, z, w]"""
        dot = sum(q0[i] * q1[i] for i in range(4))
        if dot < 0.0:
            dot = -dot
            q1 = [-v for v in q1]
        dot = max(-1.0, min(1.0, dot))
        if dot > 0.9995:
            result = [q0[i] + t * (q1[i] - q0[i]) for i in range(4)]
            norm = math.sqrt(sum(v * v for v in result))
            return [v / norm for v in result] if norm > 0 else result
        theta_0 = math.acos(dot)
        sin_t0  = math.sin(theta_0)
        theta   = theta_0 * t
        s0 = math.cos(theta) - dot * math.sin(theta) / sin_t0
        s1 = math.sin(theta) / sin_t0
        return [s0 * q0[i] + s1 * q1[i] for i in range(4)]

    @staticmethod
    def distance(p1, p2):
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2)))

    def _n_steps(self, start_pos, goal_pos):
        dist_cm = self.distance(start_pos, goal_pos) * 100.0
        return max(int(math.ceil(dist_cm / MAX_STEP_CM)), 1)

    def _plan(self, start_pose, goal_pose, n_steps):
        """生成直线运动的轨迹点序列"""
        traj = []
        for i in range(n_steps):
            t = float(i) / (n_steps - 1) if n_steps > 1 else 0.0
            pos = [start_pose["position"][j] + t * (goal_pose["position"][j] - start_pose["position"][j])
                   for j in range(3)]
            quat = self.slerp(start_pose["orientation"], goal_pose["orientation"], t)
            traj.append({"position": pos, "orientation": quat})
        return traj

    def _find_pose(self, status, name):
        """从状态数据中寻找指定 link 的当前位姿"""
        for i, frame_name in enumerate(status.frame_names):
            if frame_name == name:
                p = status.frame_poses[i]
                return {
                    "position":    [p.position.x, p.position.y, p.position.z],
                    "orientation": [p.orientation.x, p.orientation.y,
                                    p.orientation.z, p.orientation.w],
                }
        raise RuntimeError(f"帧名 '{name}' 未找到")

    # ── 运动执行 ─────────────────────────────────────────────

    def _send_dual_trajectory(self, traj_left, traj_right):
        """同时发送双臂的轨迹指令序列"""
        dt = 1.0 / RATE_HZ  # 50Hz 时 dt = 0.02秒
        steps = len(traj_left) # 左右臂步数是对齐的
        
        for i in range(steps):
            wp_l = traj_left[i]
            wp_r = traj_right[i]
            
            # --- 构造左臂指令 ---
            end_pose_l = agibot_gdk.EndEffectorPose()
            end_pose_l.life_time = LIFETIME
            end_pose_l.group     = agibot_gdk.EndEffectorControlGroup.kLeftArm

            end_pose_l.left_end_effector_pose.position.x    = wp_l["position"][0]
            end_pose_l.left_end_effector_pose.position.y    = wp_l["position"][1]
            end_pose_l.left_end_effector_pose.position.z    = wp_l["position"][2]
            end_pose_l.left_end_effector_pose.orientation.x = wp_l["orientation"][0]
            end_pose_l.left_end_effector_pose.orientation.y = wp_l["orientation"][1]
            end_pose_l.left_end_effector_pose.orientation.z = wp_l["orientation"][2]
            end_pose_l.left_end_effector_pose.orientation.w = wp_l["orientation"][3]

            # --- 构造右臂指令 ---
            end_pose_r = agibot_gdk.EndEffectorPose()
            end_pose_r.life_time = LIFETIME
            end_pose_r.group     = agibot_gdk.EndEffectorControlGroup.kRightArm

            end_pose_r.right_end_effector_pose.position.x    = wp_r["position"][0]
            end_pose_r.right_end_effector_pose.position.y    = wp_r["position"][1]
            end_pose_r.right_end_effector_pose.position.z    = wp_r["position"][2]
            end_pose_r.right_end_effector_pose.orientation.x = wp_r["orientation"][0]
            end_pose_r.right_end_effector_pose.orientation.y = wp_r["orientation"][1]
            end_pose_r.right_end_effector_pose.orientation.z = wp_r["orientation"][2]
            end_pose_r.right_end_effector_pose.orientation.w = wp_r["orientation"][3]

            try:
                # 加入微小延时，防止底层指令被覆盖
                ret_l = self.robot.end_effector_pose_control(end_pose_l)
                time.sleep(0.002)  # 等待 2 毫秒
                ret_r = self.robot.end_effector_pose_control(end_pose_r)
                
                if ret_l != 0 or ret_r != 0:
                    print(f"  [警告] 第 {i} 步指令返回非零: 左={ret_l}, 右={ret_r}")
                    return False
            except Exception as e:
                print(f"  [错误] 第 {i} 步发送异常: {e}")
                return False

            # 维持原本的 50Hz 发送频率，扣除掉前面消耗的 2 毫秒
            time.sleep(max(0.0, dt - 0.002))
            
        return True

    # ── 主流程 ───────────────────────────────────────────────

    def adjust_arms_relative(self, offset_l=(0.0, 0.0, 0.0), offset_r=(0.0, 0.0, 0.0)) -> bool:
        """
        分别设定左右臂的相对位移 (dx, dy, dz)，单位：米。
        如果不动某只手，传入 (0.0, 0.0, 0.0) 即可。
        """
        print("=" * 55)
        print(f"准备执行调整：")
        print(f"  左臂偏移 (X, Y, Z): {offset_l}")
        print(f"  右臂偏移 (X, Y, Z): {offset_r}")
        
        # 1. 获取当前状态
        status = self.robot.get_motion_control_status()
        start_l = self._find_pose(status, LEFT_NAME)
        start_r = self._find_pose(status, RIGHT_NAME)

        # 2. 计算目标位姿
        target_l = {
            "position": [
                start_l["position"][0] + offset_l[0],
                start_l["position"][1] + offset_l[1],
                start_l["position"][2] + offset_l[2]
            ],
            "orientation": list(start_l["orientation"])
        }
        
        target_r = {
            "position": [
                start_r["position"][0] + offset_r[0],
                start_r["position"][1] + offset_r[1],
                start_r["position"][2] + offset_r[2]
            ],
            "orientation": list(start_r["orientation"])
        }

        # 3. 规划轨迹
        n_l = self._n_steps(start_l["position"], target_l["position"])
        n_r = self._n_steps(start_r["position"], target_r["position"])
        n_steps = max(n_l, n_r)
        
        if n_steps <= 1:
            print("  目标位置与当前位置过近，无需移动。")
            return True

        print(f"  规划步数: {n_steps} 步")

        traj_l = self._plan(start_l, target_l, n_steps)
        traj_r = self._plan(start_r, target_r, n_steps)

        # 4. 执行轨迹
        print("  正在执行...")
        success = self._send_dual_trajectory(traj_l, traj_r)
        
        if success:
            print("调整完成")
        else:
            print("调整失败")
        print("=" * 55)
        
        return success


# ═══════════════════════════════════════════════════════════════
#  统一入口：初始化 GDK → 执行偏移 → 释放 GDK
# ═══════════════════════════════════════════════════════════════

def run_offset(offset_l=(0.0, 0.0, 0.0), offset_r=(0.0, 0.0, 0.0)):
    """
    初始化 GDK，按给定偏移量执行双臂相对移动，最后释放 GDK。
    offset 参数格式为 (X偏移, Y偏移, Z偏移)，单位为 米。
    坐标系规则： X+(向前)， Y+(向左)， Z+(向上)
    """
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK 初始化失败")
        return
    print("GDK 初始化成功")

    robot = agibot_gdk.Robot()
    time.sleep(2)   # 等待机器人就绪

    try:
        controller = EndEffectorController(robot)
        controller.adjust_arms_relative(offset_l=offset_l, offset_r=offset_r)
    except Exception as e:
        print(f"[运行错误] {e}")

    # 释放GDK系统资源
    if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
        print("GDK释放失败")
    else:
        print("GDK释放成功")
