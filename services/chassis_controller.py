#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_controller.py — G2机器人控制封装类

包含：
  - 自动从地图读取导航点
  - normal_navi 导航
  - high_precision_navi 高精度导航
  - relative_move 相对运动
  - 蟹行横移
  - 前进/后退
  - 原地旋转

用法示例：
    from robot_controller import RobotController

    robot = RobotController()

    # 查看地图导航点
    robot.list_waypoints()

    # 导航到地图导航点（按索引或名称）
    robot.go(0)
    robot.go("1")
    robot.go(1, high_precision=True)

    # 序列导航
    robot.go_sequence([0, 1])

    # 蟹行横移（正=左，负=右，单位米）
    robot.crab_walk(1.5)
    robot.crab_walk(-2.0)

    # 前进/后退（正=前，负=后）
    robot.move_forward(1.0)
    robot.move_forward(-0.5)

    # 原地旋转（正=左转，负=右转，单位弧度）
    import math
    robot.rotate(math.pi / 2)   # 左转90度
    robot.rotate(-math.pi / 4)  # 右转45度

    # 相对运动（基于当前位姿做相对位移）
    robot.go_rel(dx=0.5)                        # 前进0.5米
    robot.go_rel(dx=-0.3)                       # 后退0.3米
    robot.go_rel(dx=0.5, dy=0.2)                # 前进0.5米并左移0.2米
    robot.go_rel(yaw_rad=math.pi / 2)           # 原地左转90度
    robot.go_rel(dx=0.5, yaw_rad=math.pi / 4)   # 前进0.5米并左转45度
"""

import json
import math
import os
import signal
import sys
import time
from typing import List, Optional, Union

import agibot_gdk


# ══════════════════════════════════════════════════
#  常量
# ══════════════════════════════════════════════════

# 导航任务状态码
_S = {
    0: "空闲",   1: "启动中", 2: "运行中", 3: "暂停中",
    4: "已暂停", 5: "恢复中", 6: "取消中", 7: "已取消",
    8: "失败",   9: "成功",
}
_DONE = {7, 8, 9}

# 底盘控制默认参数
DEFAULT_LINEAR_SPEED  = 0.25   # m/s 前进/后退/蟹行
DEFAULT_ANGULAR_SPEED = 0.4    # rad/s 旋转
CTRL_HZ               = 20     # 控制频率
CTRL_DT               = 1.0 / CTRL_HZ
STOP_SPEED_THRESH     = 0.05   # m/s 认为已停止的速度阈值


# ══════════════════════════════════════════════════
#  RobotController
# ══════════════════════════════════════════════════

class RobotController:
    """
    G2机器人控制封装类

    Parameters
    ----------
    waypoints_json : str, optional
        额外的 waypoints.json 路径，不传则自动找同目录的文件
    verbose : bool
        是否打印详细日志，默认 True
    """

    def __init__(self,
                 waypoints_json: Optional[str] = None,
                 verbose: bool = True):
        self._verbose = verbose
        self._chassis_mode = -1   # -1=未申请, 0=Ackermann, 1=蟹行

        self._log("正在初始化 GDK...")
        agibot_gdk.gdk_init()
        self.pnc  = agibot_gdk.Pnc()
        self.slam = agibot_gdk.Slam()
        time.sleep(1.5)

        # 读取地图导航点（失败时降级为空列表，不影响其他模块启动）
        self._log("正在从地图读取导航点...")
        self.waypoints, self.map_id = self._load_map_waypoints()
        if self.map_id:
            self._log(f"地图 id={self.map_id}，读到 {len(self.waypoints)} 个导航点")
        else:
            self._log(f"⚠ 未读到地图（导航功能将不可用），仅依赖 waypoints.json")

        # 合并 waypoints.json
        json_path = waypoints_json or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "waypoints.json"
        )
        if os.path.exists(json_path):
            with open(json_path, encoding="utf-8") as f:
                extra = json.load(f)
            for k, v in extra.items():
                if k not in self.waypoints:
                    v["source"] = "json"
                    self.waypoints[k] = v
            self._log(f"已合并 {json_path}，共 {len(self.waypoints)} 个导航点")

        self._wp_list = list(self.waypoints.keys())

        # 注册退出信号
        signal.signal(signal.SIGINT,  self._on_exit)
        signal.signal(signal.SIGTERM, self._on_exit)

    # ── 内部工具 ──────────────────────────────────

    def _log(self, msg: str):
        if self._verbose:
            print(msg)

    def _on_exit(self, *_):
        print("\n收到退出信号，停止机器人...")
        self._stop_chassis()
        self._cancel_navi()
        sys.exit(0)

    def _load_map_waypoints(self) -> tuple:
        """读取地图导航点。失败时返回空列表，避免阻塞服务启动（导航功能降级）。"""
        m = agibot_gdk.Map()
        time.sleep(1.0)
        try:
            curr = m.get_curr_map()
            result = m.get_map(curr.id)
        except Exception as e:
            self._log(f"⚠ 读取地图失败（导航功能将不可用）: {e}")
            return {}, ""
        waypoints = {}
        for pt in result.guide_pts:
            name = str(pt.id)
            waypoints[name] = {
                "position":    [pt.pt.position.x,    pt.pt.position.y,    pt.pt.position.z],
                "orientation": [pt.pt.orientation.x, pt.pt.orientation.y,
                                pt.pt.orientation.z, pt.pt.orientation.w],
                "type":   pt.type,
                "source": "map",
            }
        return waypoints, curr.id

    def _task_state(self):
        ts = self.pnc.get_task_state()
        return ts.state, ts.id, ts.message

    def _cancel_navi(self):
        try:
            ts = self.pnc.get_task_state()
            if ts.state not in _DONE:
                self.pnc.cancel_task(ts.id)
                time.sleep(0.3)
        except Exception:
            pass

    def _request_chassis(self, mode: int):
        """申请底盘控制权（0=Ackermann，1=蟹行），避免重复申请"""
        if self._chassis_mode != mode:
            self.pnc.request_chassis_control(mode)
            time.sleep(0.3)
            self._chassis_mode = mode

    def _make_twist(self, vx=0.0, vy=0.0, wz=0.0) -> agibot_gdk.Twist:
        twist = agibot_gdk.Twist()
        twist.linear  = agibot_gdk.Vector3()
        twist.angular = agibot_gdk.Vector3()
        twist.linear.x  = vx
        twist.linear.y  = vy
        twist.linear.z  = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = wz
        return twist

    def _stop_chassis(self):
        try:
            twist = self._make_twist()
            for _ in range(5):
                self.pnc.move_chassis(twist)
                time.sleep(0.05)
        except Exception:
            pass

    def _get_speed(self) -> float:
        try:
            odom = self.slam.get_odom_info()
            return math.hypot(odom.velocity.x, odom.velocity.y)
        except Exception:
            return 0.0

    def _wait_stop(self, timeout: float = 3.0):
        """等待机器人停稳"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._get_speed() < STOP_SPEED_THRESH:
                break
            time.sleep(0.1)

    def _resolve_wp(self, key: Union[int, str]) -> Optional[str]:
        """将索引或名称解析为导航点名称"""
        # 整数索引
        if isinstance(key, int):
            if 0 <= key < len(self._wp_list):
                return self._wp_list[key]
            return None
        # 字符串：先按索引再按名称
        try:
            idx = int(key)
            if 0 <= idx < len(self._wp_list):
                return self._wp_list[idx]
        except ValueError:
            pass
        return key if key in self.waypoints else None

    # ── 公开接口：信息查询 ────────────────────────

    def list_waypoints(self):
        """打印所有导航点"""
        print(f"\n{'─'*62}")
        print(f"{'索引':<6} {'名称':<10} {'来源':<6} {'位置 (x, y)'}")
        print(f"{'─'*62}")
        for i, name in enumerate(self._wp_list):
            wp  = self.waypoints[name]
            pos = wp["position"]
            src = wp.get("source", "?")
            print(f"[{i}]    {name:<10} {src:<6} ({pos[0]:.4f}, {pos[1]:.4f})")
        print(f"{'─'*62}")
        print(f"共 {len(self.waypoints)} 个导航点\n")

    def get_current_pose(self) -> dict:
        """返回当前位姿字典 {position:[x,y,z], orientation:[qx,qy,qz,qw]}"""
        odom = self.slam.get_odom_info()
        pos  = odom.pose.pose.position
        ori  = odom.pose.pose.orientation
        return {
            "position":    [pos.x, pos.y, pos.z],
            "orientation": [ori.x, ori.y, ori.z, ori.w],
            "loc_state":   odom.loc_state,
            "loc_confidence": odom.loc_confidence,
        }

    # ── 公开接口：导航 ────────────────────────────

    def go_adjusted(self,
           waypoint: Union[int, str],
           high_precision: bool = False,
           timeout: float = 120.0) -> bool:
        """
        导航到指定导航点

        Parameters
        ----------
        waypoint : int 或 str
            导航点索引（0,1,2...）或名称（"1","2"...）
        high_precision : bool
            是否使用高精度导航（需要 loc_state=2）
        timeout : float
            超时时间（秒）

        Returns
        -------
        bool : True=成功到达，False=失败
        """
        name = self._resolve_wp(waypoint)
        if name is None:
            self._log(f"❌ 找不到导航点: '{waypoint}'")
            return False

        wp  = self.waypoints[name]
        pos = wp["position"]
        ori = wp["orientation"]
        src = wp.get("source", "?")
        mode_str = "高精度" if high_precision else "普通"

        # 取消旧任务
        state, _, _ = self._task_state()
        if state not in _DONE and state != 0:
            self._log("   取消旧任务...")
            self._cancel_navi()

        # 构建请求
        req = agibot_gdk.NaviReq()
        if(waypoint == 2):
            pos[0] = 0.2494
            pos[1] = -0.3
    
        if(waypoint == 11):
            pos[0] = -1.1761968932515214-0.8
            pos[1] = -4.018110224686903+0.08
            ori[2] = 0
        if(waypoint == 12):
            pos[0] = -1.1761968932515214 + 0.03+0.07+0.04-0.05
            pos[1] = -4.018110224686903+0.08
            ori[2] = 0
        if(waypoint == 23):
            pos[0] = 1.4891060183247533
            pos[1] = -3.9544867812030793+0.05
            ori[2] = -1
            ori[3] = -0.013
        if(waypoint == 25):
            pos[0] = 1.3296321001429732+0.05
            pos[1] = -3.9544867812030793+0.05
            ori[2] = -1
            ori[3] = -0.013
        if(waypoint == 27):
            pos[0] =-1.4082537923221872+0.34
            pos[1] = -1.452903378780695
            ori[2] = 0
            ori[3] = 1
        if(waypoint == 26):

            pos[1] = -1.4508356073064421-0.045

        if(waypoint == 32):
            pos[0] =0.09965588715268747-0.02
            pos[1] = -0.5806166148205059-0.025
            ori[2] = -0.6997130863506108
            ori[3] = 0.7144239615170411
        req.target.position.x    = pos[0]
        req.target.position.y    = pos[1]
        req.target.position.z    = pos[2]
        req.target.orientation.x = ori[0]
        req.target.orientation.y = ori[1]
        req.target.orientation.z = ori[2]
        req.target.orientation.w = ori[3]
        print(pos)
        print(ori)


        self._log(f"\n🚀 导航到: '{name}' [{src}] ({mode_str})")
        self._log(f"   目标: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")

        try:
            if high_precision:
                self.pnc.high_precision_navi(req)
            else:
                self.pnc.normal_navi(req)
        except Exception as e:
            self._log(f"❌ 发送失败: {e}")
            return False

        # 等待启动
        self._log("   等待启动...", )
        started = False
        for _ in range(20):
            time.sleep(0.5)
            state, _, msg = self._task_state()
            if state == 2:
                started = True
                self._log(" 已启动")
                break
            if state in _DONE:
                break

        if not started:
            state, _, msg = self._task_state()
            if state == 9:
                self._log(f"✅ '{name}' 已在目标点附近，无需移动")
                return True
            self._log(f"❌ 任务未能启动: {_S.get(state, state)}  {msg}")
            return False

        # 等待完成
        start = time.time()
        while time.time() - start < timeout:
            time.sleep(0.5)
            state, _, msg = self._task_state()
            elapsed = time.time() - start
            if self._verbose:
                print(f"\r   {_S.get(state, state)}... {elapsed:.0f}s/{timeout:.0f}s",
                      end="", flush=True)

            if state == 9:
                self._wait_stop()
                elapsed = time.time() - start
                self._log(f"\n✅ 已到达 '{name}'！耗时 {elapsed:.1f}s")
                return True

            if state in {7, 8}:
                self._log(f"\n❌ 导航失败: {_S.get(state, state)}  {msg}")
                return False

        self._log(f"\n⏰ 超时，取消任务")
        self._cancel_navi()
        return False

    def go(self,
           waypoint: Union[int, str],
           high_precision: bool = False,
           timeout: float = 120.0) -> bool:
        """
        导航到指定导航点

        Parameters
        ----------
        waypoint : int 或 str
            导航点索引（0,1,2...）或名称（"1","2"...）
        high_precision : bool
            是否使用高精度导航（需要 loc_state=2）
        timeout : float
            超时时间（秒）

        Returns
        -------
        bool : True=成功到达，False=失败
        """
        name = self._resolve_wp(waypoint)
        if name is None:
            self._log(f"❌ 找不到导航点: '{waypoint}'")
            return False

        wp  = self.waypoints[name]
        pos = wp["position"]
        ori = wp["orientation"]
        src = wp.get("source", "?")
        mode_str = "高精度" if high_precision else "普通"

        # 取消旧任务
        state, _, _ = self._task_state()
        if state not in _DONE and state != 0:
            self._log("   取消旧任务...")
            self._cancel_navi()

        # 构建请求
        req = agibot_gdk.NaviReq()
        req.target.position.x    = pos[0]
        req.target.position.y    = pos[1]
        req.target.position.z    = pos[2]
        req.target.orientation.x = ori[0]
        req.target.orientation.y = ori[1]
        req.target.orientation.z = ori[2]
        req.target.orientation.w = ori[3]

        self._log(f"\n🚀 导航到: '{name}' [{src}] ({mode_str})")
        self._log(f"   目标: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")

        try:
            if high_precision:
                self.pnc.high_precision_navi(req)
            else:
                self.pnc.normal_navi(req)
        except Exception as e:
            self._log(f"❌ 发送失败: {e}")
            return False

        # 等待启动
        self._log("   等待启动...", )
        started = False
        for _ in range(20):
            time.sleep(0.5)
            state, _, msg = self._task_state()
            if state == 2:
                started = True
                self._log(" 已启动")
                break
            if state in _DONE:
                break

        if not started:
            state, _, msg = self._task_state()
            if state == 9:
                self._log(f"✅ '{name}' 已在目标点附近，无需移动")
                return True
            self._log(f"❌ 任务未能启动: {_S.get(state, state)}  {msg}")
            return False

        # 等待完成
        start = time.time()
        while time.time() - start < timeout:
            time.sleep(0.5)
            state, _, msg = self._task_state()
            elapsed = time.time() - start
            if self._verbose:
                print(f"\r   {_S.get(state, state)}... {elapsed:.0f}s/{timeout:.0f}s",
                      end="", flush=True)

            if state == 9:
                self._wait_stop()
                elapsed = time.time() - start
                self._log(f"\n✅ 已到达 '{name}'！耗时 {elapsed:.1f}s")
                return True

            if state in {7, 8}:
                self._log(f"\n❌ 导航失败: {_S.get(state, state)}  {msg}")
                return False

        self._log(f"\n⏰ 超时，取消任务")
        self._cancel_navi()
        return False

    def go_by_number(self,
                     num: int,
                     high_precision: bool = False,
                     timeout: float = 120.0) -> bool:
        """
        使用数字编号导航到指定点位（需要先运行 rename_waypoints.py 生成配置）
        
        Parameters
        ----------
        num : int
            导航点数字编号（1, 2, 3...），对应重命名后的顺序
        high_precision : bool
            是否使用高精度导航
        timeout : float
            超时时间（秒）
        
        Returns
        -------
        bool : True=成功到达，False=失败
        """
        import json
        
        json_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "waypoints_renamed.json"
        )
        
        if not os.path.exists(json_path):
            self._log(f"❌ 未找到重命名配置文件: {json_path}")
            self._log("   请先运行: python3 rename_waypoints.py --map-id 7")
            return False
        
        try:
            with open(json_path, encoding='utf-8') as f:
                data = json.load(f)
            
            # 查找对应的原始ID
            target_name = str(num)
            for wp in data['waypoints']:
                if wp['name'] == target_name:
                    original_name = wp['original_name']
                    self._log(f"📌 将数字 {num} 映射到原始导航点: '{original_name}'")
                    return self.go(original_name, high_precision, timeout)
            
            self._log(f"❌ 未找到数字编号为 '{num}' 的导航点")
            return False
            
        except Exception as e:
            self._log(f"❌ 读取重命名配置文件失败: {e}")
            return False

    def go_sequence(self,
                    waypoints: List[Union[int, str]],
                    high_precision: bool = False,
                    timeout: float = 120.0,
                    stop_on_fail: bool = True) -> bool:
        """
        依次导航到多个导航点

        Parameters
        ----------
        waypoints : list
            导航点索引或名称列表，如 [0, 1, 2] 或 ["1", "2"]
        high_precision : bool
            是否使用高精度导航
        timeout : float
            每个点的超时时间（秒）
        stop_on_fail : bool
            某个点失败时是否停止后续导航

        Returns
        -------
        bool : True=全部成功
        """
        names = []
        for wp in waypoints:
            name = self._resolve_wp(wp)
            if name is None:
                self._log(f"❌ 找不到导航点: '{wp}'")
                return False
            names.append(name)

        self._log(f"\n📍 序列导航: {' → '.join(names)}")
        all_ok = True
        for i, name in enumerate(names):
            self._log(f"\n[{i+1}/{len(names)}]")
            ok = self.go(name, high_precision, timeout)
            if not ok:
                all_ok = False
                if stop_on_fail:
                    self._log(f"⛔ 在 '{name}' 失败，停止序列")
                    break
                else:
                    self._log(f"⚠️  在 '{name}' 失败，继续下一个")
            if i < len(names) - 1:
                time.sleep(1.0)

        self._log("\n🎉 序列导航完成！" if all_ok else "\n⚠️  序列导航未完全成功")
        return all_ok

    def go_rel(self,
               dx: float = 0.0,
               dy: float = 0.0,
               dz: float = 0.0,
               yaw_rad: float = 0.0,
               timeout: float = 60.0) -> bool:
        """
        相对运动（基于当前位姿做相对位移）

        使用 pnc.relative_move()，目标位姿为相对当前位姿的偏移量。

        Parameters
        ----------
        dx : float
            x 方向相对位移（米），正=前进，负=后退
        dy : float
            y 方向相对位移（米），正=左，负=右
        dz : float
            z 方向相对位移（米），通常为 0
        yaw_rad : float
            相对旋转角度（弧度），正=左转，负=右转
            默认 0 表示保持当前朝向（对应四元数 w=1）
        timeout : float
            超时时间（秒）

        Returns
        -------
        bool : True=成功到达，False=失败
        """
        # 无位移无旋转，直接返回
        if abs(dx) < 0.001 and abs(dy) < 0.001 and abs(dz) < 0.001 \
                and abs(yaw_rad) < 0.001:
            return True

        # 取消旧任务
        state, _, _ = self._task_state()
        if state not in _DONE and state != 0:
            self._log("   取消旧任务...")
            self._cancel_navi()

        # yaw（绕 z 轴）转四元数：q = (0, 0, sin(yaw/2), cos(yaw/2))
        half = yaw_rad / 2.0
        qz = math.sin(half)
        qw = math.cos(half)

        # 构建相对移动请求
        req = agibot_gdk.NaviReq()
        req.target.position.x    = dx
        req.target.position.y    = dy
        req.target.position.z    = dz
        req.target.orientation.x = 0.0
        req.target.orientation.y = 0.0
        req.target.orientation.z = qz
        req.target.orientation.w = qw

        self._log(f"\n🚀 相对运动: dx={dx:+.2f}m  dy={dy:+.2f}m  dz={dz:+.2f}m  "
                  f"yaw={math.degrees(yaw_rad):+.1f}°")

        try:
            self.pnc.relative_move(req)
        except Exception as e:
            self._log(f"❌ 发送失败: {e}")
            return False

        self._log("   相对移动请求发送成功")

        # 等待启动
        started = False
        for _ in range(20):
            time.sleep(0.5)
            state, _, msg = self._task_state()
            if state == 2:
                started = True
                self._log(" 已启动")
                break
            if state in _DONE:
                break

        if not started:
            state, _, msg = self._task_state()
            if state == 9:
                self._log("✅ 相对运动已完成（已在目标点附近）")
                return True
            self._log(f"❌ 任务未能启动: {_S.get(state, state)}  {msg}")
            return False

        # 等待完成
        start = time.time()
        while time.time() - start < timeout:
            time.sleep(0.5)
            state, _, msg = self._task_state()
            elapsed = time.time() - start
            if self._verbose:
                print(f"\r   {_S.get(state, state)}... {elapsed:.0f}s/{timeout:.0f}s",
                      end="", flush=True)

            if state == 9:
                self._wait_stop()
                elapsed = time.time() - start
                self._log(f"\n✅ 相对运动完成！耗时 {elapsed:.1f}s")
                return True

            if state in {7, 8}:
                self._log(f"\n❌ 相对运动失败: {_S.get(state, state)}  {msg}")
                return False

        self._log(f"\n⏰ 超时，取消任务")
        self._cancel_navi()
        return False

    # ── 公开接口：底盘运动 ────────────────────────

    def crab_walk(self,
                  dist_m: float,
                  speed: float = DEFAULT_LINEAR_SPEED) -> bool:
        """
        蟹行横移

        Parameters
        ----------
        dist_m : float
            横移距离（米），正数=左，负数=右
        speed : float
            横向速度（m/s），默认 0.25

        Returns
        -------
        bool : True=完成
        """
        if abs(dist_m) < 0.01:
            return True

        self._request_chassis(1)  # 蟹行模式

        direction = 1.0 if dist_m >= 0 else -1.0
        vy = direction * abs(speed)
        duration = abs(dist_m) / abs(speed)

        self._log(f"\n↔️  蟹行: {dist_m:+.2f}m  速度: {vy:+.2f}m/s  预计: {duration:.1f}s")

        twist = self._make_twist(vy=vy)
        start = time.time()

        try:
            while time.time() - start < duration:
                self.pnc.move_chassis(twist)
                time.sleep(CTRL_DT)
                if self._verbose:
                    elapsed = time.time() - start
                    print(f"\r   已移动: {elapsed/duration*abs(dist_m):.2f}m / {abs(dist_m):.2f}m",
                          end="", flush=True)
        except KeyboardInterrupt:
            self._log("\n⚠️  用户中断")
        finally:
            self._stop_chassis()
            self._wait_stop()

        self._log(f"\n✅ 蟹行完成，用时 {time.time()-start:.1f}s")
        return True

    def move_forward(self,
                     dist_m: float,
                     speed: float = DEFAULT_LINEAR_SPEED) -> bool:
        """
        前进/后退

        Parameters
        ----------
        dist_m : float
            移动距离（米），正数=前进，负数=后退
        speed : float
            速度（m/s）

        Returns
        -------
        bool : True=完成
        """
        if abs(dist_m) < 0.01:
            return True

        self._request_chassis(0)  # Ackermann 模式

        direction = 1.0 if dist_m >= 0 else -1.0
        vx = direction * abs(speed)
        duration = abs(dist_m) / abs(speed)

        self._log(f"\n⬆️  前进: {dist_m:+.2f}m  速度: {vx:+.2f}m/s  预计: {duration:.1f}s")

        twist = self._make_twist(vx=vx)
        start = time.time()

        try:
            while time.time() - start < duration:
                self.pnc.move_chassis(twist)
                time.sleep(CTRL_DT)
                if self._verbose:
                    elapsed = time.time() - start
                    print(f"\r   已移动: {elapsed/duration*abs(dist_m):.2f}m / {abs(dist_m):.2f}m",
                          end="", flush=True)
        except KeyboardInterrupt:
            self._log("\n⚠️  用户中断")
        finally:
            self._stop_chassis()
            self._wait_stop()

        self._log(f"\n✅ 前进完成，用时 {time.time()-start:.1f}s")
        return True

    def rotate(self,
               angle_rad: float,
               speed: float = DEFAULT_ANGULAR_SPEED) -> bool:
        """
        原地旋转

        Parameters
        ----------
        angle_rad : float
            旋转角度（弧度），正数=左转，负数=右转
            可以用 math.radians(90) 传入角度
        speed : float
            角速度（rad/s）

        Returns
        -------
        bool : True=完成
        """
        if abs(angle_rad) < 0.01:
            return True

        self._request_chassis(0)

        direction = 1.0 if angle_rad >= 0 else -1.0
        wz = direction * abs(speed)
        duration = abs(angle_rad) / abs(speed)

        deg = math.degrees(angle_rad)
        self._log(f"\n🔄  旋转: {deg:+.1f}°  角速度: {wz:+.2f}rad/s  预计: {duration:.1f}s")

        twist = self._make_twist(wz=wz)
        start = time.time()

        try:
            while time.time() - start < duration:
                self.pnc.move_chassis(twist)
                time.sleep(CTRL_DT)
                if self._verbose:
                    elapsed = time.time() - start
                    print(f"\r   已旋转: {elapsed/duration*abs(deg):.1f}° / {abs(deg):.1f}°",
                          end="", flush=True)
        except KeyboardInterrupt:
            self._log("\n⚠️  用户中断")
        finally:
            self._stop_chassis()
            self._wait_stop()

        self._log(f"\n✅ 旋转完成，用时 {time.time()-start:.1f}s")
        return True


# ══════════════════════════════════════════════════
#  命令行入口（直接运行时用）
# ══════════════════════════════════════════════════

if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="G2机器人控制")
    ap.add_argument("--list",           action="store_true",  help="列出所有导航点")
    ap.add_argument("--wp",             type=str,             help="导航到指定点（索引或名称）")
    ap.add_argument("--seq",            type=str, nargs="+",  help="序列导航")
    ap.add_argument("--high-precision", action="store_true",  help="高精度导航")
    ap.add_argument("--crab",           type=float,           help="蟹行横移距离（米，正=左，负=右）")
    ap.add_argument("--forward",        type=float,           help="前进距离（米，正=前，负=后）")
    ap.add_argument("--rotate",         type=float,           help="旋转角度（度，正=左，负=右）")
    ap.add_argument("--rel",            type=float, nargs="+", metavar=("DX", "DY", "YAW_DEG"),
                                              help="相对运动：dx dy [yaw_deg]，如 --rel 0.5 0 0")
    ap.add_argument("--speed",          type=float,           help="运动速度覆盖")
    ap.add_argument("--timeout",        type=float, default=120.0)
    args = ap.parse_args()

    robot = RobotController()

    if args.list:
        robot.list_waypoints()

    elif args.wp:
        robot.go(args.wp, args.high_precision, args.timeout)

    elif args.seq:
        robot.go_sequence(args.seq, args.high_precision, args.timeout)

    elif args.crab is not None:
        spd = args.speed or DEFAULT_LINEAR_SPEED
        robot.crab_walk(args.crab, spd)

    elif args.forward is not None:
        spd = args.speed or DEFAULT_LINEAR_SPEED
        robot.move_forward(args.forward, spd)

    elif args.rotate is not None:
        spd = args.speed or DEFAULT_ANGULAR_SPEED
        robot.rotate(math.radians(args.rotate), spd)

    elif args.rel is not None:
        dx = args.rel[0] if len(args.rel) > 0 else 0.0
        dy = args.rel[1] if len(args.rel) > 1 else 0.0
        yaw_deg = args.rel[2] if len(args.rel) > 2 else 0.0
        robot.go_rel(dx=dx, dy=dy, yaw_rad=math.radians(yaw_deg),
                     timeout=args.timeout)

    else:
        robot.list_waypoints()
        print("用法: python3 robot_controller.py --wp 0")
        print("      python3 robot_controller.py --crab 1.5")
        print("      python3 robot_controller.py --forward 2.0")
        print("      python3 robot_controller.py --rotate 90")
        print("      python3 robot_controller.py --rel 0.5 0 0")
