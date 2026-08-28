#!/usr/bin/env python3
"""lerobot 对接程序：将 G2 机器人真实状态通过 TCP 暴露给 lerobot 端

三个独立 TCP 服务（参考 lerobot/arm_simulator.py 与 lerobot/camera_simulator.py 协议）：

  端口 9002 (arm)：
      请求 `get\n`            -> 返回 `[j1, j2, j3, j4, j5, j6, j7, gripper]\n`
                                  前 7 个为右臂关节位置（rad，与 G2 URDF 一致）
                                  第 8 个为右臂末端 omnipicker 夹爪位置（rad，范围约 [-0.785, 0]）
        请求 `set [..8个数..]\n` -> 默认返回 `ok\n`（仅 ACK，不下发控制，避免误操作）
                              启动加 --enable-control 后，真正下发右臂关节位置 + 末端夹爪位置

  端口 9003 (head rgb)：
      请求 `get\n`            -> 返回 `<4 字节大端长度><JPEG 字节>`

  端口 9004 (right wrist rgb)：
      请求 `get\n`            -> 返回 `<4 字节大端长度><JPEG 字节>`

性能优化（v2）：
  - 后台采集线程持续从 GDK 拉取最新帧 / 关节状态，缓存到内存
  - client `get` 请求直接读缓存返回，零 GDK 调用、零 JPEG 编码
  - 默认采集 30Hz，可覆盖 lerobot 端 15Hz 需求
  - 启动时 warmup 验证 GDK 连通性

依赖：agibot_gdk、numpy、opencv-python
"""

import argparse
import json
import socket
import struct
import threading
import time
from typing import List, Optional

import cv2
import numpy as np

import agibot_gdk


# ---------------------------------------------------------------------------
# 配置
# ---------------------------------------------------------------------------
HOST = "0.0.0.0"
ARM_PORT = 9002
HEAD_CAM_PORT = 9003
WRIST_CAM_PORT = 9004

# 右臂 7 关节名称（G2 标准 URDF，与 robot_demo.py 一致）
# 注：经 probe_gdk.py 验证，G2 实际关节命名前缀为 idx6N_arm_r_jointN
RIGHT_ARM_JOINT_NAMES: List[str] = [
    "idx61_arm_r_joint1",
    "idx62_arm_r_joint2",
    "idx63_arm_r_joint3",
    "idx64_arm_r_joint4",
    "idx65_arm_r_joint5",
    "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]

# GDK 初始化后等待 DDS 建链的休眠时间
GDK_INIT_WAIT_S = 2.0
# 单次取图/取状态的超时（ms）
GDK_QUERY_TIMEOUT_MS = 1000.0
# 后台采集线程目标频率（Hz），高于 lerobot 默认 15Hz
COLLECT_FPS = 10.0
# 后台采集失败时日志的最小间隔（秒），避免刷屏
ERROR_LOG_INTERVAL_S = 5.0


# ---------------------------------------------------------------------------
# G2 状态采集器（后台线程持续采集 + 缓存）
# ---------------------------------------------------------------------------
class G2StateProvider:
    """封装 GDK Robot + Camera，提供后台采集与零延迟读取

    内部启动 3 个后台线程：
      - arm_collector：30Hz 拉取关节状态 + 末端夹爪，缓存 8 个 float
      - head_cam_collector：30Hz 拉取头部相机帧并编码 JPEG，缓存 bytes
      - wrist_cam_collector：30Hz 拉取腕部相机帧并编码 JPEG，缓存 bytes

    client `get` 请求直接读缓存返回，无 GDK 调用、无 JPEG 编码延迟。
    """

    def __init__(self, jpeg_quality: int = 40, enable_control: bool = False):
        self.robot: Optional[agibot_gdk.Robot] = None
        self.camera: Optional[agibot_gdk.Camera] = None
        self.jpeg_quality = jpeg_quality
        self.enable_control = enable_control
        self._jpeg_encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)]

        # 关节状态缓存
        self._arm_lock = threading.Lock()
        self._arm_state: Optional[List[float]] = None
        self._arm_ts: float = 0.0

        # 相机帧缓存
        self._head_lock = threading.Lock()
        self._head_jpg: Optional[bytes] = None
        self._head_ts: float = 0.0

        self._wrist_lock = threading.Lock()
        self._wrist_jpg: Optional[bytes] = None
        self._wrist_ts: float = 0.0

        # 后台线程控制
        self._stop_event = threading.Event()
        self._threads: List[threading.Thread] = []

        # 错误日志限频
        self._last_err_log: dict = {}
        self._err_log_lock = threading.Lock()

    # ---------- 生命周期 ----------
    def initialize(self) -> None:
        if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError("GDK 初始化失败")
        print("[dock] GDK 初始化成功", flush=True)

        self.robot = agibot_gdk.Robot()
        self.camera = agibot_gdk.Camera()
        time.sleep(GDK_INIT_WAIT_S)  # 等待 DDS 建链
        print("[dock] Robot / Camera 对象就绪", flush=True)

        # 相机预热：直接调用一次 get_latest_image，验证连通性
        # 同时触发 GDK 内部订阅/缓存建立
        self._warmup_cameras()

        # 启动后台采集线程
        self._start_collectors()

    def _warmup_cameras(self) -> None:
        """对每个用到的相机做一次预热查询，打印结果"""
        cams = [
            ("kHeadColor", agibot_gdk.CameraType.kHeadColor),
            ("kHandRightColor", agibot_gdk.CameraType.kHandRightColor),
        ]
        for name, ctype in cams:
            try:
                img = self.camera.get_latest_image(ctype, GDK_QUERY_TIMEOUT_MS)
                if img is None:
                    print(f"[dock] warmup {name}: image is None", flush=True)
                else:
                    data = getattr(img, "data", None)
                    n = len(data) if data is not None else 0
                    print(f"[dock] warmup {name}: "
                          f"{img.width}x{img.height}  encoding={img.encoding}  "
                          f"data_len={n}", flush=True)
            except Exception as e:
                print(f"[dock] warmup {name} 异常: {type(e).__name__}: {e}",
                      flush=True)

    def release(self) -> None:
        self._stop_event.set()
        for t in self._threads:
            t.join(timeout=2.0)
        if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
            print("[dock] GDK 释放失败", flush=True)
        else:
            print("[dock] GDK 释放成功", flush=True)

    # ---------- 右臂控制（推理用） ----------
    def set_right_arm_state(self, joints: List[float], gripper: float) -> None:
        """下发右臂 7 关节 + 末端夹爪位置到 G2 机器人

        参数:
            joints: 7 个关节位置（rad），顺序与 RIGHT_ARM_JOINT_NAMES 一致
            gripper: omnipicker 夹爪位置（rad，范围约 [-0.785, 0]）

        使用 GDK 正确 API：
          - 关节：JointControlReq + robot.joint_control_request（参考 mc_example.py）
          - 夹爪：JointStates + robot.move_ee_pos（参考 robot_demo.py test_move_ee_pos）
        """
        if not self.enable_control:
            return  # 控制未启用，静默返回（仅 ACK）

        if self.robot is None:
            raise RuntimeError("Robot not initialized")

        if len(joints) != 7:
            raise ValueError(f"joints must be 7-dim, got {len(joints)}")

        # 限幅：夹爪范围 [-0.785, 0.0]，关节位置不在此处限幅（由 URDF 决定）
        gripper_clamped = max(-0.7, min(0.0, float(gripper)))
       

        # ---- 1. 下发 7 个右臂关节位置 ----
        try:
            req = agibot_gdk.JointControlReq()
            req.life_time = 1.0  # 命令有效期 1s（lerobot 默认每帧 ~66ms，足够）
            req.joint_names = list(RIGHT_ARM_JOINT_NAMES)
            req.joint_positions = [float(v) for v in joints]
            print(joints)
            req.joint_velocities = [0.3] * 7  # 与 mc_example.py 默认一致
            self.robot.joint_control_request(req)
            time.sleep(0.02)
        except Exception as e:
            raise RuntimeError(
                f"joint_control_request failed: {type(e).__name__}: {e}"
            ) from e

        # ---- 2. 下发末端夹爪位置（omnipicker） ----
        joint_states = agibot_gdk.JointStates()
        joint_states.group = "right_tool"
        joint_states.target_type = "omnipicker"
        joint_state = agibot_gdk.JointState()
        joint_state.position = gripper_clamped
        print(gripper_clamped)
        joint_states.states = [joint_state]
        joint_states.nums = 1
        try:


            self.robot.move_ee_pos(joint_states)
            # time.sleep(0.02)

    

        except Exception as e:
            # 夹爪下发失败不致命（关节已下发），仅打印警告
            print(f"[dock:arm] warn: move_ee_pos failed: "
                  f"{type(e).__name__}: {e}", flush=True)

    # ---------- 后台采集线程 ----------
    def _start_collectors(self) -> None:
        self._threads = [
            threading.Thread(
                target=self._arm_collector_loop,
                daemon=True, name="dock-arm-collector",
            ),
            threading.Thread(
                target=self._cam_collector_loop,
                args=("head", agibot_gdk.CameraType.kHeadColor,
                      self._head_lock, "_head_jpg", "_head_ts"),
                daemon=True, name="dock-head-collector",
            ),
            threading.Thread(
                target=self._cam_collector_loop,
                args=("wrist", agibot_gdk.CameraType.kHandRightColor,
                      self._wrist_lock, "_wrist_jpg", "_wrist_ts"),
                daemon=True, name="dock-wrist-collector",
            ),
        ]
        for t in self._threads:
            t.start()
        print(f"[dock] 后台采集线程已启动，目标 {COLLECT_FPS} Hz", flush=True)

    def _log_error(self, tag: str, msg: str) -> None:
        """错误日志限频输出"""
        now = time.time()
        with self._err_log_lock:
            last = self._last_err_log.get(tag, 0.0)
            if now - last < ERROR_LOG_INTERVAL_S:
                return
            self._last_err_log[tag] = now
        print(f"[dock:{tag}] {msg}", flush=True)

    def _arm_collector_loop(self) -> None:
        """30Hz 拉取右臂关节状态 + 末端夹爪"""
        period = 1.0 / COLLECT_FPS
        while not self._stop_event.is_set():
            t0 = time.time()
            try:
                # 1) 关节
                joint_states = self.robot.get_joint_states()
                name_to_pos = {
                    s["name"]: float(s["position"])
                    for s in joint_states["states"]
                }
                joints: List[float] = []
                ok = True
                for name in RIGHT_ARM_JOINT_NAMES:
                    if name not in name_to_pos:
                        self._log_error("arm", f"关节状态中找不到 {name}")
                        ok = False
                        break
                    joints.append(name_to_pos[name])
                if not ok:
                    time.sleep(period)
                    continue

                # 2) 末端夹爪
                end_state = self.robot.get_end_state()
                right_end = end_state["right_end_state"]
                end_states_list = right_end.get("end_states") or []
                gripper_pos = float(end_states_list[0].get("position", 0.0)) if end_states_list else 0.0

                # 3) 写入缓存
                with self._arm_lock:
                    self._arm_state = joints + [gripper_pos]
                    self._arm_ts = time.time()
            except Exception as e:
                self._log_error("arm", f"采集异常: {type(e).__name__}: {e}")

            # 精确周期控制
            elapsed = time.time() - t0
            sleep_s = period - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)

    def _cam_collector_loop(self, tag: str, camera_type, lock: threading.Lock,
                            jpg_attr: str, ts_attr: str) -> None:
        """30Hz 拉取相机帧并编码 JPEG，缓存到内存"""
        period = 1.0 / COLLECT_FPS
        while not self._stop_event.is_set():
            t0 = time.time()
            try:
                jpg = self._fetch_and_encode(camera_type, tag)
                if jpg is not None and len(jpg) > 0:
                    with lock:
                        setattr(self, jpg_attr, jpg)
                        setattr(self, ts_attr, time.time())
            except Exception as e:
                self._log_error(tag, f"采集异常: {type(e).__name__}: {e}")

            elapsed = time.time() - t0
            sleep_s = period - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)

    def _fetch_and_encode(self, camera_type, tag: str) -> Optional[bytes]:
        """从 GDK 取一帧图像并编码为 JPEG 字节（线程内部使用，不加锁）"""
        try:
            image = self.camera.get_latest_image(camera_type, GDK_QUERY_TIMEOUT_MS)
        except Exception as e:
            self._log_error(tag, f"get_latest_image 异常: {type(e).__name__}: {e}")
            return None
        if image is None:
            self._log_error(tag, "相机无帧 (image is None)")
            return None
        data = getattr(image, "data", None)
        if data is None:
            self._log_error(tag, f"相机帧 data 为 None (encoding={image.encoding})")
            return None
        try:
            n = len(data)
        except TypeError:
            n = 0
        if n == 0:
            self._log_error(tag, f"相机帧 data 为空 (encoding={image.encoding})")
            return None

        # 已是 JPEG，直接用原始字节
        if image.encoding == agibot_gdk.Encoding.JPEG:
            return bytes(image.data)
        if image.encoding == agibot_gdk.Encoding.PNG:
            nparr = np.frombuffer(image.data, np.uint8)
            decoded = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            ok, jpg = cv2.imencode(".jpg", decoded, self._jpeg_encode_param)
            return jpg.tobytes() if ok else None

        # UNCOMPRESSED 分支
        if image.encoding != agibot_gdk.Encoding.UNCOMPRESSED:
            return None

        try:
            h, w = int(image.height), int(image.width)
            cf = image.color_format
            if cf in (agibot_gdk.ColorFormat.RGB, agibot_gdk.ColorFormat.BGR):
                arr = np.frombuffer(image.data, dtype=np.uint8).reshape((h, w, 3))
                if cf == agibot_gdk.ColorFormat.RGB:
                    arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            elif cf == agibot_gdk.ColorFormat.GRAY8:
                arr = np.frombuffer(image.data, dtype=np.uint8).reshape((h, w))
                arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
            elif cf == agibot_gdk.ColorFormat.GRAY16:
                arr = np.frombuffer(image.data, dtype=np.uint16).reshape((h, w))
                arr = (arr / 256).astype(np.uint8)
                arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
            else:
                return None
            ok, jpg = cv2.imencode(".jpg", arr, self._jpeg_encode_param)
            return jpg.tobytes() if ok else None
        except Exception as e:
            self._log_error(tag, f"图像解码失败: {e}")
            return None

    # ---------- client 读取接口（直接读缓存，零延迟） ----------
    def get_right_arm_state(self) -> Optional[List[float]]:
        """返回缓存的 [j1..j7, gripper]，长度 8。无数据时返回 None"""
        with self._arm_lock:
            return list(self._arm_state) if self._arm_state is not None else None

    def get_head_jpeg(self) -> Optional[bytes]:
        with self._head_lock:
            return self._head_jpg

    def get_wrist_jpeg(self) -> Optional[bytes]:
        with self._wrist_lock:
            return self._wrist_jpg

    def get_stats(self) -> dict:
        """返回采集线程统计信息"""
        now = time.time()
        with self._arm_lock:
            arm_age = now - self._arm_ts if self._arm_ts else None
        with self._head_lock:
            head_age = now - self._head_ts if self._head_ts else None
        with self._wrist_lock:
            wrist_age = now - self._wrist_ts if self._wrist_ts else None
        return {
            "arm_age_ms": int(arm_age * 1000) if arm_age else None,
            "head_age_ms": int(head_age * 1000) if head_age else None,
            "wrist_age_ms": int(wrist_age * 1000) if wrist_age else None,
        }


# ---------------------------------------------------------------------------
# 通用 TCP 服务框架
# ---------------------------------------------------------------------------
def serve_tcp(
    port: int,
    handler,  # callable(conn, addr)
    tag: str,
):
    """启动一个 TCP 服务并阻塞运行"""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, port))
    srv.listen(4)
    print(f"[dock:{tag}] listening on {HOST}:{port}", flush=True)
    try:
        while True:
            conn, addr = srv.accept()
            threading.Thread(
                target=handler, args=(conn, addr), daemon=True,
                name=f"dock-{tag}-{addr[0]}:{addr[1]}",
            ).start()
    except KeyboardInterrupt:
        print(f"\n[dock:{tag}] shutting down", flush=True)
    finally:
        srv.close()


# ---------------------------------------------------------------------------
# 9002: 右臂关节 + 夹爪
# ---------------------------------------------------------------------------
def make_arm_handler(provider: G2StateProvider):
    def handle_client(conn: socket.socket, addr):
        print(f"[dock:arm] client connected from {addr}", flush=True)
        try:
            with conn:
                buf = bytearray()
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break
                    buf.extend(data)
                    while b"\n" in buf:
                        line, _, buf = buf.partition(b"\n")
                        request = line.decode("utf-8", errors="replace").strip()
                        if not request:
                            continue
                        try:
                            if request.startswith("get"):
                                state = provider.get_right_arm_state()
                                if state is None:
                                    # 缓存还未就绪，返回全零
                                    state = [0.0] * 8
                                reply = json.dumps([round(v, 4) for v in state]) + "\n"
                                conn.sendall(reply.encode("utf-8"))
                            elif request.startswith("set"):
                                # 解析 8 维 action: [j1..j7, gripper]
                                payload = request[len("set"):].strip()
                                values = json.loads(payload)
                                if not isinstance(values, list) or len(values) != 8:
                                    conn.sendall(
                                        f"error: expected 8-dim list, got {len(values) if isinstance(values, list) else type(values).__name__}\n".encode("utf-8")
                                    )
                                    continue

                                joints = [float(v) for v in values[:7]]
                                gripper = float(values[7])

                                if provider.enable_control:
                                    # 真正下发控制命令到 G2
                                    try:
                                        provider.set_right_arm_state(joints, gripper)
                                        conn.sendall(b"ok\n")
                                    except Exception as e:
                                        print(f"[dock:arm] set_right_arm_state 失败: "
                                              f"{type(e).__name__}: {e}", flush=True)
                                        conn.sendall(f"error: {e}\n".encode("utf-8"))
                                else:
                                    # 控制未启用，仅 ACK（保留旧行为，安全默认）
                                    conn.sendall(b"ok\n")
                            else:
                                conn.sendall(b"error\n")
                        except Exception as e:
                            print(f"[dock:arm] 处理请求 '{request}' 失败: "
                                  f"{type(e).__name__}: {e}", flush=True)
                            try:
                                conn.sendall(f"error: {e}\n".encode("utf-8"))
                            except Exception:
                                pass
        except (ConnectionError, OSError) as e:
            print(f"[dock:arm] 连接异常: {type(e).__name__}: {e}", flush=True)
        finally:
            print(f"[dock:arm] client disconnected: {addr}", flush=True)
    return handle_client


# ---------------------------------------------------------------------------
# 9003 / 9004: 相机帧
# ---------------------------------------------------------------------------
def make_camera_handler(provider: G2StateProvider, getter, tag: str):
    """getter 是一个 callable，返回缓存的 JPEG bytes 或 None"""
    def handle_client(conn: socket.socket, addr):
        print(f"[dock:{tag}] client connected from {addr}", flush=True)
        try:
            with conn:
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break
                    if b"get" in data:
                        try:
                            jpg = getter()
                        except Exception as e:
                            print(f"[dock:{tag}] get_jpeg failed: {e}", flush=True)
                            jpg = None
                        if jpg is None or len(jpg) == 0:
                            conn.sendall(struct.pack(">I", 0))
                        else:
                            conn.sendall(struct.pack(">I", len(jpg)) + jpg)
        except ConnectionError:
            pass
        finally:
            print(f"[dock:{tag}] client disconnected: {addr}", flush=True)
    return handle_client


# ---------------------------------------------------------------------------
# 统计信息打印线程
# ---------------------------------------------------------------------------
def stats_loop(provider: G2StateProvider, interval: float = 10.0):
    """周期性打印采集状态"""
    while True:
        time.sleep(interval)
        try:
            stats = provider.get_stats()
            print(f"[dock] stats: arm={stats['arm_age_ms']}ms  "
                  f"head={stats['head_age_ms']}ms  "
                  f"wrist={stats['wrist_age_ms']}ms", flush=True)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# 主入口
# ---------------------------------------------------------------------------
def main():
    global COLLECT_FPS

    parser = argparse.ArgumentParser(description="lerobot 对接程序：G2 真机 → TCP")
    parser.add_argument("--jpeg-quality", type=int, default=80,
                        help="JPEG 编码质量 (1-100，默认 80)")
    parser.add_argument("--collect-fps", type=float, default=COLLECT_FPS,
                        help=f"后台采集频率 Hz (默认 {COLLECT_FPS})")
    parser.add_argument("--enable-control", action="store_true", default=False,
                        help="启用控制：收到 set 命令时真正下发到 G2 机器人 "
                             "(默认关闭，仅 ACK，用于安全数据采集；"
                             "推理时必须加此参数机器人才能动)")
    args = parser.parse_args()
    if not (1 <= args.jpeg_quality <= 100):
        parser.error("--jpeg-quality 必须在 1..100 范围")
    if not (1.0 <= args.collect_fps <= 200.0):
        parser.error("--collect-fps 必须在 1..200 范围")

    # 全局变量覆盖（在创建 provider 之前）
    COLLECT_FPS = args.collect_fps

    provider = G2StateProvider(
        jpeg_quality=args.jpeg_quality,
        enable_control=args.enable_control,
    )
    try:
        provider.initialize()
    except Exception as e:
        print(f"[dock] 初始化失败: {e}", flush=True)
        return

    # 打印控制状态，提示用户当前是否启用控制
    if args.enable_control:
        print("=" * 60, flush=True)
        print("[dock] ⚠️  控制已启用 (CONTROL ENABLED)", flush=True)
        print("[dock] ⚠️  收到 set 命令会真正下发到 G2 机器人", flush=True)
        print("[dock] ⚠️  请确保机器人周围无人和障碍物", flush=True)
        print("=" * 60, flush=True)
    else:
        print("[dock] 控制未启用 (set 命令仅 ACK，不会动)", flush=True)
        print("[dock] 如需推理，请加 --enable-control 参数", flush=True)

    # 启动三个 TCP 服务线程
    threads: List[threading.Thread] = []

    t_arm = threading.Thread(
        target=serve_tcp,
        args=(ARM_PORT, make_arm_handler(provider), "arm"),
        daemon=True, name="dock-arm-server",
    )
    t_head = threading.Thread(
        target=serve_tcp,
        args=(HEAD_CAM_PORT,
              make_camera_handler(provider, provider.get_head_jpeg, "head_cam"),
              "head_cam"),
        daemon=True, name="dock-head-cam-server",
    )
    t_wrist = threading.Thread(
        target=serve_tcp,
        args=(WRIST_CAM_PORT,
              make_camera_handler(provider, provider.get_wrist_jpeg, "wrist_cam"),
              "wrist_cam"),
        daemon=True, name="dock-wrist-cam-server",
    )
    for t in (t_arm, t_head, t_wrist):
        t.start()
        threads.append(t)

    # 启动统计线程
    t_stats = threading.Thread(
        target=stats_loop, args=(provider, 10.0),
        daemon=True, name="dock-stats",
    )
    t_stats.start()

    print(f"[dock] 全部服务就绪："
          f"arm={ARM_PORT}  head_cam={HEAD_CAM_PORT}  wrist_cam={WRIST_CAM_PORT}",
          flush=True)
    print(f"[dock] 采集频率 {COLLECT_FPS} Hz，JPEG 质量 {args.jpeg_quality}", flush=True)

    # 主线程只等待 Ctrl+C
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[dock] 收到中断信号，退出中...", flush=True)
    finally:
        provider.release()


if __name__ == "__main__":
    main()
