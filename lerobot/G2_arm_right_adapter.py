#!/usr/bin/env python3
"""lerobot 对接程序：将 G2 机器人真实状态通过 TCP 暴露给 lerobot 端

线程模型（5 个线程）：
  1. read 线程（100ms 周期）：
       依次读取右臂关节角 + 末端夹爪、头部相机、腕部相机，
       存储到全局变量 g_arm_state / g_head_jpg / g_wrist_jpg。
  2. 9002 TCP 线程（arm get+set）：
       get -> 返回 g_arm_state（[j1..j7, gripper]）
       set -> 写入 pending_joints/pending_gripper；
              若 set_running==0，则置 set_to_run=1；回复 ok
  3. 9003 TCP 线程（head cam）：get -> 返回 g_head_jpg
  4. 9004 TCP 线程（wrist cam）：get -> 返回 g_wrist_jpg
  5. set_joint 线程：
       若 set_to_run==1：置 set_running=1、set_to_run=0，
       执行右臂 7 关节 + 末端夹爪动作，结束后 set_running=0

端口协议：
  端口 9002 (arm)：
      `get\n`            -> `[j1..j7, gripper]\n`
      `set [..8个数..]\n` -> `ok\n`
  端口 9003 (head rgb)：  `get\n` -> `<4 字节大端长度><JPEG>`
  端口 9004 (wrist rgb)： `get\n` -> `<4 字节大端长度><JPEG>`

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
RIGHT_ARM_JOINT_NAMES: List[str] = [
    "idx61_arm_r_joint1",
    "idx62_arm_r_joint2",
    "idx63_arm_r_joint3",
    "idx64_arm_r_joint4",
    "idx65_arm_r_joint5",
    "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]

GDK_INIT_WAIT_S = 2.0
GDK_QUERY_TIMEOUT_MS = 1000.0
READ_PERIOD_S = 0.1  # read 线程周期 100ms
ERROR_LOG_INTERVAL_S = 5.0


# ---------------------------------------------------------------------------
# 全局状态（read 线程写，TCP 线程读，不加锁以追求最低延迟）
# ---------------------------------------------------------------------------
g_arm_state: List[float] = [0.0] * 8  # [j1..j7, gripper]
g_head_jpg: Optional[bytes] = None
g_wrist_jpg: Optional[bytes] = None

# set 相关全局（9002 TCP 线程写 pending + 触发；set_joint 线程读 + 执行）
g_set_lock = threading.Lock()
g_set_to_run: int = 0      # 1 = 有待执行的动作请求
g_set_running: int = 0     # 1 = 动作正在执行中
g_pending_joints: List[float] = [0.0] * 7
g_pending_gripper: float = 0.0


# ---------------------------------------------------------------------------
# G2 GDK 封装：初始化 / 取图编码 / 下发控制
# ---------------------------------------------------------------------------
class G2Dock:
    """封装 GDK Robot + Camera 的初始化、图像编码、动作下发"""

    def __init__(self, jpeg_quality: int = 80, enable_control: bool = False):
        self.robot: Optional[agibot_gdk.Robot] = None
        self.camera: Optional[agibot_gdk.Camera] = None
        self.jpeg_quality = jpeg_quality
        self.enable_control = enable_control
        self._jpeg_encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)]
        self._stop_event = threading.Event()
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

        self._warmup_cameras()

    def _warmup_cameras(self) -> None:
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
                print(f"[dock] warmup {name} 异常: {type(e).__name__}: {e}", flush=True)

    def release(self) -> None:
        self._stop_event.set()
        if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
            print("[dock] GDK 释放失败", flush=True)
        else:
            print("[dock] GDK 释放成功", flush=True)

    # ---------- 日志限频 ----------
    def _log_error(self, tag: str, msg: str) -> None:
        now = time.time()
        with self._err_log_lock:
            last = self._last_err_log.get(tag, 0.0)
            if now - last < ERROR_LOG_INTERVAL_S:
                return
            self._last_err_log[tag] = now
        print(f"[dock:{tag}] {msg}", flush=True)

    # ---------- 关节 + 相机读取 ----------
    def read_arm_state(self) -> List[float]:
        """读取右臂 7 关节 + 末端夹爪，返回 8 维 list"""
        joint_states = self.robot.get_joint_states()
        name_to_pos = {
            s["name"]: float(s["position"])
            for s in joint_states["states"]
        }
        joints: List[float] = []
        for name in RIGHT_ARM_JOINT_NAMES:
            if name not in name_to_pos:
                self._log_error("arm", f"关节状态中找不到 {name}")
                return g_arm_state  # 返回旧值
            joints.append(name_to_pos[name])

        end_state = self.robot.get_end_state()
        right_end = end_state["right_end_state"]
        end_states_list = right_end.get("end_states") or []
        gripper_pos = float(end_states_list[0].get("position", 0.0)) if end_states_list else 0.0
        return joints + [gripper_pos]

    def fetch_jpeg(self, camera_type, tag: str) -> Optional[bytes]:
        """从 GDK 取一帧图像并编码为 JPEG 字节"""
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

        if image.encoding == agibot_gdk.Encoding.JPEG:
            return bytes(image.data)
        if image.encoding == agibot_gdk.Encoding.PNG:
            nparr = np.frombuffer(image.data, np.uint8)
            decoded = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            ok, jpg = cv2.imencode(".jpg", decoded, self._jpeg_encode_param)
            return jpg.tobytes() if ok else None

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

    # ---------- 右臂控制（set_joint 线程调用） ----------
    def set_right_arm_state(self, joints: List[float], gripper: float) -> None:
        """下发右臂 7 关节 + 末端夹爪位置到 G2 机器人"""
        if not self.enable_control:
            return
        if self.robot is None:
            raise RuntimeError("Robot not initialized")
        if len(joints) != 7:
            raise ValueError(f"joints must be 7-dim, got {len(joints)}")

        gripper_clamped = max(-0.7, min(0.0, float(gripper)))

        # 1. 下发 7 个右臂关节位置
        try:
            req = agibot_gdk.JointControlReq()
            req.life_time = 1.0
            req.joint_names = list(RIGHT_ARM_JOINT_NAMES)
            req.joint_positions = [float(v) for v in joints]
            req.joint_velocities = [0.3] * 7
            self.robot.joint_control_request(req)
            time.sleep(0.02)
        except Exception as e:
            raise RuntimeError(
                f"joint_control_request failed: {type(e).__name__}: {e}"
            ) from e

        # 2. 下发末端夹爪位置（omnipicker）
        joint_states = agibot_gdk.JointStates()
        joint_states.group = "right_tool"
        joint_states.target_type = "omnipicker"
        joint_state = agibot_gdk.JointState()
        joint_state.position = gripper_clamped
        joint_states.states = [joint_state]
        joint_states.nums = 1
        try:
            self.robot.move_ee_pos(joint_states)
        except Exception as e:
            print(f"[dock:set] warn: move_ee_pos failed: "
                  f"{type(e).__name__}: {e}", flush=True)


# ---------------------------------------------------------------------------
# 线程 1：read（100ms 周期采集关节 + 两个相机 → 全局变量）
# ---------------------------------------------------------------------------
def read_loop(dock: G2Dock):
    global g_arm_state, g_head_jpg, g_wrist_jpg
    print(f"[dock:read] 启动，周期 {READ_PERIOD_S*1000:.0f}ms", flush=True)
    while not dock._stop_event.is_set():
        t0 = time.time()
        try:
            arm = dock.read_arm_state()
            head_jpg = dock.fetch_jpeg(agibot_gdk.CameraType.kHeadColor, "head")
            wrist_jpg = dock.fetch_jpeg(agibot_gdk.CameraType.kHandRightColor, "wrist")
            g_arm_state = arm
            if head_jpg is not None and len(head_jpg) > 0:
                g_head_jpg = head_jpg
            if wrist_jpg is not None and len(wrist_jpg) > 0:
                g_wrist_jpg = wrist_jpg
        except Exception as e:
            dock._log_error("read", f"采集异常: {type(e).__name__}: {e}")

        elapsed = time.time() - t0
        sleep_s = READ_PERIOD_S - elapsed
        if sleep_s > 0:
            time.sleep(sleep_s)


def get_snapshot_arm() -> List[float]:
    return g_arm_state


def get_snapshot_head() -> Optional[bytes]:
    return g_head_jpg


def get_snapshot_wrist() -> Optional[bytes]:
    return g_wrist_jpg


# ---------------------------------------------------------------------------
# 线程 2：9002 TCP（arm get + set）
# ---------------------------------------------------------------------------
def serve_9002(dock: G2Dock):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, ARM_PORT))
    srv.listen(8)
    print(f"[dock:9002] listening on {HOST}:{ARM_PORT} (get+set)", flush=True)
    try:
        while not dock._stop_event.is_set():
            try:
                conn, addr = srv.accept()
            except OSError:
                break
            print(f"[dock:9002] client connected from {addr}", flush=True)
            threading.Thread(
                target=_handle_9002_conn, args=(conn, addr, dock),
                daemon=True, name=f"dock-9002-{addr[0]}:{addr[1]}",
            ).start()
    finally:
        srv.close()


def _handle_9002_conn(conn: socket.socket, addr, dock: G2Dock):
    global g_pending_joints, g_pending_gripper, g_set_to_run, g_set_running
    try:
        with conn:
            buf = bytearray()
            while True:
                try:
                    data = conn.recv(1024)
                except (ConnectionError, OSError):
                    break
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
                            state = get_snapshot_arm()
                            reply = json.dumps([round(v, 4) for v in state]) + "\n"
                            conn.sendall(reply.encode("utf-8"))
                        elif request.startswith("set"):
                            payload = request[len("set"):].strip()
                            values = json.loads(payload)
                            if not isinstance(values, list) or len(values) != 8:
                                conn.sendall(
                                    f"error: expected 8-dim list, got "
                                    f"{len(values) if isinstance(values, list) else type(values).__name__}\n".encode("utf-8")
                                )
                                continue
                            joints = [float(v) for v in values[:7]]
                            gripper = float(values[7])
                            # 写入 pending，并在空闲时触发 set_joint 线程
                            with g_set_lock:
                                g_pending_joints = joints
                                g_pending_gripper = gripper
                                if g_set_running == 0:
                                    g_set_to_run = 1
                            conn.sendall(b"ok\n")
                        else:
                            conn.sendall(b"error\n")
                    except Exception as e:
                        print(f"[dock:9002] 处理请求 '{request}' 失败: "
                              f"{type(e).__name__}: {e}", flush=True)
                        try:
                            conn.sendall(f"error: {e}\n".encode("utf-8"))
                        except Exception:
                            pass
    finally:
        print(f"[dock:9002] client disconnected: {addr}", flush=True)


# ---------------------------------------------------------------------------
# 线程 3 / 4：9003 / 9004 TCP（相机 get）
# ---------------------------------------------------------------------------
def serve_camera(port: int, getter, tag: str, dock: G2Dock):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, port))
    srv.listen(8)
    print(f"[dock:{tag}] listening on {HOST}:{port}", flush=True)
    try:
        while not dock._stop_event.is_set():
            try:
                conn, addr = srv.accept()
            except OSError:
                break
            print(f"[dock:{tag}] client connected from {addr}", flush=True)
            threading.Thread(
                target=_handle_camera_conn,
                args=(conn, addr, getter, tag, dock),
                daemon=True, name=f"dock-{tag}-{addr[0]}:{addr[1]}",
            ).start()
    finally:
        srv.close()


def _handle_camera_conn(conn: socket.socket, addr, getter, tag: str, dock: G2Dock):
    try:
        with conn:
            while True:
                try:
                    data = conn.recv(1024)
                except (ConnectionError, OSError):
                    break
                if not data:
                    break
                if b"get" in data:
                    jpg = getter()
                    if jpg is None or len(jpg) == 0:
                        conn.sendall(struct.pack(">I", 0))
                    else:
                        conn.sendall(struct.pack(">I", len(jpg)) + jpg)
    except ConnectionError:
        pass
    finally:
        print(f"[dock:{tag}] client disconnected: {addr}", flush=True)


# ---------------------------------------------------------------------------
# 线程 5：set_joint（监视 set_to_run，执行关节 + 夹爪动作）
# ---------------------------------------------------------------------------
def set_joint_loop(dock: G2Dock):
    global g_set_to_run, g_set_running
    print("[dock:set_joint] 启动，等待 set 命令", flush=True)
    while not dock._stop_event.is_set():
        run = False
        joints: List[float] = [0.0] * 7
        gripper: float = 0.0
        with g_set_lock:
            if g_set_to_run == 1:
                g_set_running = 1
                g_set_to_run = 0
                joints = list(g_pending_joints)
                gripper = g_pending_gripper
                run = True
        if run:
            try:
                dock.set_right_arm_state(joints, gripper)
            except Exception as e:
                print(f"[dock:set_joint] 动作执行失败: "
                      f"{type(e).__name__}: {e}", flush=True)
            finally:
                with g_set_lock:
                    g_set_running = 0
        else:
            time.sleep(0.001)


# ---------------------------------------------------------------------------
# 主入口
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="lerobot 对接程序：G2 真机 → TCP")
    parser.add_argument("--jpeg-quality", type=int, default=80,
                        help="JPEG 编码质量 (1-100，默认 80)")
    parser.add_argument("--enable-control", action="store_true", default=False,
                        help="启用控制：收到 set 命令时真正下发到 G2 机器人 "
                             "(默认关闭，仅 ACK；推理时必须加此参数机器人才能动)")
    args = parser.parse_args()
    if not (1 <= args.jpeg_quality <= 100):
        parser.error("--jpeg-quality 必须在 1..100 范围")

    dock = G2Dock(
        jpeg_quality=args.jpeg_quality,
        enable_control=args.enable_control,
    )
    try:
        dock.initialize()
    except Exception as e:
        print(f"[dock] 初始化失败: {e}", flush=True)
        return

    if args.enable_control:
        print("=" * 60, flush=True)
        print("[dock] 控制已启用 (CONTROL ENABLED)", flush=True)
        print("[dock] 收到 set 命令会真正下发到 G2 机器人", flush=True)
        print("[dock] 请确保机器人周围无人和障碍物", flush=True)
        print("=" * 60, flush=True)
    else:
        print("[dock] 控制未启用 (set 命令仅 ACK，不会动)", flush=True)
        print("[dock] 如需推理，请加 --enable-control 参数", flush=True)

    threads: List[threading.Thread] = [
        threading.Thread(target=read_loop, args=(dock,),
                         daemon=True, name="dock-read"),
        threading.Thread(target=serve_9002, args=(dock,),
                         daemon=True, name="dock-9002"),
        threading.Thread(target=serve_camera,
                         args=(HEAD_CAM_PORT, get_snapshot_head, "9003", dock),
                         daemon=True, name="dock-9003"),
        threading.Thread(target=serve_camera,
                         args=(WRIST_CAM_PORT, get_snapshot_wrist, "9004", dock),
                         daemon=True, name="dock-9004"),
        threading.Thread(target=set_joint_loop, args=(dock,),
                         daemon=True, name="dock-set_joint"),
    ]
    for t in threads:
        t.start()

    print(f"[dock] 全部服务就绪：arm(get+set)={ARM_PORT}  "
          f"head={HEAD_CAM_PORT}  wrist={WRIST_CAM_PORT}", flush=True)
    print(f"[dock] read 周期 {READ_PERIOD_S*1000:.0f}ms，JPEG 质量 {args.jpeg_quality}",
          flush=True)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[dock] 收到中断信号，退出中...", flush=True)
    finally:
        dock.release()


if __name__ == "__main__":
    main()
