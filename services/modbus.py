#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
modbus.py — Modbus TCP 读写组件

职责：
  - 加载 datas/modbus.json 配置
  - 后台线程按 rate 周期读取 holding registers，发布到 /humanoid/modbus/data
  - 接收 /humanoid/modbus/control 命令：
      read             手动触发一次读取并发布
      write            写入指定地址的 holding register
      add_device       增加新设备
      add_read_addrs   为设备增加读取地址范围
      add_write_addrs  为设备增加写入地址范围
      del_read_addr    删除设备的读取地址
      del_write_addr   删除设备的写入地址

消息格式（/humanoid/modbus/data，发布）：
  {"command": "modbus_data", "devices": [
      {"ip": "10.20.15.120", "port": 10502,
       "read":  [{"address": 0, "value": 123}, ...],
       "write": [{"address": 10, "value": 0}, ...]}
  ]}
"""

import os
import json
import time
import socket
import struct
import threading

import common

# ── 配置 ───────────────────────────────────────────────────
_CONFIG_PATH = os.path.join(common.DATAS_DIR, "modbus.json")
_devices = []          # 解析后的设备配置列表
_thread_running = False
_thread = None
_lock = threading.Lock()


# ═══════════════════════════════════════════════════════════
#  Modbus TCP 纯 socket 实现
# ═══════════════════════════════════════════════════════════

class ModbusTcpClient:
    """轻量级 Modbus TCP 客户端（纯 socket，无第三方依赖）"""

    def __init__(self, ip, port=502, unit_id=1, timeout=1.0):
        self.ip = ip
        self.port = port
        self.unit_id = unit_id
        self.timeout = timeout
        self._tx_id = 0

    def _next_tx_id(self):
        self._tx_id = (self._tx_id + 1) & 0xFFFF
        return self._tx_id

    def _send_recv(self, pdu):
        """构建 MBAP 报文并发送/接收，返回 PDU 响应"""
        tx_id = self._next_tx_id()
        mbap = struct.pack(">HHHB", tx_id, 0x0000, len(pdu) + 1, self.unit_id)
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.timeout)
            sock.connect((self.ip, self.port))
            sock.sendall(mbap + pdu)
            header = self._recv_exact(sock, 7)
            if not header:
                return None
            _, _, length, _ = struct.unpack(">HHHB", header)
            pdu_len = length - 1
            pdu_resp = self._recv_exact(sock, pdu_len)
            return pdu_resp
        except Exception:
            return None
        finally:
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass

    @staticmethod
    def _recv_exact(sock, n):
        buf = b""
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def read_holding_registers(self, start_addr, count):
        pdu = struct.pack(">BHH", 0x03, start_addr, count)
        resp = self._send_recv(pdu)
        if not resp or len(resp) < 2:
            return None
        func_code = resp[0]
        if func_code & 0x80:
            return None
        byte_count = resp[1]
        values = []
        for i in range(0, byte_count, 2):
            val = struct.unpack(">H", resp[2 + i:4 + i])[0]
            values.append(val)
        return values

    def write_holding_register(self, addr, value):
        pdu = struct.pack(">BHH", 0x06, addr, value & 0xFFFF)
        resp = self._send_recv(pdu)
        if not resp or len(resp) < 5:
            return False
        func_code = resp[0]
        if func_code & 0x80:
            return False
        return True


def _group_continuous(addresses):
    """将地址列表分组为连续段 [(start, count), ...]"""
    if not addresses:
        return []
    addrs = sorted(set(addresses))
    groups = []
    start = addrs[0]
    prev = addrs[0]
    for a in addrs[1:]:
        if a == prev + 1:
            prev = a
        else:
            groups.append((start, prev - start + 1))
            start = a
            prev = a
    groups.append((start, prev - start + 1))
    return groups


def _save_config():
    """将当前 _devices 写回 modbus.json"""
    try:
        raw = []
        for dev in _devices:
            raw.append({
                "ip": dev["ip"],
                "port": dev["port"],
                "read": {"holdings": sorted(dev["read_holdings"])},
                "write": {"holdings": sorted(dev["write_holdings"])},
                "rate": dev["rate"]
            })
        with open(_CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(raw, f, ensure_ascii=False, indent=2)
        print(f"[Modbus] 配置已保存到 {_CONFIG_PATH}")
    except Exception as e:
        print(f"[Modbus] 保存配置失败: {e}")


def _publish_device(dev):
    """立即读取并发布单个设备的数据"""
    try:
        read_result, write_result = _read_device(dev)
        common.publish(common.TOPIC_MODBUS_DATA, {
            "command": "modbus_data",
            "devices": [{
                "ip": dev["ip"],
                "port": dev["port"],
                "read": read_result,
                "write": write_result,
            }]
        })
    except Exception as e:
        print(f"[Modbus] 读取 {dev['ip']} 失败: {e}")


# ═══════════════════════════════════════════════════════════
#  配置加载
# ═══════════════════════════════════════════════════════════

def load_config():
    """加载 modbus.json 配置"""
    global _devices
    _devices = []
    if not os.path.exists(_CONFIG_PATH):
        print("[Modbus] 配置文件不存在:", _CONFIG_PATH)
        return
    try:
        with open(_CONFIG_PATH, "r", encoding="utf-8") as f:
            raw = json.load(f)
        for dev in raw:
            _devices.append({
                "ip": dev.get("ip", "127.0.0.1"),
                "port": dev.get("port", 502),
                "read_holdings": dev.get("read", {}).get("holdings", []),
                "write_holdings": dev.get("write", {}).get("holdings", []),
                "rate": dev.get("rate", 1000),
            })
        print(f"[Modbus] 已加载 {len(_devices)} 个设备配置")
    except Exception as e:
        print(f"[Modbus] 加载配置失败: {e}")


# ═══════════════════════════════════════════════════════════
#  数据读取与发布
# ═══════════════════════════════════════════════════════════

def _read_device(dev):
    """读取单个设备的所有 read/write holdings，返回结果字典"""
    client = ModbusTcpClient(dev["ip"], dev["port"])
    read_result = []
    write_result = []

    for start, count in _group_continuous(dev["read_holdings"]):
        values = client.read_holding_registers(start, count)
        if values is not None:
            for i, v in enumerate(values):
                read_result.append({"address": start + i, "value": v})
        else:
            for a in range(start, start + count):
                read_result.append({"address": a, "value": None})

    for start, count in _group_continuous(dev["write_holdings"]):
        values = client.read_holding_registers(start, count)
        if values is not None:
            for i, v in enumerate(values):
                write_result.append({"address": start + i, "value": v})
        else:
            for a in range(start, start + count):
                write_result.append({"address": a, "value": None})

    read_result.sort(key=lambda x: x["address"])
    write_result.sort(key=lambda x: x["address"])
    return read_result, write_result


def read_and_publish():
    """读取所有设备数据并发布到 MQTT"""
    with _lock:
        devices_data = []
        for dev in _devices:
            read_result, write_result = _read_device(dev)
            devices_data.append({
                "ip": dev["ip"],
                "port": dev["port"],
                "read": read_result,
                "write": write_result,
            })
        common.publish(common.TOPIC_MODBUS_DATA, {
            "command": "modbus_data",
            "devices": devices_data
        })


# ═══════════════════════════════════════════════════════════
#  后台轮询线程
# ═══════════════════════════════════════════════════════════

def _polling_loop():
    """按各设备 rate 周期读取并发布数据"""
    global _thread_running
    print("[Modbus] 轮询线程已启动")
    next_times = {}
    while _thread_running:
        now = time.time()
        any_due = False
        with _lock:
            devs_snapshot = list(_devices)
        for dev in devs_snapshot:
            key = (dev["ip"], dev["port"])
            if key not in next_times:
                next_times[key] = 0.0
            if now >= next_times[key]:
                any_due = True
                try:
                    _publish_device(dev)
                except Exception as e:
                    print(f"[Modbus] 读取 {dev['ip']} 失败: {e}")
                next_times[key] = now + dev["rate"] / 1000.0
        if not any_due:
            time.sleep(0.01)
        else:
            time.sleep(0.005)
    print("[Modbus] 轮询线程已停止")


def start_polling_thread():
    """启动后台轮询线程"""
    global _thread_running, _thread
    load_config()
    if not _devices:
        print("[Modbus] 无设备配置，不启动轮询")
        return
    _thread_running = True
    _thread = threading.Thread(target=_polling_loop, daemon=True)
    _thread.start()


def stop_polling_thread():
    """停止后台轮询线程"""
    global _thread_running
    _thread_running = False
    if _thread:
        _thread.join(timeout=2)


# ═══════════════════════════════════════════════════════════
#  命令处理
# ═══════════════════════════════════════════════════════════

def handle_control(payload):
    """处理 /humanoid/modbus/control 命令"""
    cmd = payload.get("command")
    print(f"\n[命令] modbus/control: {cmd}")

    if cmd == "read":
        read_and_publish()

    elif cmd == "write":
        data = payload.get("data", {})
        ip = data.get("ip")
        address = data.get("address")
        value = data.get("value")
        if ip is None or address is None or value is None:
            print("[Modbus] write 命令缺少参数")
            return
        with _lock:
            dev = None
            for d in _devices:
                if d["ip"] == ip:
                    dev = d
                    break
        if dev is None:
            print(f"[Modbus] 未找到设备: {ip}")
            return
        if address not in dev["write_holdings"]:
            print(f"[Modbus] 地址 {address} 不在可写列表中")
            return
        client = ModbusTcpClient(dev["ip"], dev["port"])
        ok = client.write_holding_register(address, value)
        if ok:
            print(f"[Modbus] 写入成功 {ip}:{address} = {value}")
            _publish_device(dev)
        else:
            print(f"[Modbus] 写入失败 {ip}:{address}")

    elif cmd == "add_device":
        data = payload.get("data", {})
        ip = (data.get("ip") or "").strip()
        port = int(data.get("port", 502))
        rate = int(data.get("rate", 50))
        if not ip:
            print("[Modbus] add_device 缺少 ip")
            return
        with _lock:
            for d in _devices:
                if d["ip"] == ip and d["port"] == port:
                    print(f"[Modbus] 设备已存在: {ip}:{port}")
                    return
            _devices.append({
                "ip": ip,
                "port": port,
                "read_holdings": [],
                "write_holdings": [],
                "rate": rate
            })
        _save_config()
        print(f"[Modbus] 已添加设备: {ip}:{port}")
        # 立即读取并发布新设备
        with _lock:
            for d in _devices:
                if d["ip"] == ip and d["port"] == port:
                    _publish_device(d)
                    break

    elif cmd == "add_read_addrs":
        data = payload.get("data", {})
        ip = data.get("ip")
        start = int(data.get("start", 0))
        end = int(data.get("end", 0))
        if ip is None or start < 0 or end < start:
            print("[Modbus] add_read_addrs 参数错误")
            return
        with _lock:
            dev = None
            for d in _devices:
                if d["ip"] == ip:
                    dev = d
                    break
            if dev is None:
                print(f"[Modbus] 未找到设备: {ip}")
                return
            added = 0
            for a in range(start, end + 1):
                if a not in dev["read_holdings"]:
                    dev["read_holdings"].append(a)
                    added += 1
        _save_config()
        print(f"[Modbus] 设备 {ip} 读取区增加 {added} 个地址 [{start}-{end}]")
        _publish_device(dev)

    elif cmd == "add_write_addrs":
        data = payload.get("data", {})
        ip = data.get("ip")
        start = int(data.get("start", 0))
        end = int(data.get("end", 0))
        if ip is None or start < 0 or end < start:
            print("[Modbus] add_write_addrs 参数错误")
            return
        with _lock:
            dev = None
            for d in _devices:
                if d["ip"] == ip:
                    dev = d
                    break
            if dev is None:
                print(f"[Modbus] 未找到设备: {ip}")
                return
            added = 0
            for a in range(start, end + 1):
                if a not in dev["write_holdings"]:
                    dev["write_holdings"].append(a)
                    added += 1
        _save_config()
        print(f"[Modbus] 设备 {ip} 写入区增加 {added} 个地址 [{start}-{end}]")
        _publish_device(dev)

    elif cmd == "del_read_addr":
        data = payload.get("data", {})
        ip = data.get("ip")
        address = int(data.get("address", -1))
        if ip is None or address < 0:
            print("[Modbus] del_read_addr 参数错误")
            return
        with _lock:
            dev = None
            for d in _devices:
                if d["ip"] == ip:
                    dev = d
                    break
            if dev is None:
                print(f"[Modbus] 未找到设备: {ip}")
                return
            if address in dev["read_holdings"]:
                dev["read_holdings"].remove(address)
                print(f"[Modbus] 设备 {ip} 删除读取地址 {address}")
            else:
                print(f"[Modbus] 地址 {address} 不在读取列表中")
                return
        _save_config()
        _publish_device(dev)

    elif cmd == "del_write_addr":
        data = payload.get("data", {})
        ip = data.get("ip")
        address = int(data.get("address", -1))
        if ip is None or address < 0:
            print("[Modbus] del_write_addr 参数错误")
            return
        with _lock:
            dev = None
            for d in _devices:
                if d["ip"] == ip:
                    dev = d
                    break
            if dev is None:
                print(f"[Modbus] 未找到设备: {ip}")
                return
            if address in dev["write_holdings"]:
                dev["write_holdings"].remove(address)
                print(f"[Modbus] 设备 {ip} 删除写入地址 {address}")
            else:
                print(f"[Modbus] 地址 {address} 不在写入列表中")
                return
        _save_config()
        _publish_device(dev)

    else:
        print(f"[Modbus] 未知命令: {cmd}")
