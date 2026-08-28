#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
data.py — SQLite 数据库管理（内存运行 + 文件持久化）

职责：
  - 系统初始化时从硬盘 SQLite 文件加载到内存数据库
  - 运行时所有读写操作在内存中完成（快速）
  - 写操作（save/update/delete）同步持久化到硬盘文件
  - 提供关节角、末端位姿、地图点位的增删改查接口

数据库表设计：
  joints     — 关节角数据
      id, type, name, value(TEXT JSON)
  positions  — 末端位姿数据
      id, type, name, value(TEXT JSON)
  map_points — 地图点位
      id, name, source, position(TEXT JSON), orientation(TEXT JSON)
"""

import os
import json
import sqlite3
import threading

import common

# ── SQLite 数据库文件路径 ──────────────────────────────────
DB_PATH = os.path.join(common.DATAS_DIR, "robot_data.db")

# 内存数据库连接（运行时使用，快速）
_mem_conn = None
# 文件数据库连接（持久化使用）
_file_conn = None
_db_lock = threading.Lock()


# ═══════════════════════════════════════════════════════════
#  数据库初始化
# ═══════════════════════════════════════════════════════════

def init_db():
    """初始化数据库

    流程：
      1. 如果 DB_PATH 文件存在 → 从文件加载到内存
      2. 如果文件不存在 → 创建空数据库，建表
    """
    global _mem_conn, _file_conn

    os.makedirs(common.DATAS_DIR, exist_ok=True)

    # 内存连接（运行时读写）
    _mem_conn = sqlite3.connect(":memory:", check_same_thread=False)

    if os.path.exists(DB_PATH):
        # 从文件数据库加载到内存
        _file_conn = sqlite3.connect(DB_PATH, check_same_thread=False)
        _file_conn.backup(_mem_conn)
        print(f"[DB] 已从文件加载: {DB_PATH}")
    else:
        # 首次创建，建表并持久化空库
        _create_tables(_mem_conn)
        _file_conn = sqlite3.connect(DB_PATH, check_same_thread=False)
        _mem_conn.backup(_file_conn)
        print(f"[DB] 已创建新数据库文件: {DB_PATH}")

    # 确保表存在
    _create_tables(_mem_conn)

    # 统计
    with _db_lock:
        cur = _mem_conn.cursor()
        cur.execute("SELECT COUNT(*) FROM joints")
        j_count = cur.fetchone()[0]
        cur.execute("SELECT COUNT(*) FROM positions")
        p_count = cur.fetchone()[0]
        cur.execute("SELECT COUNT(*) FROM map_points")
        m_count = cur.fetchone()[0]
    print(f"[DB] 初始化完成: joints={j_count}, positions={p_count}, map_points={m_count}")


def _create_tables(conn):
    """在指定连接上创建表"""
    cur = conn.cursor()
    cur.execute("""
        CREATE TABLE IF NOT EXISTS joints (
            id    INTEGER PRIMARY KEY AUTOINCREMENT,
            type  TEXT NOT NULL,
            name  TEXT NOT NULL,
            value TEXT NOT NULL,
            UNIQUE(type, name)
        )
    """)
    cur.execute("""
        CREATE TABLE IF NOT EXISTS positions (
            id    INTEGER PRIMARY KEY AUTOINCREMENT,
            type  TEXT NOT NULL,
            name  TEXT NOT NULL,
            value TEXT NOT NULL,
            UNIQUE(type, name)
        )
    """)
    cur.execute("""
        CREATE TABLE IF NOT EXISTS map_points (
            id           INTEGER PRIMARY KEY AUTOINCREMENT,
            name         TEXT NOT NULL,
            source       TEXT NOT NULL DEFAULT 'local',
            position     TEXT NOT NULL DEFAULT '[]',
            orientation  TEXT NOT NULL DEFAULT '[0,0,0,1]',
            UNIQUE(name, source)
        )
    """)
    conn.commit()


def _sync_to_file():
    """将内存数据库同步到文件（写操作后调用）"""
    _mem_conn.backup(_file_conn)


# ═══════════════════════════════════════════════════════════
#  关节数据 CRUD
# ═══════════════════════════════════════════════════════════

def get_joints(jtype=None, name=None):
    """查询关节数据"""
    with _db_lock:
        cur = _mem_conn.cursor()
        if jtype and name:
            cur.execute("SELECT type, name, value FROM joints WHERE type=? AND name=?", (jtype, name))
        elif jtype:
            cur.execute("SELECT type, name, value FROM joints WHERE type=? ORDER BY name", (jtype,))
        else:
            cur.execute("SELECT type, name, value FROM joints ORDER BY type, name")
        rows = cur.fetchall()
    return [{"type": r[0], "name": r[1], "value": json.loads(r[2])} for r in rows]


def save_joints(jtype, name, value):
    """保存或更新关节数据"""
    with _db_lock:
        cur = _mem_conn.cursor()
        cur.execute(
            "INSERT INTO joints (type, name, value) VALUES (?, ?, ?) "
            "ON CONFLICT(type, name) DO UPDATE SET value=excluded.value",
            (jtype, name, json.dumps(value, ensure_ascii=False))
        )
        _mem_conn.commit()
        _sync_to_file()
    print(f"  [DB] 保存关节: {jtype}/{name}")


def update_joints(jtype, name, value):
    """更新关节数据"""
    save_joints(jtype, name, value)


def delete_joints(jtype, name):
    """删除关节数据"""
    with _db_lock:
        cur = _mem_conn.cursor()
        cur.execute("DELETE FROM joints WHERE type=? AND name=?", (jtype, name))
        _mem_conn.commit()
        _sync_to_file()
    print(f"  [DB] 删除关节: {jtype}/{name}")


# ═══════════════════════════════════════════════════════════
#  位姿数据 CRUD
# ═══════════════════════════════════════════════════════════

def get_positions(ptype=None, name=None):
    """查询位姿数据"""
    with _db_lock:
        cur = _mem_conn.cursor()
        if ptype and name:
            cur.execute("SELECT type, name, value FROM positions WHERE type=? AND name=?", (ptype, name))
        elif ptype:
            cur.execute("SELECT type, name, value FROM positions WHERE type=? ORDER BY name", (ptype,))
        else:
            cur.execute("SELECT type, name, value FROM positions ORDER BY type, name")
        rows = cur.fetchall()
    return [{"type": r[0], "name": r[1], "value": json.loads(r[2])} for r in rows]


def save_positions(ptype, name, value):
    """保存或更新位姿数据"""
    with _db_lock:
        cur = _mem_conn.cursor()
        cur.execute(
            "INSERT INTO positions (type, name, value) VALUES (?, ?, ?) "
            "ON CONFLICT(type, name) DO UPDATE SET value=excluded.value",
            (ptype, name, json.dumps(value, ensure_ascii=False))
        )
        _mem_conn.commit()
        _sync_to_file()
    print(f"  [DB] 保存位姿: {ptype}/{name}")


def update_positions(ptype, name, value):
    """更新位姿数据"""
    save_positions(ptype, name, value)


def delete_positions(ptype, name):
    """删除位姿数据"""
    with _db_lock:
        cur = _mem_conn.cursor()
        cur.execute("DELETE FROM positions WHERE type=? AND name=?", (ptype, name))
        _mem_conn.commit()
        _sync_to_file()
    print(f"  [DB] 删除位姿: {ptype}/{name}")


# ═══════════════════════════════════════════════════════════
#  地图点位 CRUD
# ═══════════════════════════════════════════════════════════

def get_map_points(source=None):
    """查询地图点位"""
    with _db_lock:
        cur = _mem_conn.cursor()
        if source:
            cur.execute(
                "SELECT name, source, position, orientation FROM map_points WHERE source=? ORDER BY name",
                (source,)
            )
        else:
            cur.execute(
                "SELECT name, source, position, orientation FROM map_points ORDER BY source, name"
            )
        rows = cur.fetchall()
    return [{
        "name": r[0],
        "source": r[1],
        "position": json.loads(r[2]),
        "orientation": json.loads(r[3]),
    } for r in rows]


def save_map_point(name, position, orientation, source="local"):
    """保存或更新地图点位"""
    with _db_lock:
        cur = _mem_conn.cursor()
        cur.execute(
            "INSERT INTO map_points (name, source, position, orientation) VALUES (?, ?, ?, ?) "
            "ON CONFLICT(name, source) DO UPDATE SET position=excluded.position, orientation=excluded.orientation",
            (name, source,
             json.dumps(position, ensure_ascii=False),
             json.dumps(orientation, ensure_ascii=False))
        )
        _mem_conn.commit()
        _sync_to_file()
    print(f"  [DB] 保存地图点位: {name} ({source})")


def delete_map_point(name, source="local"):
    """删除地图点位"""
    with _db_lock:
        cur = _mem_conn.cursor()
        cur.execute("DELETE FROM map_points WHERE name=? AND source=?", (name, source))
        _mem_conn.commit()
        _sync_to_file()
    print(f"  [DB] 删除地图点位: {name} ({source})")


# ═══════════════════════════════════════════════════════════
#  统一查询/保存/更新/删除接口
# ═══════════════════════════════════════════════════════════

def get_all_data():
    """查询所有关节数据和位姿数据，返回统一格式列表"""
    items = []
    for item in get_joints():
        items.append({
            "category": "joints",
            "type": item["type"],
            "name": item["name"],
            "value": item["value"]
        })
    for item in get_positions():
        items.append({
            "category": "positions",
            "type": item["type"],
            "name": item["name"],
            "value": item["value"]
        })
    return items


def save_data(category, dtype, name, value):
    """统一保存接口"""
    if category == "joints":
        save_joints(dtype, name, value)
    elif category == "positions":
        save_positions(dtype, name, value)
    else:
        print(f"  [DB] 未知数据类别: {category}")


def update_data(category, dtype, name, value):
    """统一更新接口"""
    if category == "joints":
        update_joints(dtype, name, value)
    elif category == "positions":
        update_positions(dtype, name, value)
    else:
        print(f"  [DB] 未知数据类别: {category}")


def delete_data(category, dtype, name):
    """统一删除接口"""
    if category == "joints":
        delete_joints(dtype, name)
    elif category == "positions":
        delete_positions(dtype, name)
    else:
        print(f"  [DB] 未知数据类别: {category}")
