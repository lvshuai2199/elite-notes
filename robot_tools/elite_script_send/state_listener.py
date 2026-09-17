#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""解析 Elite 30001 机器人状态报文中的关节角与笛卡尔位姿。

依据 CS_用户手册_TCPIP通讯接口协议：
- 报文头 5 字节：uint32 总长 + uint8 类型，MESSAGE_TYPE_ROBOT_STATE = 16
- 子包头 5 字节：uint32 子包长 + uint8 类型
- JOINT_DATA = 1，每轴 57 字节，首字段 actual_joint（rad）
- CARTESIAN_INFO = 4，tcp_x/y/z、rot_x/y/z（m / rad）
字节序为大端。
"""
from __future__ import print_function

import math
import struct

MESSAGE_TYPE_ROBOT_STATE = 16
SUB_JOINT_DATA = 1
SUB_CARTESIAN_INFO = 4
JOINT_COUNT = 6
JOINT_STRIDE = 57
MAX_PACKET = 65536


def rad_to_deg(value):
    return value * 180.0 / math.pi


def parse_robot_state(packet):
    """解析一条完整状态包，返回 (joints_rad, cart_m_rad) 或 None。"""
    if packet is None or len(packet) < 5:
        return None
    size, msg_type = struct.unpack_from(">iB", packet)
    if msg_type != MESSAGE_TYPE_ROBOT_STATE or size < 5 or size > len(packet):
        return None
    joints = None
    cart = None
    offset = 5
    while offset + 5 <= size:
        sub_len, sub_type = struct.unpack_from(">iB", packet, offset)
        if sub_len < 5 or offset + sub_len > size:
            break
        payload = packet[offset + 5 : offset + sub_len]
        if sub_type == SUB_JOINT_DATA and len(payload) >= JOINT_STRIDE * JOINT_COUNT:
            joints = tuple(
                struct.unpack_from(">d", payload, i * JOINT_STRIDE)[0]
                for i in range(JOINT_COUNT)
            )
        elif sub_type == SUB_CARTESIAN_INFO and len(payload) >= 48:
            cart = struct.unpack_from(">6d", payload)
        offset += sub_len
    if joints is None and cart is None:
        return None
    return {"joints_rad": joints, "cart_m_rad": cart}


def format_pose(parsed):
    """转成界面显示用的度 / 毫米。"""
    out = {"joints_deg": None, "xyz_mm": None, "rpy_deg": None}
    if not parsed:
        return out
    joints = parsed.get("joints_rad")
    cart = parsed.get("cart_m_rad")
    if joints:
        out["joints_deg"] = tuple(rad_to_deg(v) for v in joints)
    if cart:
        out["xyz_mm"] = (cart[0] * 1000.0, cart[1] * 1000.0, cart[2] * 1000.0)
        out["rpy_deg"] = tuple(rad_to_deg(v) for v in cart[3:6])
    return out


class RobotStateStream(object):
    def __init__(self):
        self._buf = b""

    def reset(self):
        self._buf = b""

    def feed(self, data):
        """喂入字节，产出解析后的位姿字典。"""
        if not data:
            return []
        self._buf += data
        poses = []
        while len(self._buf) >= 5:
            size, msg_type = struct.unpack_from(">iB", self._buf)
            if size < 5 or size > MAX_PACKET:
                self._buf = self._buf[1:]
                continue
            if len(self._buf) < size:
                break
            packet = self._buf[:size]
            self._buf = self._buf[size:]
            if msg_type != MESSAGE_TYPE_ROBOT_STATE:
                continue
            parsed = parse_robot_state(packet)
            if parsed:
                poses.append(parsed)
        if len(self._buf) > MAX_PACKET * 2:
            self._buf = b""
        return poses
