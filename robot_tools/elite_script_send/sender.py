#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Elite 30001 脚本下发 + 29999 Dashboard（上电/下电/松闸/play/查任务）。"""
from __future__ import print_function

import os
import socket
import time
from pathlib import Path

SCRIPT_SUFFIXES = {".script", ".prg", ".txt"}
DEFAULT_HOST = os.environ.get("SCRIPT_SEND_HOST", "192.168.249.128")
DEFAULT_PORT = int(os.environ.get("SCRIPT_SEND_PORT", "30001"))
DEFAULT_DASHBOARD_PORT = int(os.environ.get("DASHBOARD_PORT", "29999"))
DEFAULT_TIMEOUT = float(os.environ.get("SCRIPT_SEND_TIMEOUT", "5.0"))
MODULE_DIR = Path(__file__).resolve().parent
DEFAULT_SCRIPT_DIR = Path(os.environ.get("SCRIPT_OUTPUT_DIR", str(MODULE_DIR / "scripts")))


class SendError(Exception):
    def __init__(self, message, code=1):
        self.message = message
        self.code = code
        Exception.__init__(self, message)


def list_script_files(directory=None):
    d = Path(directory) if directory else DEFAULT_SCRIPT_DIR
    if not d.exists():
        return []
    return sorted(
        p for p in d.iterdir() if p.is_file() and p.suffix.lower() in SCRIPT_SUFFIXES
    )


def send_text(script_text, host=None, port=None, timeout=None, read_reply=False):
    host = host or DEFAULT_HOST
    port = int(port or DEFAULT_PORT)
    timeout = float(timeout if timeout is not None else DEFAULT_TIMEOUT)
    text = script_text if script_text.endswith("\n") else script_text + "\n"
    content = text.encode("utf-8")
    if not content:
        raise SendError("脚本内容为空", 4005)
    start = time.time()
    reply = ""
    try:
        with socket.create_connection((host, port), timeout=timeout) as sock:
            sock.sendall(content)
            if read_reply:
                sock.settimeout(timeout)
                try:
                    reply = sock.recv(8192).decode("utf-8", "ignore")
                except socket.timeout:
                    reply = ""
    except (ConnectionRefusedError, ConnectionResetError, socket.timeout, TimeoutError, OSError) as exc:
        raise SendError("脚本下发失败（%s:%s）: %s" % (host, port, exc), 5003)
    return {
        "host": host,
        "port": port,
        "bytes_sent": len(content),
        "duration_ms": int((time.time() - start) * 1000),
        "reply": reply,
    }


def dashboard_cmd(command, host=None, port=None, timeout=None):
    host = host or DEFAULT_HOST
    port = int(port or DEFAULT_DASHBOARD_PORT)
    timeout = float(timeout if timeout is not None else DEFAULT_TIMEOUT)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        sock.connect((host, port))
        sock.settimeout(min(0.4, timeout))
        try:
            sock.recv(4096)
        except socket.timeout:
            pass
        sock.settimeout(timeout)
        line = command if command.endswith("\n") else command + "\n"
        sock.sendall(line.encode("utf-8"))
        data = sock.recv(4096).decode("utf-8", "ignore")
        return data.replace("\n", "").replace("\r", "")
    except (ConnectionRefusedError, ConnectionResetError, socket.timeout, TimeoutError, OSError) as exc:
        raise SendError("Dashboard 失败（%s:%s）: %s" % (host, port, exc), 5003)
    finally:
        sock.close()


def powering_on(host=None, timeout=None):
    data = dashboard_cmd("robotControl -on", host=host, timeout=timeout)
    time.sleep(0.5)
    ok = data == "Powering on"
    return {"ok": ok, "reply": data, "message": "上电成功" if ok else data}


def powering_off(host=None, timeout=None, stop_event=None, interval=0.5, max_wait=60.0):
    started = time.time()
    last = ""
    while True:
        if stop_event is not None and stop_event.is_set():
            raise SendError("下电已取消")
        if max_wait and (time.time() - started) > max_wait:
            raise SendError("下电超时: %s" % last)
        last = dashboard_cmd("robotControl -off", host=host, timeout=timeout)
        if last == "Powering off":
            return {"ok": True, "reply": last, "message": "下电成功"}
        time.sleep(interval)


def brake_releasing(host=None, timeout=None, stop_event=None, interval=0.5, max_wait=60.0):
    started = time.time()
    last = ""
    while True:
        if stop_event is not None and stop_event.is_set():
            raise SendError("松闸已取消")
        if max_wait and (time.time() - started) > max_wait:
            raise SendError("松闸超时: %s" % last)
        last = dashboard_cmd("brakeRelease", host=host, timeout=timeout)
        if last == "Brake is released.":
            return {"ok": True, "reply": last, "message": "抱闸释放成功"}
        time.sleep(interval)


def play(host=None, timeout=None):
    data = dashboard_cmd("play", host=host, timeout=timeout)
    time.sleep(1)
    ok = data == "Starting task"
    return {"ok": ok, "reply": data, "message": "已启动任务" if ok else ("运行失败：" + data)}


def task(host=None, timeout=None):
    data = dashboard_cmd("task -r", host=host, timeout=timeout)
    running = data == "Task is running"
    msg = "任务正在运行" if running else ("当前运行状态为：" + data)
    return {"ok": True, "reply": data, "running": running, "message": msg}


def close_popup(host=None, timeout=None):
    """手册：popup -c，关闭最近由 popup 弹出的消息框，返回 Closing popup。"""
    data = dashboard_cmd("popup -c", host=host, timeout=timeout)
    ok = "closing popup" in data.lower()
    return {"ok": ok, "reply": data, "message": "已清除弹窗" if ok else data}


def boot_and_send(script_text, host=None, timeout=None, stop_event=None, read_reply=False):
    power = powering_on(host=host, timeout=timeout)
    if not power["ok"]:
        raise SendError("上电失败: %s" % power["reply"])
    brake = brake_releasing(host=host, timeout=timeout, stop_event=stop_event)
    send = send_text(script_text, host=host, timeout=timeout, read_reply=read_reply)
    send["message"] = "上电+松闸+下发完成  %s bytes" % send["bytes_sent"]
    send["power"] = power
    send["brake"] = brake
    return send
