#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Elite 脚本下发测试界面 / 命令行。"""
from __future__ import print_function
import argparse
import json
from pathlib import Path
import socket
import sys
import threading
import time
import traceback
import webbrowser

from sender import (
    DEFAULT_HOST,
    DEFAULT_PORT,
    DEFAULT_SCRIPT_DIR,
    DEFAULT_TIMEOUT,
    SendError,
    boot_and_send,
    brake_releasing,
    close_popup,
    dashboard_cmd,
    emergency_stop,
    list_script_files,
    play,
    powering_off,
    powering_on,
    send_text,
    stop_task,
    task,
)
from state_listener import RobotStateStream, format_pose
from trajectory import pose_sample, write_csv, write_html

try:
    import tkinter as tk
    from tkinter import filedialog, messagebox, scrolledtext, ttk
except ImportError:
    tk = None

SETTINGS_PATH = Path(__file__).resolve().parent / "last_settings.json"


def load_settings():
    try:
        data = json.loads(SETTINGS_PATH.read_text(encoding="utf-8"))
        if isinstance(data, dict):
            return data
    except (OSError, ValueError):
        pass
    return {}


def save_settings(data):
    try:
        SETTINGS_PATH.write_text(
            json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    except OSError:
        pass


SAMPLE = """def elite_script_send_probe():
    popup(s="elite_script_send ok", blocking=False)
end
elite_script_send_probe()
"""

TITLE_CMDS = {
    "上电": "robotControl -on",
    "下电": "robotControl -off",
    "松闸": "brakeRelease",
    "运行": "play",
    "停止工程": "stop",
    "查询任务": "task -r",
    "清除弹窗": "popup -c",
    "测试连接": "echo",
}


def _read_file(path):
    p = Path(path)
    try:
        return p.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        return p.read_text(encoding="gbk", errors="replace")


def _cli_text(args):
    if args.file:
        return _read_file(args.file)
    if args.stdin:
        return sys.stdin.read()
    return SAMPLE


def run_cli(args):
    try:
        if args.dashboard:
            print(dashboard_cmd(args.dashboard, host=args.host, timeout=args.timeout))
            return 0
        if args.power_on:
            print(powering_on(host=args.host, timeout=args.timeout)["message"])
        if args.power_off:
            print(powering_off(host=args.host, timeout=args.timeout)["message"])
        if args.close_popup:
            print(close_popup(host=args.host, timeout=args.timeout)["message"])
        if args.stop_task:
            print(stop_task(host=args.host, timeout=args.timeout)["message"])
        if args.estop:
            print(emergency_stop(host=args.host, timeout=args.timeout)["message"])
        if args.brake:
            print(brake_releasing(host=args.host, timeout=args.timeout)["message"])
        if args.boot_send:
            result = boot_and_send(
                _cli_text(args), host=args.host, timeout=args.timeout, read_reply=args.read_reply
            )
            print(result["message"])
            if result.get("reply"):
                print(result["reply"])
            return 0
        if args.file or args.stdin or not (
            args.power_on or args.power_off or args.close_popup or args.stop_task or args.estop or args.brake or args.play or args.task
        ):
            result = send_text(
                _cli_text(args),
                host=args.host,
                port=args.port,
                timeout=args.timeout,
                read_reply=args.read_reply,
            )
            print(
                "ok bytes=%s ms=%s %s:%s"
                % (result["bytes_sent"], result["duration_ms"], result["host"], result["port"])
            )
            if result.get("reply"):
                print(result["reply"])
        if args.play:
            print(play(host=args.host, timeout=args.timeout)["message"])
        if args.task:
            print(task(host=args.host, timeout=args.timeout)["message"])
        return 0
    except SendError as exc:
        print(exc.message, file=sys.stderr)
        return 1


class EliteScriptTester(object):
    def __init__(self, root):
        self.root = root
        self.root.title("Elite 脚本下发测试")
        self.root.minsize(720, 420)
        self.cfg = load_settings()
        self.root.geometry(self._fit_geometry(self.cfg.get("geometry")))
        saved_dir = self.cfg.get("script_dir")
        self.script_dir = Path(saved_dir) if saved_dir else DEFAULT_SCRIPT_DIR
        self.busy = False
        self.stop_event = threading.Event()
        self._save_job = None
        self._loading = True
        self.var_host = tk.StringVar(value=str(self.cfg.get("host") or DEFAULT_HOST))
        self.var_port = tk.StringVar(value=str(self.cfg.get("port") or DEFAULT_PORT))
        self.var_timeout = tk.StringVar(value=str(self.cfg.get("timeout") or DEFAULT_TIMEOUT))
        self.var_reply = tk.BooleanVar(value=bool(self.cfg.get("read_reply", False)))
        self.var_dash = tk.StringVar(value=str(self.cfg.get("dashboard") or "task -r"))
        self.var_dir = tk.StringVar(value=str(self.script_dir))
        self.var_status = tk.StringVar(value="未连接")
        self.history = list(self.cfg.get("command_history") or [])
        self.hist_list = None
        self.hist_detail = None
        self._listen_stop = threading.Event()
        self._listen_thread = None
        self._latest_pose = None
        self._listen_ok_logged = False
        self.var_listen = tk.StringVar(value="未监听")
        self._listening = False
        self.var_j = [tk.StringVar(value="—") for _ in range(6)]
        self.var_xyz = [tk.StringVar(value="—") for _ in range(3)]
        self.var_rpy = [tk.StringVar(value="—") for _ in range(3)]
        self._recording = False
        self._rec_lock = threading.Lock()
        self._rec_samples = []
        self._rec_t0 = None
        self.var_rec = tk.StringVar(value="")
        self.var_open_html = tk.BooleanVar(value=bool(self.cfg.get("open_html", False)))

        self._build_ui()
        self._refresh(log=False)
        self._restore_last_file()
        self._refresh_history_list()
        self._append("30001 下发脚本会立即执行；停止工程=stop，急停=halt+stop。")
        for var in (self.var_host, self.var_port, self.var_timeout, self.var_reply, self.var_dash, self.var_open_html):
            var.trace("w", lambda *_: self._schedule_save())
        self.txt.bind("<KeyRelease>", lambda *_: self._schedule_save())
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._loading = False
        self._start_listen()
        self._poll_pose()

    def _fit_geometry(self, saved):
        default = "780x500"
        if not saved:
            return default
        try:
            part = str(saved).split("+")[0]
            w, h = part.lower().split("x")
            if int(w) > 860 or int(h) > 540:
                return default
            return saved
        except (TypeError, ValueError):
            return default

    def _build_ui(self):
        style = ttk.Style(self.root)
        style.configure(".", font=("Microsoft YaHei", 9))
        style.configure("Tool.TButton", padding=(4, 1))
        style.configure("TNotebook.Tab", padding=(8, 2), font=("Microsoft YaHei", 9))

        bar = ttk.Frame(self.root, padding=(6, 4, 6, 2))
        bar.pack(fill="x")
        ttk.Label(bar, text="IP").pack(side="left")
        ttk.Entry(bar, textvariable=self.var_host, width=14).pack(side="left", padx=(3, 6))
        ttk.Label(bar, text="30001").pack(side="left")
        ttk.Entry(bar, textvariable=self.var_port, width=5).pack(side="left", padx=(3, 6))
        ttk.Label(bar, text="超时").pack(side="left")
        ttk.Entry(bar, textvariable=self.var_timeout, width=4).pack(side="left", padx=(3, 6))
        ttk.Checkbutton(bar, text="回执", variable=self.var_reply).pack(side="left")
        ttk.Button(bar, text="测连", command=self._ping, style="Tool.TButton", width=5).pack(side="left", padx=(4, 6))
        self.lbl_status = tk.Label(
            bar, textvariable=self.var_status, font=("Microsoft YaHei", 9),
            fg="#333333", anchor="e",
        )
        self.lbl_status.pack(side="right", padx=(8, 0))

        dash = ttk.LabelFrame(self.root, text=" Dashboard  29999 ", padding=(6, 3, 6, 4))
        dash.pack(fill="x", padx=6, pady=(0, 4))
        row1 = ttk.Frame(dash)
        row1.pack(fill="x")
        for text, cmd, w in (
            ("上电", self._power_on, 5),
            ("下电", self._power_off, 5),
            ("松闸", self._brake, 5),
            ("运行", self._play, 5),
            ("停止", self._stop_task, 5),
            ("查询", self._task, 5),
            ("清弹窗", self._close_popup, 6),
            ("取消", self._cancel_wait, 5),
        ):
            ttk.Button(row1, text=text, command=cmd, style="Tool.TButton", width=w).pack(side="left", padx=1)
        tk.Button(
            row1, text="急停", command=self._estop, width=5,
            bg="#c62828", fg="#ffffff", activebackground="#8e0000",
            activeforeground="#ffffff", relief="flat", cursor="hand2",
            font=("Microsoft YaHei", 9, "bold"), padx=8, pady=1,
        ).pack(side="right")

        pose = ttk.LabelFrame(self.root, text=" 实时位姿  30001 ", padding=(6, 3, 6, 4))
        pose.pack(fill="x", padx=6, pady=(0, 4))
        pose_bar = ttk.Frame(pose)
        pose_bar.pack(fill="x")
        self._listen_slot = ttk.Frame(pose_bar)
        self._listen_slot.pack(side="left")
        self.btn_listen_start = tk.Button(
            self._listen_slot, text="监听", command=self._start_listen, width=10,
            bg="#2e7d32", fg="#ffffff", activebackground="#1b5e20", activeforeground="#ffffff",
            relief="flat", cursor="hand2", font=("Microsoft YaHei", 9), padx=6, pady=1,
        )
        self.btn_listen_stop = tk.Button(
            self._listen_slot, text="停止监听", command=self._stop_listen, width=10,
            bg="#c62828", fg="#ffffff", activebackground="#8e0000", activeforeground="#ffffff",
            relief="flat", cursor="hand2", font=("Microsoft YaHei", 9), padx=6, pady=1,
        )
        self._record_slot = ttk.Frame(pose_bar)
        self._record_slot.pack(side="left", padx=(4, 8))
        self.btn_record_start = tk.Button(
            self._record_slot, text="记录", command=self._start_record, width=10,
            bg="#1565c0", fg="#ffffff", activebackground="#0d47a1", activeforeground="#ffffff",
            relief="flat", cursor="hand2", font=("Microsoft YaHei", 9), padx=6, pady=1,
        )
        self.btn_record_stop = tk.Button(
            self._record_slot, text="停止记录", command=self._stop_record, width=10,
            bg="#ef6c00", fg="#ffffff", activebackground="#e65100", activeforeground="#ffffff",
            relief="flat", cursor="hand2", font=("Microsoft YaHei", 9), padx=6, pady=1,
        )
        self._sync_pose_buttons()
        ttk.Checkbutton(pose_bar, text="打开HTML", variable=self.var_open_html).pack(side="left")
        self.lbl_listen = tk.Label(
            pose_bar, textvariable=self.var_listen, font=("Microsoft YaHei", 9), fg="#555555",
        )
        self.lbl_listen.pack(side="left", padx=(8, 0))
        self.lbl_rec = tk.Label(
            pose_bar, textvariable=self.var_rec, font=("Microsoft YaHei", 9), fg="#ef6c00",
        )
        self.lbl_rec.pack(side="right")
        num_font = ("Consolas", 10)
        cart_row = ttk.Frame(pose)
        cart_row.pack(fill="x", pady=(4, 2))
        ttk.Label(cart_row, text="笛卡尔").pack(side="left")
        for name, var, unit in (
            ("X", self.var_xyz[0], "mm"),
            ("Y", self.var_xyz[1], "mm"),
            ("Z", self.var_xyz[2], "mm"),
            ("Rx", self.var_rpy[0], "°"),
            ("Ry", self.var_rpy[1], "°"),
            ("Rz", self.var_rpy[2], "°"),
        ):
            ttk.Label(cart_row, text=name).pack(side="left", padx=(8, 2))
            tk.Label(cart_row, textvariable=var, font=num_font, fg="#111111", width=9, anchor="e").pack(side="left")
            ttk.Label(cart_row, text=unit, foreground="#777777").pack(side="left")
        joint_row = ttk.Frame(pose)
        joint_row.pack(fill="x")
        ttk.Label(joint_row, text="关节  ").pack(side="left")
        for i, var in enumerate(self.var_j):
            ttk.Label(joint_row, text="J%d" % (i + 1)).pack(side="left", padx=(8, 2))
            tk.Label(joint_row, textvariable=var, font=num_font, fg="#111111", width=8, anchor="e").pack(side="left")
            ttk.Label(joint_row, text="°", foreground="#777777").pack(side="left")

        body = ttk.Panedwindow(self.root, orient="horizontal")
        body.pack(fill="both", expand=True, padx=6, pady=(0, 4))

        left = ttk.Frame(body, padding=0)
        files_bar = ttk.Frame(left)
        files_bar.pack(fill="x")
        ttk.Label(files_bar, text="脚本", font=("Microsoft YaHei", 9)).pack(side="left")
        ttk.Button(files_bar, text="目录", command=self._pick_dir, style="Tool.TButton", width=4).pack(side="right")
        ttk.Button(files_bar, text="刷新", command=self._refresh, style="Tool.TButton", width=4).pack(side="right", padx=(0, 2))
        ttk.Label(left, textvariable=self.var_dir, font=("Microsoft YaHei", 7), foreground="#777777").pack(fill="x")
        self.lst = tk.Listbox(
            left, width=18, height=10, font=("Consolas", 8), exportselection=False,
            relief="flat", highlightthickness=1, highlightbackground="#cfd8dc",
        )
        self.lst.pack(fill="both", expand=True, pady=(2, 0))
        self.lst.bind("<<ListboxSelect>>", self._on_select)
        body.add(left, weight=0)

        editor = ttk.Frame(body)
        self.txt = scrolledtext.ScrolledText(
            editor, font=("Consolas", 9), undo=True, relief="flat", height=12,
            highlightthickness=1, highlightbackground="#cfd8dc",
        )
        self.txt.pack(fill="both", expand=True)
        self.txt.insert("1.0", self.cfg.get("script_text") or SAMPLE)
        sendbar = ttk.Frame(editor)
        sendbar.pack(fill="x", pady=(4, 0))
        ttk.Button(sendbar, text="下发脚本", command=self._send, style="Tool.TButton", width=8).pack(side="left")
        ttk.Button(sendbar, text="上电+松闸+下发", command=self._boot_send, style="Tool.TButton").pack(side="left", padx=6)
        body.add(editor, weight=1)

        tabs = ttk.Notebook(self.root)
        tabs.pack(fill="x", padx=6, pady=(0, 6))

        log_tab = ttk.Frame(tabs, padding=2)
        self.log = scrolledtext.ScrolledText(
            log_tab, height=6, font=("Microsoft YaHei", 10), state="disabled",
            relief="flat", bg="#ffffff", fg="#111111", insertbackground="#111111",
            highlightthickness=1, highlightbackground="#cfd8dc",
        )
        self.log.tag_configure("info", foreground="#111111")
        self.log.tag_configure("ok", foreground="#1b5e20")
        self.log.tag_configure("err", foreground="#b71c1c")
        self.log.tag_configure("busy", foreground="#e65100")
        self.log.pack(fill="both", expand=True)
        tabs.add(log_tab, text="日志")

        hist_tab = ttk.Frame(tabs, padding=2)
        self.hist_list = tk.Listbox(
            hist_tab, font=("Microsoft YaHei", 10), exportselection=False, height=6,
            relief="flat", highlightthickness=1, highlightbackground="#cfd8dc",
            fg="#111111", bg="#ffffff",
        )
        self.hist_list.pack(fill="both", expand=True)
        self.hist_list.bind("<Double-Button-1>", self._on_history_dbl)
        hbar = ttk.Frame(hist_tab)
        hbar.pack(fill="x", pady=(2, 0))
        ttk.Label(hbar, text="29999").pack(side="left")
        dash_entry = ttk.Entry(hbar, textvariable=self.var_dash)
        dash_entry.pack(side="left", padx=4, fill="x", expand=True)
        dash_entry.bind("<Return>", lambda *_: self._dashboard())
        ttk.Button(hbar, text="发送", command=self._dashboard, style="Tool.TButton", width=5).pack(side="left")
        ttk.Button(hbar, text="填入", command=self._history_fill, style="Tool.TButton", width=5).pack(side="left", padx=(6, 0))
        ttk.Button(hbar, text="重发", command=self._history_resend, style="Tool.TButton", width=5).pack(side="left", padx=4)
        ttk.Button(hbar, text="清空", command=self._history_clear, style="Tool.TButton", width=5).pack(side="right")
        tabs.add(hist_tab, text="历史")
        self.hist_detail = None

    def _set_status(self, text, kind="info"):
        colors = {"ok": "#2e7d32", "err": "#c62828", "info": "#555555", "busy": "#ef6c00"}
        self.var_status.set(text)
        self.lbl_status.configure(fg=colors.get(kind, "#555555"))

    def _history_item(self, index=None):
        if self.hist_list is None:
            return None
        sel = self.hist_list.curselection() if index is None else (index,)
        if not sel:
            return None
        pos = int(sel[0])
        real = len(self.history) - 1 - pos
        if real < 0 or real >= len(self.history):
            return None
        return self.history[real]

    def _refresh_history_list(self):
        if self.hist_list is None:
            return
        self.hist_list.delete(0, "end")
        for item in reversed(self.history):
            mark = "OK" if item.get("ok", True) else "NG"
            self.hist_list.insert("end", "%s  [%s]  %s" % (item.get("ts", ""), mark, item.get("title", "")))
        if self.history:
            self.hist_list.selection_set(0)
            self.hist_list.see(0)

    def _on_history_dbl(self, _evt=None):
        self._history_fill()

    def _history_fill(self):
        item = self._history_item()
        if item is None:
            return
        cmd = (item.get("cmd") or "").strip()
        if cmd in ("halt + stop", "boot_and_send") or cmd.startswith("send "):
            return
        self.var_dash.set(cmd)
        self._schedule_save()

    def _history_resend(self):
        item = self._history_item()
        if item is None:
            return
        title = item.get("title") or ""
        cmd = (item.get("cmd") or "").strip()
        if title == "下发" or cmd.startswith("send "):
            self._send()
        elif title == "急停":
            self._estop()
        elif title == "上电+松闸+下发":
            self._boot_send()
        elif cmd:
            self.var_dash.set(cmd)
            self._dashboard()

    def _history_clear(self):
        self.history = []
        self._refresh_history_list()
        self._schedule_save()

    def _record_history(self, title, cmd, result_text, ok=True):
        self.history.append({
            "ts": time.strftime("%H:%M:%S"),
            "title": title,
            "cmd": cmd or title,
            "result": (result_text or "").strip()[:800],
            "ok": bool(ok),
        })
        self.history = self.history[-80:]
        self._refresh_history_list()
        self._schedule_save()

    def _collect_settings(self):
        last_file = ""
        sel = self.lst.curselection()
        if sel:
            last_file = self.lst.get(sel[0]).split("  (")[0]
        elif self.cfg.get("last_file"):
            last_file = self.cfg.get("last_file")
        try:
            port = int(self.var_port.get())
        except ValueError:
            port = DEFAULT_PORT
        try:
            timeout = float(self.var_timeout.get())
        except ValueError:
            timeout = DEFAULT_TIMEOUT
        return {
            "host": self.var_host.get().strip(),
            "port": port,
            "timeout": timeout,
            "read_reply": bool(self.var_reply.get()),
            "script_dir": str(self.script_dir),
            "last_file": last_file,
            "script_text": self.txt.get("1.0", "end").rstrip("\n"),
            "dashboard": self.var_dash.get().strip(),
            "geometry": self.root.geometry(),
            "command_history": self.history[-80:],
            "open_html": bool(self.var_open_html.get()),
        }

    def _schedule_save(self):
        if self._loading:
            return
        if self._save_job is not None:
            self.root.after_cancel(self._save_job)
        self._save_job = self.root.after(400, self._save_now)

    def _save_now(self):
        self._save_job = None
        data = self._collect_settings()
        self.cfg = data
        save_settings(data)

    def _on_close(self):
        if self._save_job is not None:
            self.root.after_cancel(self._save_job)
        save_settings(self._collect_settings())
        if self._recording:
            self._stop_record(open_html=False)
        self._listen_stop.set()
        self.root.destroy()

    def _sync_pose_buttons(self):
        self.btn_listen_start.pack_forget()
        self.btn_listen_stop.pack_forget()
        self.btn_record_start.pack_forget()
        self.btn_record_stop.pack_forget()
        if self._listening:
            self.btn_listen_stop.pack()
        else:
            self.btn_listen_start.pack()
        if self._recording:
            self.btn_record_stop.pack()
        else:
            self.btn_record_start.pack()

    def _start_listen(self):
        self._listen_stop.set()
        self._listen_stop = threading.Event()
        self._listen_ok_logged = False
        self._listening = True
        self.var_listen.set("正在连接…")
        self.lbl_listen.configure(fg="#ef6c00")
        self._sync_pose_buttons()
        self._listen_thread = threading.Thread(target=self._listen_worker, daemon=True)
        self._listen_thread.start()

    def _stop_listen(self):
        self._listen_stop.set()
        self._listening = False
        self.var_listen.set("已停止")
        self.lbl_listen.configure(fg="#555555")
        self._sync_pose_buttons()

    def _on_listen_status(self, text, kind):
        self.var_listen.set(text[:72])
        colors = {"ok": "#2e7d32", "err": "#c62828", "info": "#555555", "busy": "#ef6c00"}
        self.lbl_listen.configure(fg=colors.get(kind, "#555555"))

    def _listen_worker(self):
        stop = self._listen_stop
        stream = RobotStateStream()
        while not stop.is_set():
            sock = None
            try:
                host, port, _timeout = self._params()
                sock = socket.create_connection((host, port), timeout=3)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.settimeout(1.0)
                stream.reset()
                self.root.after(0, lambda h=host, p=port: self._on_listen_status("监听中  %s:%s" % (h, p), "ok"))
                while not stop.is_set():
                    try:
                        chunk = sock.recv(8192)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise socket.error("连接已关闭")
                    poses = stream.feed(chunk)
                    if poses:
                        self._latest_pose = poses[-1]
                        if self._recording:
                            now = time.time()
                            with self._rec_lock:
                                t0 = self._rec_t0
                                if t0 is not None and len(self._rec_samples) < 500000:
                                    for parsed in poses:
                                        self._rec_samples.append(pose_sample(parsed, t0, now))
            except Exception as exc:
                if stop.is_set():
                    break
                self.root.after(0, lambda m=str(exc): self._on_listen_status("断开，重连中  %s" % m, "err"))
                stop.wait(1.5)
            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except Exception:
                        pass

    def _poll_pose(self):
        parsed = self._latest_pose
        if parsed:
            shown = format_pose(parsed)
            joints = shown.get("joints_deg")
            xyz = shown.get("xyz_mm")
            rpy = shown.get("rpy_deg")
            if joints:
                for i, val in enumerate(joints):
                    self.var_j[i].set("%.3f" % val)
            if xyz:
                for i, val in enumerate(xyz):
                    self.var_xyz[i].set("%.2f" % val)
            if rpy:
                for i, val in enumerate(rpy):
                    self.var_rpy[i].set("%.3f" % val)
        if self._recording:
            with self._rec_lock:
                n = len(self._rec_samples)
            self.var_rec.set("记录中  %d 点" % n)
        try:
            self.root.after(100, self._poll_pose)
        except tk.TclError:
            pass

    def _start_record(self):
        if self._recording:
            return
        if self._listen_stop.is_set():
            self._start_listen()
        with self._rec_lock:
            self._rec_samples = []
            self._rec_t0 = time.time()
        self._recording = True
        self.var_rec.set("记录中  0 点")
        self.lbl_rec.configure(fg="#ef6c00")
        self._sync_pose_buttons()
        self._append("开始记录 TCP / 关节")

    def _stop_record(self, open_html=None):
        if not self._recording and not self._rec_samples:
            return
        self._recording = False
        self._sync_pose_buttons()
        with self._rec_lock:
            samples = list(self._rec_samples)
            self._rec_t0 = None
        if not samples:
            self.var_rec.set("未采到点")
            self._append("记录停止：没有数据", "err")
            return
        stamp = time.strftime("%Y%m%d_%H%M%S")
        folder = Path(__file__).resolve().parent / "traces"
        try:
            csv_path = write_csv(folder / ("traj_%s.csv" % stamp), samples)
            html_path = write_html(folder / ("traj_%s.html" % stamp), samples)
        except OSError as exc:
            self._append("保存轨迹失败: %s" % exc, "err")
            return
        self.var_rec.set("已保存 %d 点" % len(samples))
        self.lbl_rec.configure(fg="#2e7d32")
        self._append("轨迹 CSV   %s" % csv_path, "ok")
        self._append("轨迹 HTML  %s" % html_path, "ok")
        if open_html is None:
            open_html = bool(self.var_open_html.get())
        if open_html:
            try:
                webbrowser.open(html_path.resolve().as_uri())
            except Exception:
                pass

    def _restore_last_file(self):
        name = self.cfg.get("last_file") or ""
        if not name:
            return
        path = self.script_dir / name
        for i in range(self.lst.size()):
            if self.lst.get(i).split("  (")[0] == name:
                self.lst.selection_set(i)
                self.lst.see(i)
                break
        if path.is_file() and not self.cfg.get("script_text"):
            try:
                text = _read_file(path)
                self.txt.delete("1.0", "end")
                self.txt.insert("1.0", text)
            except OSError:
                pass

    def _append(self, line, kind="info"):
        tag = kind if kind in ("ok", "err", "busy", "info") else "info"
        self.log.configure(state="normal")
        self.log.insert("end", line + "\n", tag)
        self.log.see("end")
        self.log.configure(state="disabled")
        self._set_status(line.split("\n")[0][:80], kind)

    def _refresh(self, log=True):
        self.script_dir.mkdir(parents=True, exist_ok=True)
        shown = str(self.script_dir)
        if len(shown) > 32:
            shown = "..." + shown[-29:]
        self.var_dir.set(shown)
        self.lst.delete(0, "end")
        files = list_script_files(self.script_dir)
        for p in files:
            self.lst.insert("end", "%s  (%d B)" % (p.name, p.stat().st_size))
        if log:
            self._append("目录 %s  共 %d 个脚本" % (self.script_dir, len(files)))

    def _pick_dir(self):
        chosen = filedialog.askdirectory(initialdir=str(self.script_dir))
        if chosen:
            self.script_dir = Path(chosen)
            self._refresh()
            self._schedule_save()

    def _on_select(self, _evt=None):
        sel = self.lst.curselection()
        if not sel:
            return
        name = self.lst.get(sel[0]).split("  (")[0]
        try:
            text = _read_file(self.script_dir / name)
        except OSError as exc:
            messagebox.showerror("读取失败", str(exc))
            return
        self.txt.delete("1.0", "end")
        self.txt.insert("1.0", text)
        self._schedule_save()

    def _params(self):
        return (
            self.var_host.get().strip(),
            int(self.var_port.get()),
            float(self.var_timeout.get()),
        )

    def _run_bg(self, title, fn, force=False):
        if self.busy and not force:
            return
        if not force:
            self.busy = True
            self.stop_event.clear()
            self._set_status(title + "…", "busy")
        else:
            self._set_status(title + "…", "err" if title == "急停" else "busy")

        def work():
            try:
                result = fn()
                self.root.after(0, lambda: self._done(title, result, None, force=force))
            except Exception:
                err = traceback.format_exc()
                self.root.after(0, lambda: self._done(title, None, err, force=force))

        threading.Thread(target=work, daemon=True).start()

    def _cancel_wait(self):
        self.stop_event.set()
        self._append("已取消等待（松闸/下电循环会退出）")

    def _done(self, title, result, err, force=False):
        if not force:
            self.busy = False
        if err:
            self._append("%s 失败\n%s" % (title, err.strip()), "err")
            self._record_history(title, title, err.strip(), ok=False)
            messagebox.showerror(title, err.splitlines()[-1])
            return
        if isinstance(result, dict):
            msg = result.get("message")
            extra = result.get("reply") or ""
            kind = "err" if result.get("ok") is False else "ok"
            if msg:
                extra_bit = extra if extra and extra != msg else ""
                line = "%s  %s  %s" % (title, msg, extra_bit)
                self._append(line, kind)
            else:
                line = "%s 成功  %s bytes  %sms  %s:%s  %s" % (
                    title,
                    result.get("bytes_sent", "-"),
                    result.get("duration_ms", "-"),
                    result.get("host"),
                    result.get("port"),
                    extra,
                )
                self._append(line, kind)
            cmd = TITLE_CMDS.get(title, title)
            if title.startswith("Dashboard"):
                cmd = title.replace("Dashboard ", "", 1)
            elif title == "下发":
                cmd = "send 30001 (%s bytes)" % result.get("bytes_sent", "")
            elif title == "急停":
                cmd = "halt + stop"
            elif title == "上电+松闸+下发":
                cmd = "boot_and_send"
            self._record_history(title, cmd, line, ok=(result.get("ok") is not False))
            if result.get("ok") is False:
                messagebox.showwarning(title, msg or extra or "失败")
        else:
            self._append("%s  %s" % (title, result), "ok")
            self._record_history(title, title, str(result), ok=True)

    def _ping(self):
        def fn():
            host, port, timeout = self._params()
            with socket.create_connection((host, port), timeout=timeout):
                pass
            return "已连通 %s:%s" % (host, port)

        self._set_status("正在连接…", "busy")
        self._run_bg("测试连接", fn)

    def _send(self):
        text = self.txt.get("1.0", "end").rstrip() + "\n"

        def fn():
            host, port, timeout = self._params()
            return send_text(
                text, host=host, port=port, timeout=timeout, read_reply=self.var_reply.get()
            )

        self._run_bg("下发", fn)

    def _power_on(self):
        def fn():
            host, _port, timeout = self._params()
            return powering_on(host=host, timeout=timeout)

        self._run_bg("上电", fn)

    def _power_off(self):
        def fn():
            host, _port, timeout = self._params()
            return powering_off(host=host, timeout=timeout, stop_event=self.stop_event)

        self._run_bg("下电", fn)

    def _brake(self):
        def fn():
            host, _port, timeout = self._params()
            return brake_releasing(host=host, timeout=timeout, stop_event=self.stop_event)

        self._run_bg("松闸", fn)

    def _play(self):
        def fn():
            host, _port, timeout = self._params()
            return play(host=host, timeout=timeout)

        self._run_bg("运行", fn)

    def _stop_task(self):
        def fn():
            host, _port, timeout = self._params()
            return stop_task(host=host, timeout=timeout)

        self._run_bg("停止工程", fn, force=True)

    def _estop(self):
        self.stop_event.set()

        def fn():
            host, _port, timeout = self._params()
            return emergency_stop(host=host, timeout=timeout)

        self._run_bg("急停", fn, force=True)

    def _task(self):
        def fn():
            host, _port, timeout = self._params()
            return task(host=host, timeout=timeout)

        self._run_bg("查询任务", fn)

    def _close_popup(self):
        def fn():
            host, _port, timeout = self._params()
            return close_popup(host=host, timeout=timeout)

        self._run_bg("清除弹窗", fn)

    def _boot_send(self):
        text = self.txt.get("1.0", "end").rstrip() + "\n"

        def fn():
            host, _port, timeout = self._params()
            return boot_and_send(
                text,
                host=host,
                timeout=timeout,
                stop_event=self.stop_event,
                read_reply=self.var_reply.get(),
            )

        self._run_bg("上电+松闸+下发", fn)

    def _dashboard(self):
        cmd = self.var_dash.get().strip()
        if not cmd:
            return

        def fn():
            host, _port, timeout = self._params()
            return dashboard_cmd(cmd, host=host, timeout=timeout)

        self._run_bg("Dashboard " + cmd, fn)


def main(argv=None):
    saved = load_settings()
    parser = argparse.ArgumentParser(description="Elite 30001 脚本下发测试")
    parser.add_argument("--host", default=saved.get("host") or DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=int(saved.get("port") or DEFAULT_PORT))
    parser.add_argument("--timeout", type=float, default=float(saved.get("timeout") or DEFAULT_TIMEOUT))
    parser.add_argument("--file", help="脚本文件路径")
    parser.add_argument("--stdin", action="store_true")
    parser.add_argument("--read-reply", action="store_true")
    parser.add_argument("--dashboard", help="29999 命令，例如 task -r")
    parser.add_argument("--power-on", action="store_true")
    parser.add_argument("--power-off", action="store_true")
    parser.add_argument("--close-popup", action="store_true")
    parser.add_argument("--stop-task", action="store_true")
    parser.add_argument("--estop", action="store_true")
    parser.add_argument("--brake", action="store_true")
    parser.add_argument("--play", action="store_true")
    parser.add_argument("--task", action="store_true")
    parser.add_argument("--boot-send", action="store_true")
    parser.add_argument("--cli", action="store_true")
    args = parser.parse_args(argv)

    if args.cli or args.file or args.stdin or args.dashboard or args.power_on or args.power_off or args.close_popup or args.stop_task or args.estop or args.brake or args.play or args.task or args.boot_send:
        return run_cli(args)
    if tk is None:
        print("无 tkinter，请加 --cli", file=sys.stderr)
        return 1
    root = tk.Tk()
    EliteScriptTester(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
