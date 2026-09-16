#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Megmeet / 固高 CAN 焊机模拟器（虚拟机侧）。
"""

from __future__ import print_function

import os
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext

import can

ROBOT_ID = 0x1FD07063
WELDER_ID = 0x1FD08063
CMD_POLL = 0xFF
JOB_MODES = {
    0: "保留0", 1: "保留1", 2: "JOB", 3: "保留3",
    4: "保留4", 5: "保留5", 6: "保留6", 7: "保留7",
}
FLOW_NODES = ["插件", "RPC", "CanBus", "can0", "焊机"]
CAN0_NOTES = (
    "完整说明见同目录 can0启动说明.txt。\n"
    "建议：插件先连 MEGMEET CAN 拉起 can0，再点「打开总线」。\n"
    "「关闭」只松开本程序，不会 down 掉 can0。\n"
    "pip3 install python-can；缺界面则 sudo apt-get install -y python3-tk"
)
CHIP_OFF = {"bg": "#e5e5e5", "fg": "#777777"}
CHIP_ON = {"bg": "#e65100", "fg": "#ffffff"}
CHIP_OK = {"bg": "#2e7d32", "fg": "#ffffff"}
CHIP_ERR = {"bg": "#c62828", "fg": "#ffffff"}


def bit(value, pos):
    return bool(value & (1 << pos))


def set_bit(value, pos, on):
    return (value | (1 << pos)) if on else (value & ~(1 << pos))


def list_can_ifaces():
    net_dir = "/sys/class/net"
    names = []
    if os.path.isdir(net_dir):
        for name in sorted(os.listdir(net_dir)):
            if name.startswith("can") and os.path.isdir(os.path.join(net_dir, name)):
                names.append(name)
    return names or ["can0"]


def hex8(data):
    return " ".join("{:02X}".format(b) for b in data)


def run_cmd(args):
    try:
        p = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        if p.returncode == 0:
            return True, (p.stdout or b"").decode("utf-8", "ignore")
        err = (p.stderr or b"").decode("utf-8", "ignore").strip()
        p2 = subprocess.run(["sudo", "-n"] + args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        if p2.returncode == 0:
            return True, (p2.stdout or b"").decode("utf-8", "ignore")
        err2 = (p2.stderr or b"").decode("utf-8", "ignore").strip()
        return False, err2 or err or "exit {}".format(p.returncode)
    except Exception as exc:
        return False, str(exc)


class MegmeetCanResponder(object):
    def __init__(self, root):
        self.root = root
        self.root.title("Megmeet CAN 模拟焊机")
        self.root.geometry("700x460")
        self.root.minsize(620, 400)
        style = ttk.Style(self.root)
        style.configure(".", font=("Microsoft YaHei", 8))
        style.configure("TLabelframe.Label", font=("Microsoft YaHei", 8, "bold"))

        self.bus = None
        self.running = False
        self.rx_thread = None
        self.lock = threading.Lock()

        ifaces = list_can_ifaces()
        self.var_channel = tk.StringVar(value="can0" if "can0" in ifaces else ifaces[0])
        self.var_bitrate = tk.StringVar(value="125000")
        self.var_bustype = tk.StringVar(value="socketcan")
        self.var_auto_reply = tk.BooleanVar(value=True)
        self.var_auto_arc = tk.BooleanVar(value=True)
        self.var_log_changes = tk.BooleanVar(value=True)

        self.var_ready = tk.BooleanVar(value=True)
        self.var_error = tk.BooleanVar(value=False)
        self.var_touch_ok = tk.BooleanVar(value=False)
        self.var_arc_ok = tk.BooleanVar(value=False)
        self.var_arc_ex = tk.BooleanVar(value=False)
        self.var_other_ex = tk.BooleanVar(value=False)
        self.var_stick = tk.BooleanVar(value=False)
        self.var_current = tk.StringVar(value="180")
        self.var_voltage = tk.StringVar(value="22.5")
        self.var_errcode = tk.StringVar(value="0")

        self.lbl_robot = {}
        self.rx_byte_cells = []
        self.tx_byte_cells = []
        self.flow_boxes = []
        self.flow_fwd = []
        self.flow_back = []

        self.poll_count = 0
        self.map_count = 0
        self.reply_count = 0
        self.last_rx_key = None
        self.last_weld = None
        self.hz = 0.0
        self.rx_times = []

        self.build_ui()
        self.append_log("口: {}。点「打开总线」。说明见顶部「说明」。".format(", ".join(ifaces)))

    def build_ui(self):
        self._build_port_bar()
        self._build_flow()
        self._build_body()
        self._build_log()

    def _chip(self, parent, key, title):
        box = tk.Frame(parent)
        tk.Label(box, text=title, font=("Microsoft YaHei", 7), fg="#555").pack(side="top")
        lab = tk.Label(box, text="—", width=6, font=("Microsoft YaHei", 8, "bold"), **CHIP_OFF)
        lab.pack(side="top", pady=(1, 0))
        self.lbl_robot[key] = lab
        return box

    def _set_chip(self, key, on, text_on="开", text_off="关", style=None):
        lab = self.lbl_robot.get(key)
        if lab is None:
            return
        colors = style if on else CHIP_OFF
        lab.config(text=text_on if on else text_off, **colors)

    def _build_port_bar(self):
        bar = ttk.Frame(self.root, padding=(6, 4))
        bar.pack(fill="x")

        ttk.Label(bar, text="端口").pack(side="left")
        self.cmb_channel = ttk.Combobox(bar, textvariable=self.var_channel, width=6, values=list_can_ifaces())
        self.cmb_channel.pack(side="left", padx=(4, 2))
        ttk.Button(bar, text="刷新", command=self.refresh_ifaces, width=4).pack(side="left")
        ttk.Combobox(bar, textvariable=self.var_bitrate, width=7,
                     values=("125000", "250000", "500000")).pack(side="left", padx=4)
        ttk.Combobox(bar, textvariable=self.var_bustype, width=8,
                     values=("socketcan", "slcan", "pcan", "usb2can")).pack(side="left")

        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y", padx=8, pady=2)

        self.led = tk.Label(bar, text="关", width=3, bg="#bdbdbd", fg="#333333",
                            font=("Microsoft YaHei", 8), relief="groove")
        self.led.pack(side="left")
        self.btn_open = ttk.Button(bar, text="打开总线", command=self.open_port, width=8)
        self.btn_open.pack(side="left", padx=(6, 2))
        self.btn_close = ttk.Button(bar, text="关闭", command=self.close_port, width=6, state="disabled")
        self.btn_close.pack(side="left")
        ttk.Button(bar, text="说明", command=self.show_notes, width=4).pack(side="left", padx=6)

        ttk.Checkbutton(bar, text="应答", variable=self.var_auto_reply).pack(side="right", padx=2)
        ttk.Checkbutton(bar, text="起焊沿起弧", variable=self.var_auto_arc).pack(side="right")
        ttk.Checkbutton(bar, text="只记变化", variable=self.var_log_changes).pack(side="right", padx=2)

        stat = ttk.Frame(self.root, padding=(6, 0, 6, 2))
        stat.pack(fill="x")
        self.lbl_hz = ttk.Label(stat, text="0 Hz")
        self.lbl_hz.pack(side="left")
        self.lbl_cnt = ttk.Label(stat, text="  映射0 轮询0 应答0")
        self.lbl_cnt.pack(side="left")
        self.lbl_last = ttk.Label(stat, text="等待打开总线")
        self.lbl_last.pack(side="left", padx=8)

    def _build_flow(self):
        self.canvas = tk.Canvas(self.root, height=36, bg="#eceff1", highlightthickness=0)
        self.canvas.pack(fill="x")
        self.root.after(60, self._draw_flow)
        self.root.bind("<Configure>", lambda e: self._schedule_flow_redraw())
        self._flow_redraw_job = None

    def _schedule_flow_redraw(self):
        if self._flow_redraw_job:
            self.root.after_cancel(self._flow_redraw_job)
        self._flow_redraw_job = self.root.after(120, self._draw_flow)

    def _draw_flow(self):
        c = self.canvas
        c.delete("all")
        w = max(c.winfo_width(), 600)
        n = len(FLOW_NODES)
        box_w, box_h = 70, 22
        gap = max(8, (w - 12 - n * box_w) / float(n - 1))
        y = 7
        self.flow_boxes = []
        self.flow_fwd = []
        self.flow_back = []
        xs = []
        for i, title in enumerate(FLOW_NODES):
            x = 6 + i * (box_w + gap)
            xs.append(x)
            box = c.create_rectangle(x, y, x + box_w, y + box_h, fill="#ffffff", outline="#607d8b", width=1)
            c.create_text(x + box_w / 2, y + box_h / 2, text=title, font=("Microsoft YaHei", 8))
            self.flow_boxes.append(box)
        for i in range(n - 1):
            x1 = xs[i] + box_w
            x2 = xs[i + 1]
            self.flow_fwd.append(c.create_line(x1 + 1, y + 7, x2 - 1, y + 7, arrow=tk.LAST, fill="#b0bec5", width=2))
            self.flow_back.append(c.create_line(x2 - 1, y + 16, x1 + 1, y + 16, arrow=tk.LAST, fill="#b0bec5", width=2))

    def _flash_flow(self, direction):
        color = "#e65100" if direction == "rx" else "#2e7d32"
        items = self.flow_fwd if direction == "rx" else self.flow_back
        boxes = self.flow_boxes if direction == "rx" else list(reversed(self.flow_boxes))
        for item in items:
            self.canvas.itemconfig(item, fill=color, width=3)
        for box in boxes:
            self.canvas.itemconfig(box, outline=color, width=2)
        self.root.after(220, self._reset_flow)

    def _reset_flow(self):
        for item in self.flow_fwd + self.flow_back:
            self.canvas.itemconfig(item, fill="#b0bec5", width=2)
        for box in self.flow_boxes:
            self.canvas.itemconfig(box, outline="#607d8b", width=1)

    def _build_body(self):
        mid = tk.Frame(self.root)
        mid.pack(fill="both", expand=True, padx=4, pady=2)

        left = tk.LabelFrame(mid, text=" 插件下发  0x1FD07063 ",
                             font=("Microsoft YaHei", 8), padx=4, pady=2)
        left.pack(side="left", fill="both", expand=True, padx=(0, 3))
        self._build_byte_row(left, "RX", self.rx_byte_cells, "#fff3e0")
        chips = tk.Frame(left)
        chips.pack(fill="x", pady=3)
        for key, title in (
            ("开始焊接", "起焊"), ("寻位", "寻位"), ("检气", "检气"),
            ("送丝", "送丝"), ("回抽", "回抽"), ("机器人故障", "机故"),
        ):
            self._chip(chips, key, title).pack(side="left", padx=2)
        extra = tk.Frame(left)
        extra.pack(fill="x")
        tk.Label(extra, text="命令", font=("Microsoft YaHei", 7)).pack(side="left")
        self.lbl_cmd = tk.Label(extra, text="—", font=("Consolas", 9, "bold"), fg="#e65100")
        self.lbl_cmd.pack(side="left", padx=4)
        tk.Label(extra, text="JOB", font=("Microsoft YaHei", 7)).pack(side="left")
        self.lbl_job = tk.Label(extra, text="—", font=("Consolas", 11, "bold"), fg="#0d47a1")
        self.lbl_job.pack(side="left", padx=2)
        tk.Label(extra, text="模式", font=("Microsoft YaHei", 7)).pack(side="left", padx=(8, 0))
        self.lbl_mode = tk.Label(extra, text="—", font=("Microsoft YaHei", 8, "bold"))
        self.lbl_mode.pack(side="left")

        right = tk.LabelFrame(mid, text=" 焊机回帧  0x1FD08063 ",
                              font=("Microsoft YaHei", 8), padx=4, pady=2)
        right.pack(side="left", fill="both", expand=True)
        self._build_byte_row(right, "TX", self.tx_byte_cells, "#e8f5e9")

        checks = tk.Frame(right)
        checks.pack(fill="x", pady=2)
        for i, (text, var) in enumerate((
            ("就绪", self.var_ready),
            ("故障", self.var_error),
            ("寻位成功", self.var_touch_ok),
            ("起弧成功", self.var_arc_ok),
            ("电弧异常", self.var_arc_ex),
            ("其他异常", self.var_other_ex),
            ("粘丝", self.var_stick),
        )):
            ttk.Checkbutton(checks, text=text, variable=var, command=self._refresh_tx_preview).grid(
                row=i // 4, column=i % 4, sticky="w", padx=2
            )

        vals = tk.Frame(right)
        vals.pack(fill="x", pady=2)
        tk.Label(vals, text="A", font=("Microsoft YaHei", 8, "bold")).pack(side="left")
        ttk.Entry(vals, textvariable=self.var_current, width=5).pack(side="left", padx=(0, 6))
        tk.Label(vals, text="V", font=("Microsoft YaHei", 8, "bold")).pack(side="left")
        ttk.Entry(vals, textvariable=self.var_voltage, width=5).pack(side="left", padx=(0, 6))
        tk.Label(vals, text="错", font=("Microsoft YaHei", 8, "bold")).pack(side="left")
        ttk.Entry(vals, textvariable=self.var_errcode, width=4).pack(side="left")
        ttk.Button(vals, text="手发一帧", command=self.send_reply, width=8).pack(side="right")

        self.var_current.trace("w", lambda *_: self._refresh_tx_preview())
        self.var_voltage.trace("w", lambda *_: self._refresh_tx_preview())
        self.var_errcode.trace("w", lambda *_: self._refresh_tx_preview())
        self.root.after(150, self._refresh_tx_preview)

    def _build_byte_row(self, parent, prefix, store, bg):
        row = tk.Frame(parent)
        row.pack(fill="x")
        tk.Label(row, text=prefix, width=3, font=("Consolas", 8, "bold")).pack(side="left")
        for _ in range(8):
            cell = tk.Label(row, text="--", width=3, relief="solid", bd=1,
                            font=("Consolas", 10, "bold"), bg=bg)
            cell.pack(side="left", padx=1)
            store.append(cell)

    def _build_log(self):
        frame = tk.LabelFrame(self.root, text="时间线", font=("Microsoft YaHei", 8, "bold"))
        frame.pack(fill="both", expand=True, padx=4, pady=(0, 4))
        self.log = scrolledtext.ScrolledText(frame, height=5, font=("Consolas", 8))
        self.log.pack(fill="both", expand=True)
        self.log.tag_config("rx", foreground="#c05600")
        self.log.tag_config("tx", foreground="#1b5e20")
        self.log.tag_config("info", foreground="#333")
        self.log.tag_config("chg", foreground="#0d47a1")
        ttk.Button(frame, text="清空", command=lambda: self.log.delete("1.0", "end"), width=5).pack(
            anchor="e", padx=2, pady=1
        )

    def refresh_ifaces(self):
        names = list_can_ifaces()
        self.cmb_channel["values"] = names
        if self.var_channel.get() not in names:
            self.var_channel.set("can0" if "can0" in names else names[0])
        self.append_log("刷新: {}".format(", ".join(names)), "info")

    def append_log(self, text, tag="info"):
        self.log.insert("end", time.strftime("[%H:%M:%S] ") + text + "\n", tag)
        self.log.see("end")

    def set_bytes(self, cells, data, active_bg):
        for i, cell in enumerate(cells):
            cell.config(text="{:02X}".format(data[i]) if i < len(data) else "--",
                        bg=active_bg if i < len(data) else "#eee")

    def _refresh_tx_preview(self):
        if self.tx_byte_cells:
            self.set_bytes(self.tx_byte_cells, self.build_reply(), "#c8e6c9")

    def _set_led(self, on):
        if on:
            self.led.config(text="开", bg="#c8e6c9", fg="#1b5e20")
            self.btn_open.config(state="disabled")
            self.btn_close.config(state="normal")
        else:
            self.led.config(text="关", bg="#bdbdbd", fg="#333333")
            self.btn_open.config(state="normal")
            self.btn_close.config(state="disabled")

    def iface_path(self, name):
        return os.path.join("/sys/class/net", name)

    def iface_exists(self, name):
        return os.path.isdir(self.iface_path(name))

    def iface_is_up(self, name):
        oper = os.path.join(self.iface_path(name), "operstate")
        if not os.path.isfile(oper):
            return False
        try:
            state = open(oper, "r").read().strip().lower()
        except Exception:
            return False
        return state in ("up", "unknown")

    def bring_iface_up(self):
        name = self.var_channel.get()
        baud = self.var_bitrate.get()
        if self.var_bustype.get() != "socketcan":
            return True
        if not self.iface_exists(name):
            self.append_log("没有网卡 {}，跳过 ip link".format(name), "info")
            return False
        if self.iface_is_up(name):
            self.append_log("{} 已是 UP，直接绑定".format(name), "info")
            return True
        ok, msg = run_cmd(["ip", "link", "set", name, "up", "type", "can", "bitrate", baud])
        if ok:
            self.append_log("端口已 up: {} bitrate {}".format(name, baud), "info")
            return True
        ok2, msg2 = run_cmd(["ip", "link", "set", name, "up"])
        if ok2:
            self.append_log("端口已 up: {}".format(name), "info")
            return True
        self.append_log("ip link 无权限或失败（可忽略）: {}".format(msg2 or msg), "info")
        return self.iface_exists(name)

    def bring_iface_down(self):
        name = self.var_channel.get()
        if self.var_bustype.get() != "socketcan":
            return
        ok, msg = run_cmd(["ip", "link", "set", name, "down"])
        self.append_log("端口 down {}: {}".format(name, "ok" if ok else msg), "info")

    def show_notes(self):
        note_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "can0启动说明.txt")
        text = CAN0_NOTES
        try:
            with open(note_path, "r", encoding="utf-8") as f:
                text = f.read()
        except Exception:
            pass
        win = tk.Toplevel(self.root)
        win.title("虚拟机 CAN 模拟说明")
        win.geometry("640x480")
        box = scrolledtext.ScrolledText(win, font=("Microsoft YaHei", 9), wrap="word")
        box.pack(fill="both", expand=True, padx=6, pady=6)
        box.insert("1.0", text)
        box.config(state="disabled")
        ttk.Button(win, text="关闭", command=win.destroy).pack(pady=(0, 6))

    def open_port(self):
        self.refresh_ifaces()
        name = self.var_channel.get()
        if self.var_bustype.get() == "socketcan" and not self.iface_exists(name):
            messagebox.showerror(
                "打开总线",
                "系统里没有 {}。\n\n先在插件里连接 MEGMEET CAN（会拉起 can0），\n或确认 USB2CAN 已枚举，再点刷新。\n点「说明」可看 can0 启动步骤。".format(name)
            )
            return
        self.bring_iface_up()
        self.open_bus()

    def close_port(self):
        self.close_bus()

    def open_bus(self):
        kwargs = {"bustype": self.var_bustype.get(), "channel": self.var_channel.get()}
        if self.var_bustype.get() != "socketcan":
            kwargs["bitrate"] = int(self.var_bitrate.get())
        try:
            self.bus = can.interface.Bus(**kwargs)
        except Exception as exc:
            messagebox.showerror("打开失败", str(exc))
            return
        self.running = True
        self.rx_thread = threading.Thread(target=self.rx_loop)
        self.rx_thread.daemon = True
        self.rx_thread.start()
        self._set_led(True)
        self.lbl_last.config(text="总线已开，等插件连 MEGMEET CAN → {}".format(self.var_channel.get()))
        self.append_log("已绑定 {} / {}".format(self.var_bustype.get(), self.var_channel.get()), "info")

    def close_bus(self):
        self.running = False
        bus = self.bus
        self.bus = None
        if bus is not None:
            try:
                bus.shutdown()
            except Exception:
                pass
        self._set_led(False)
        self.lbl_last.config(text="总线已关")
        self.append_log("已关闭绑定", "info")

    def rx_loop(self):
        while self.running and self.bus is not None:
            try:
                msg = self.bus.recv(timeout=0.2)
            except Exception as exc:
                self.root.after(0, self.append_log, "收帧异常: {}".format(exc), "info")
                break
            if msg is None or msg.arbitration_id != ROBOT_ID:
                continue
            data = list(msg.data) + [0] * 8
            self.root.after(0, self.on_robot_frame, data[:8])

    def on_robot_frame(self, data):
        now = time.time()
        self.rx_times = [t for t in self.rx_times if now - t < 1.0]
        self.rx_times.append(now)
        self.hz = len(self.rx_times)

        cmd = data[0]
        robot_err = bit(data[1], 0)
        seek = bit(data[1], 1)
        weld = bit(data[2], 0)
        gas = bit(data[2], 1)
        feed = bit(data[2], 2)
        retract = bit(data[2], 3)
        mode = (data[2] >> 5) & 0x07
        job = data[3]
        rx_key = tuple(data[:4])

        self.set_bytes(self.rx_byte_cells, data, "#ffe0b2")
        self._flash_flow("rx")

        self.lbl_cmd.config(text="0x{:02X} {}".format(cmd, "轮询" if cmd == CMD_POLL else "映射"))
        self.lbl_job.config(text=str(job))
        self.lbl_mode.config(text=JOB_MODES.get(mode, str(mode)))
        self._set_chip("开始焊接", weld, style=CHIP_ON)
        self._set_chip("寻位", seek, style=CHIP_ON)
        self._set_chip("检气", gas, style=CHIP_ON)
        self._set_chip("送丝", feed, style=CHIP_ON)
        self._set_chip("回抽", retract, style=CHIP_ON)
        self._set_chip("机器人故障", robot_err, text_on="是", text_off="否", style=CHIP_ERR)

        changed = rx_key != self.last_rx_key
        self.last_rx_key = rx_key

        if cmd in (0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5):
            self.map_count += 1
            action = "映射 F{:X}→{}".format(cmd & 0x0F, data[1])
            self.append_log("↓ " + action + "  [" + hex8(data) + "]", "rx")
        else:
            self.poll_count += 1
            bits = [n for n, f in (("起焊", weld), ("检气", gas), ("送丝", feed), ("回抽", retract), ("寻位", seek)) if f]
            action = "JOB={} {} {}".format(job, JOB_MODES.get(mode, mode), " ".join(bits) or "空闲")
            if changed or not self.var_log_changes.get():
                self.append_log("↓ " + action + "  [" + hex8(data) + "]", "chg" if changed else "rx")

        self.lbl_last.config(text=action)
        self.lbl_hz.config(text="{:.0f} Hz".format(self.hz))
        self.lbl_cnt.config(text="  映射{} 轮询{} 应答{}".format(
            self.map_count, self.poll_count, self.reply_count))

        if self.var_auto_arc.get() and cmd == CMD_POLL:
            if self.last_weld is None:
                self.last_weld = weld
            elif weld and not self.last_weld:
                self.var_arc_ok.set(True)
                self.append_log("起焊↑ 自动勾起弧成功", "tx")
            elif (not weld) and self.last_weld:
                self.var_arc_ok.set(False)
                self.append_log("起焊↓ 清起弧成功", "info")
            self.last_weld = weld
        if seek:
            self.var_touch_ok.set(True)
        if self.var_auto_reply.get():
            self.send_reply(from_auto=True, highlight_change=changed)

    def build_reply(self):
        data = [0] * 8
        data[1] = set_bit(data[1], 0, self.var_ready.get())
        data[1] = set_bit(data[1], 1, self.var_error.get())
        data[1] = set_bit(data[1], 2, self.var_touch_ok.get())
        data[2] = set_bit(data[2], 0, self.var_arc_ok.get())
        data[2] = set_bit(data[2], 1, self.var_arc_ex.get())
        data[2] = set_bit(data[2], 2, self.var_other_ex.get())
        data[2] = set_bit(data[2], 3, self.var_stick.get())
        try:
            current = max(0, min(65535, int(float(self.var_current.get()))))
        except ValueError:
            current = 0
        try:
            voltage_x10 = max(0, min(65535, int(round(float(self.var_voltage.get()) * 10))))
        except ValueError:
            voltage_x10 = 0
        try:
            err = max(0, min(255, int(self.var_errcode.get())))
        except ValueError:
            err = 0
        data[3] = (current >> 8) & 0xFF
        data[4] = current & 0xFF
        data[5] = (voltage_x10 >> 8) & 0xFF
        data[6] = voltage_x10 & 0xFF
        data[7] = err
        return data

    def send_reply(self, from_auto=False, highlight_change=True):
        if self.bus is None:
            if not from_auto:
                messagebox.showwarning("未打开", "先点「打开总线」")
            return
        data = self.build_reply()
        msg = can.Message(arbitration_id=WELDER_ID, data=data, is_extended_id=True)
        try:
            with self.lock:
                self.bus.send(msg)
        except Exception as exc:
            self.append_log("发帧失败: {}".format(exc), "info")
            return
        self.reply_count += 1
        self.set_bytes(self.tx_byte_cells, data, "#a5d6a7")
        self._flash_flow("tx")
        flags = [n for n, f in (
            ("就绪", self.var_ready.get()),
            ("故障", self.var_error.get()),
            ("寻位成功", self.var_touch_ok.get()),
            ("起弧成功", self.var_arc_ok.get()),
            ("电弧异常", self.var_arc_ex.get()),
        ) if f]
        if highlight_change or not self.var_log_changes.get() or not from_auto:
            self.append_log("↑ {} {}A {}V  [{}]".format(
                " ".join(flags) or "全关", self.var_current.get(),
                self.var_voltage.get(), hex8(data)), "tx")
        self.lbl_cnt.config(text="  映射{} 轮询{} 应答{}".format(
            self.map_count, self.poll_count, self.reply_count))


if __name__ == "__main__":
    root = tk.Tk()
    app = MegmeetCanResponder(root)

    def on_close():
        app.close_bus()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()
