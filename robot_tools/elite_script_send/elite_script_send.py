#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Elite 脚本下发测试界面 / 命令行。"""
from __future__ import print_function

import argparse
import json
import socket
import sys
import threading
import traceback
from pathlib import Path

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
    list_script_files,
    play,
    powering_off,
    powering_on,
    send_text,
    task,
)

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
            args.power_on or args.power_off or args.close_popup or args.brake or args.play or args.task
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
        self.root.minsize(760, 500)
        self.cfg = load_settings()
        self.root.geometry(self.cfg.get("geometry") or "900x580")
        saved_dir = self.cfg.get("script_dir")
        self.script_dir = Path(saved_dir) if saved_dir else DEFAULT_SCRIPT_DIR
        self.busy = False
        self.stop_event = threading.Event()
        self._save_job = None
        self._loading = True

        style = ttk.Style(self.root)
        style.configure(".", font=("Microsoft YaHei", 9))

        bar = ttk.Frame(root, padding=6)
        bar.pack(fill="x")
        ttk.Label(bar, text="IP").pack(side="left")
        self.var_host = tk.StringVar(value=str(self.cfg.get("host") or DEFAULT_HOST))
        ttk.Entry(bar, textvariable=self.var_host, width=16).pack(side="left", padx=4)
        ttk.Label(bar, text="30001").pack(side="left")
        self.var_port = tk.StringVar(value=str(self.cfg.get("port") or DEFAULT_PORT))
        ttk.Entry(bar, textvariable=self.var_port, width=6).pack(side="left", padx=4)
        ttk.Label(bar, text="超时s").pack(side="left")
        self.var_timeout = tk.StringVar(value=str(self.cfg.get("timeout") or DEFAULT_TIMEOUT))
        ttk.Entry(bar, textvariable=self.var_timeout, width=5).pack(side="left", padx=4)
        self.var_reply = tk.BooleanVar(value=bool(self.cfg.get("read_reply", False)))
        ttk.Checkbutton(bar, text="读回执", variable=self.var_reply).pack(side="left", padx=8)
        ttk.Button(bar, text="测试连接", command=self._ping).pack(side="left", padx=4)
        ttk.Button(bar, text="下发 30001", command=self._send).pack(side="left")
        ttk.Button(bar, text="上电+松闸+下发", command=self._boot_send).pack(side="left", padx=4)
        ttk.Button(bar, text="停止", command=self._stop).pack(side="left")

        mid = ttk.Frame(root, padding=(6, 0, 6, 0))
        mid.pack(fill="both", expand=True)

        left = ttk.LabelFrame(mid, text=" 脚本目录 ", padding=4)
        left.pack(side="left", fill="y")
        self.lst = tk.Listbox(left, width=28, font=("Consolas", 9), exportselection=False)
        self.lst.pack(fill="both", expand=True)
        self.lst.bind("<<ListboxSelect>>", self._on_select)
        btns = ttk.Frame(left)
        btns.pack(fill="x", pady=4)
        ttk.Button(btns, text="刷新", command=self._refresh).pack(side="left")
        ttk.Button(btns, text="选目录", command=self._pick_dir).pack(side="left", padx=4)

        right = ttk.Frame(mid)
        right.pack(side="left", fill="both", expand=True, padx=(8, 0))
        self.txt = scrolledtext.ScrolledText(right, font=("Consolas", 10), undo=True)
        self.txt.pack(fill="both", expand=True)
        self.txt.insert("1.0", self.cfg.get("script_text") or SAMPLE)

        dash = ttk.Frame(root, padding=(6, 4, 6, 0))
        dash.pack(fill="x")
        ttk.Label(dash, text="29999").pack(side="left")
        ttk.Button(dash, text="上电", command=self._power_on).pack(side="left", padx=2)
        ttk.Button(dash, text="下电", command=self._power_off).pack(side="left", padx=2)
        ttk.Button(dash, text="松闸", command=self._brake).pack(side="left", padx=2)
        ttk.Button(dash, text="运行 play", command=self._play).pack(side="left", padx=2)
        ttk.Button(dash, text="查询任务", command=self._task).pack(side="left", padx=2)
        ttk.Button(dash, text="清除弹窗", command=self._close_popup).pack(side="left", padx=2)
        self.var_dash = tk.StringVar(value=str(self.cfg.get("dashboard") or "task -r"))
        ttk.Entry(dash, textvariable=self.var_dash, width=22).pack(side="left", padx=6)
        ttk.Button(dash, text="发 Dashboard", command=self._dashboard).pack(side="left")

        self.log = scrolledtext.ScrolledText(
            root, height=8, font=("Consolas", 9), state="disabled"
        )
        self.log.pack(fill="x", padx=6, pady=6)
        self._refresh()
        self._restore_last_file()
        self._append("上电 / 下电 / 松闸 / play / 查任务走 29999；下发走 30001，收到即执行。")
        for var in (self.var_host, self.var_port, self.var_timeout, self.var_reply, self.var_dash):
            var.trace("w", lambda *_: self._schedule_save())
        self.txt.bind("<KeyRelease>", lambda *_: self._schedule_save())
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._loading = False

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
        self.root.destroy()

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

    def _append(self, line):
        self.log.configure(state="normal")
        self.log.insert("end", line + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _refresh(self):
        self.script_dir.mkdir(parents=True, exist_ok=True)
        self.lst.delete(0, "end")
        files = list_script_files(self.script_dir)
        for p in files:
            self.lst.insert("end", "%s  (%d B)" % (p.name, p.stat().st_size))
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

    def _run_bg(self, title, fn):
        if self.busy:
            return
        self.busy = True
        self.stop_event.clear()

        def work():
            try:
                result = fn()
                self.root.after(0, lambda: self._done(title, result, None))
            except Exception:
                err = traceback.format_exc()
                self.root.after(0, lambda: self._done(title, None, err))

        threading.Thread(target=work, daemon=True).start()

    def _stop(self):
        self.stop_event.set()
        self._append("已请求停止（松闸/下电循环会退出）")

    def _done(self, title, result, err):
        self.busy = False
        if err:
            self._append("%s 失败\n%s" % (title, err.strip()))
            messagebox.showerror(title, err.splitlines()[-1])
            return
        if isinstance(result, dict):
            msg = result.get("message")
            extra = result.get("reply") or ""
            if msg:
                extra_bit = extra if extra and extra != msg else ""
                self._append("%s  %s  %s" % (title, msg, extra_bit))
            else:
                self._append(
                    "%s 成功  %s bytes  %sms  %s:%s  %s"
                    % (
                        title,
                        result.get("bytes_sent", "-"),
                        result.get("duration_ms", "-"),
                        result.get("host"),
                        result.get("port"),
                        extra,
                    )
                )
            if result.get("ok") is False:
                messagebox.showwarning(title, msg or extra or "失败")
        else:
            self._append("%s  %s" % (title, result))

    def _ping(self):
        def fn():
            host, port, timeout = self._params()
            with socket.create_connection((host, port), timeout=timeout):
                pass
            return "已连通 %s:%s" % (host, port)

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
    parser.add_argument("--brake", action="store_true")
    parser.add_argument("--play", action="store_true")
    parser.add_argument("--task", action="store_true")
    parser.add_argument("--boot-send", action="store_true")
    parser.add_argument("--cli", action="store_true")
    args = parser.parse_args(argv)

    if args.cli or args.file or args.stdin or args.dashboard or args.power_on or args.power_off or args.close_popup or args.brake or args.play or args.task or args.boot_send:
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
