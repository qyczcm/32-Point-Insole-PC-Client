"""串口体重秤上位机。

秤持续发送一行一个读数，例如 `-0.06,kg\\r\\n`。
选择串口和波特率后打开，界面持续显示解析结果。
点「记录」后，每一条成功解析的重量连同接收时间写入 CSV。
"""

import csv
import queue
import sys
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少 pyserial。请先执行: pip install pyserial", file=sys.stderr)
    raise SystemExit(1)

OUTPUT_DIR = Path(__file__).resolve().parent / "data"
BAUDRATES = ["9600", "4800", "19200", "38400", "57600", "115200"]
UI_INTERVAL_MS = 100


def parse_weight_line(raw: bytes) -> float | None:
    text = raw.decode("ascii", errors="ignore").strip()
    if not text:
        return None
    value_text = text.split(",", 1)[0].strip()
    try:
        return float(value_text)
    except ValueError:
        return None


class WeightRecorder:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._queue: queue.Queue | None = None
        self._thread: threading.Thread | None = None
        self.path: Path | None = None
        self.count = 0
        self.active = False
        self.note = ""

    def start(self, directory: Path) -> Path | None:
        self.stop()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
            if thread.is_alive():
                self.note = "上一段记录还在保存，请稍后再试"
                return None
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"scale_{datetime.now():%Y%m%d_%H%M%S}.csv"
        with self._lock:
            self.path = path
            self.count = 0
            self.note = f"记录中 0 条  {path.name}"
            self._queue = queue.Queue()
            self.active = True
            self._thread = threading.Thread(target=self._loop, args=(path, self._queue), daemon=True)
            self._thread.start()
        print(f"开始记录 {path}", flush=True)
        return path

    def add(self, stamp: str, weight: float) -> None:
        with self._lock:
            if not self.active or self._queue is None:
                return
            self._queue.put((stamp, weight))
            self.count += 1

    def stop(self) -> None:
        with self._lock:
            if not self.active:
                return
            self.active = False
            pending = self._queue
            if self.count:
                self.note = f"正在保存 {self.count} 条..."
        if pending is not None:
            pending.put(None)

    def _loop(self, path: Path, pending: queue.Queue) -> None:
        saved = 0
        try:
            with path.open("w", newline="", encoding="utf-8-sig") as handle:
                writer = csv.writer(handle)
                writer.writerow(["时间", "重量kg"])
                last_flush = time.monotonic()
                while True:
                    item = pending.get()
                    if item is None:
                        break
                    stamp, weight = item
                    writer.writerow([stamp, f"{weight:.3f}"])
                    saved += 1
                    now = time.monotonic()
                    if now - last_flush >= 1.0:
                        handle.flush()
                        last_flush = now
                handle.flush()
            self.count = saved
            self.note = f"已保存 {saved} 条  {path.name}"
        except Exception as exc:
            self.active = False
            self.note = f"记录失败：{exc}"
        print(self.note, flush=True)


class ScaleView(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("串口体重秤")
        self.resizable(False, False)
        self.configure(bg="#f7f7f7")

        self.lock = threading.Lock()
        self.stop_flag = threading.Event()
        self.worker: threading.Thread | None = None
        self.serial_conn: serial.Serial | None = None
        self.connected = False
        self._closed_pending = False
        self.weight: float | None = None
        self.raw_text = ""
        self.sample_count = 0
        self.status_text = "未连接"
        self.recorder = WeightRecorder()
        self._status_shown = ""
        self._summary_shown = ""
        self._record_btn_shown: tuple[str, str] | None = None

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="9600")
        self._ports: dict[str, str] = {}
        self._build()
        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(UI_INTERVAL_MS, self._refresh)

    def _build(self) -> None:
        top = tk.Frame(self, bg="#f7f7f7")
        top.pack(fill="x", padx=12, pady=(12, 6))
        tk.Label(top, text="串口", bg="#f7f7f7").pack(side="left")
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=28, state="readonly")
        self.port_combo.pack(side="left", padx=(6, 6))
        ttk.Button(top, text="刷新", command=self.refresh_ports).pack(side="left")
        tk.Label(top, text="波特率", bg="#f7f7f7").pack(side="left", padx=(12, 0))
        self.baud_combo = ttk.Combobox(
            top, textvariable=self.baud_var, width=8, values=BAUDRATES, state="readonly"
        )
        self.baud_combo.pack(side="left", padx=(6, 10))
        self.open_btn = ttk.Button(top, text="打开串口", command=self._open)
        self.open_btn.pack(side="left")
        self.close_btn = ttk.Button(top, text="关闭", command=self._close, state="disabled")
        self.close_btn.pack(side="left", padx=(6, 0))
        self.record_btn = ttk.Button(top, text="记录", command=self._toggle_record, state="disabled")
        self.record_btn.pack(side="left", padx=(6, 0))

        self.status = tk.Label(self, text=self.status_text, anchor="w", bg="#f7f7f7", fg="#333333")
        self.status.pack(fill="x", padx=12, pady=(0, 8))

        body = tk.Frame(self, bg="#f7f7f7")
        body.pack(fill="x", padx=12, pady=8)
        tk.Label(body, text="重量 kg", bg="#f7f7f7", fg="#666666", font=("Microsoft YaHei UI", 12)).pack()
        self.weight_label = tk.Label(
            body, text="--", bg="#f7f7f7", fg="#1c1c1c", font=("Consolas", 42, "bold")
        )
        self.weight_label.pack(pady=(4, 8))
        self.raw_label = tk.Label(
            body, text="等待数据", bg="#f7f7f7", fg="#666666", font=("Consolas", 11)
        )
        self.raw_label.pack()

        self.summary = tk.Label(self, text="采样 0", anchor="w", bg="#f7f7f7")
        self.summary.pack(fill="x", padx=12, pady=(8, 12))

    def refresh_ports(self) -> None:
        if self.connected:
            return
        found = list(list_ports.comports())
        labels = []
        mapping: dict[str, str] = {}
        for port in found:
            label = port.device if not port.description else f"{port.device}: {port.description}"
            labels.append(label)
            mapping[label] = port.device
        self._ports = mapping
        self.port_combo["values"] = labels
        current = self.port_var.get()
        if current not in labels:
            self.port_var.set(labels[0] if labels else "")

    def _selected_port(self) -> str:
        label = self.port_var.get().strip()
        return self._ports.get(label, label.split(":", 1)[0].strip())

    def _set_status(self, text: str) -> None:
        with self.lock:
            self.status_text = text
        print(text, flush=True)

    def _open(self) -> None:
        if self.worker and self.worker.is_alive():
            return
        port = self._selected_port()
        if not port:
            self._set_status("没有可用串口")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            self._set_status("波特率无效")
            return
        try:
            conn = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.2,
            )
            conn.dtr = True
            conn.rts = False
        except Exception as exc:
            self._set_status(f"打开失败：{exc}")
            return
        self.serial_conn = conn
        self.stop_flag.clear()
        with self.lock:
            self.sample_count = 0
            self.weight = None
            self.raw_text = ""
        self.connected = True
        self.open_btn.configure(state="disabled")
        self.close_btn.configure(state="normal")
        self.port_combo.configure(state="disabled")
        self.baud_combo.configure(state="disabled")
        self._set_status(f"已打开 {port}  {baud} 8N1")
        self.worker = threading.Thread(target=self._read_loop, daemon=True)
        self.worker.start()

    def _close(self) -> None:
        self.recorder.stop()
        self.stop_flag.set()
        self._set_status("正在关闭...")

    def _toggle_record(self) -> None:
        if self.recorder.active:
            self.recorder.stop()
            return
        if not self.connected:
            return
        self.recorder.start(OUTPUT_DIR)

    def _read_loop(self) -> None:
        conn = self.serial_conn
        try:
            while conn is not None and not self.stop_flag.is_set():
                raw = conn.readline()
                if not raw:
                    continue
                text = raw.decode("ascii", errors="ignore").strip()
                weight = parse_weight_line(raw)
                stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                with self.lock:
                    self.raw_text = text
                    if weight is not None:
                        self.weight = weight
                        self.sample_count += 1
                if weight is not None:
                    self.recorder.add(stamp, weight)
        except Exception as exc:
            if not self.stop_flag.is_set():
                self._set_status(f"串口中断：{exc}")
        finally:
            conn = self.serial_conn
            self.serial_conn = None
            if conn is not None:
                try:
                    if conn.is_open:
                        conn.close()
                except Exception:
                    pass
            self.connected = False
            self.recorder.stop()
            self._closed_pending = True
            if self.stop_flag.is_set():
                self._set_status("已关闭")

    def _on_close(self) -> None:
        self.recorder.stop()
        thread = self.recorder._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self.stop_flag.set()
        conn = self.serial_conn
        if conn is not None and conn.is_open:
            try:
                conn.close()
            except Exception:
                pass
        self.destroy()

    def _refresh(self) -> None:
        if not self.winfo_exists():
            return
        with self.lock:
            status = self.status_text
            weight = self.weight
            raw_text = self.raw_text
            sample_count = self.sample_count
        recording = self.recorder.active
        recorded = self.recorder.count
        record_note = self.recorder.note
        connected = self.connected

        if status != self._status_shown:
            self._status_shown = status
            self.status.configure(text=status)
        shown_weight = "--" if weight is None else f"{weight:.3f}"
        if self.weight_label.cget("text") != shown_weight:
            self.weight_label.configure(text=shown_weight)
        shown_raw = raw_text or "等待数据"
        if self.raw_label.cget("text") != shown_raw:
            self.raw_label.configure(text=shown_raw)

        summary = f"采样 {sample_count}"
        if recording:
            summary += f"    记录 {recorded} 条"
        elif record_note:
            summary += f"    {record_note}"
        if summary != self._summary_shown:
            self._summary_shown = summary
            self.summary.configure(text=summary)

        if self._closed_pending:
            self._closed_pending = False
            self.open_btn.configure(state="normal")
            self.close_btn.configure(state="disabled")
            self.port_combo.configure(state="readonly")
            self.baud_combo.configure(state="readonly")

        if connected and recording:
            record_shown = ("normal", "停止")
        elif connected:
            record_shown = ("normal", "记录")
        else:
            record_shown = ("disabled", "记录")
        if record_shown != self._record_btn_shown:
            self._record_btn_shown = record_shown
            self.record_btn.configure(state=record_shown[0], text=record_shown[1])
        self.after(UI_INTERVAL_MS, self._refresh)


def main() -> None:
    try:
        from ctypes import windll

        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass
    ScaleView().mainloop()


if __name__ == "__main__":
    main()
