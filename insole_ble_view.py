"""实时显示鞋垫蓝牙网关转发的 32 点压力。

接收与 insole_ble_print.py 相同：订阅 FFE4，只保留最新一帧合法的
AA 55 压力帧。界面约每 30 ms 刷新，颜色按原始 ADC 0～4095 从浅到深。
"""

import asyncio
import csv
import queue
import sys
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from pathlib import Path
from tkinter import ttk

try:
    from bleak import BleakClient, BleakScanner
    from bleak.exc import BleakError
except ImportError:
    print("缺少 bleak。请先执行: pip install bleak", file=sys.stderr)
    raise SystemExit(1)

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

BAUDRATES = ("9600", "4800", "19200", "38400", "57600", "115200")

from insole_ble_print import (
    FRAME_HEADER,
    FRAME_LEN,
    MIN_MTU,
    NOTIFY_UUID,
    STATUS_HEADER,
    parse_frame,
    xor_ok,
)

SCAN_TIMEOUT = 12.0
UI_INTERVAL_MS = 30
ESTIMATE_WINDOW_S = 0.5
FOOT_SCALE = 0.82
FOOT_PAD = 36
FOOT_RADIUS = 21
# 装配图蓝色编号 1～32。坐标仍是原来的传感器中心，脚尖在上，y 向下。编号减 1 是 ch 序号。
FOOT_POINTS = (
    (1, 77.9, 563.5),
    (2, 76.8, 624.0),
    (3, 134.9, 614.5),
    (4, 110.6, 674.6),
    (5, 175.7, 659.9),
    (6, 191.5, 594.1),
    (7, 134.0, 549.6),
    (8, 191.3, 528.4),
    (9, 198.8, 468.0),
    (10, 208.5, 406.7),
    (11, 224.3, 347.3),
    (12, 163.2, 353.5),
    (13, 233.4, 288.3),
    (14, 174.0, 287.4),
    (15, 237.1, 227.0),
    (16, 175.2, 225.3),
    (32, 226.1, 168.7),
    (31, 167.7, 161.3),
    (30, 205.7, 111.8),
    (29, 166.1, 61.2),
    (28, 110.1, 40.8),
    (27, 125.2, 115.9),
    (26, 67.9, 96.0),
    (25, 111.2, 187.3),
    (24, 55.8, 157.6),
    (23, 113.5, 249.4),
    (22, 54.9, 220.5),
    (21, 117.9, 312.4),
    (20, 56.8, 284.6),
    (19, 61.5, 345.8),
    (18, 137.2, 408.5),
    (17, 66.0, 405.9),
)
FOOT_OUTLINE = (28, 26, 24, 22, 20, 19, 17, 1, 2, 4, 5, 6, 8, 9, 10, 11, 13, 15, 32, 30, 29)

# 五个区域。点号是装配图蓝色编号，通道序号是点号减 1。
REGIONS = (
    ("脚尖", (26, 27, 28, 29)),
    ("前掌内", (20, 21, 22, 23, 24, 25)),
    ("前掌外", (13, 14, 15, 16, 30, 31, 32)),
    ("足弓", (9, 10, 11, 12, 17, 18, 19)),
    ("脚跟", (1, 2, 3, 4, 5, 6, 7, 8)),
)
# 鞋垫直接放在秤上、14～45 kg 稳住样本，按上面五个区里的点做非负拟合。空载底数取第一次近 0 kg 段。
BASELINE = (
    210.47, 198.21, 199.51, 196.92, 196.70, 198.88, 200.01, 196.39,
    194.87, 198.14, 201.04, 195.57, 200.31, 197.70, 201.01, 204.12,
    202.45, 198.74, 197.77, 197.03, 192.71, 204.28, 203.22, 199.81,
    206.29, 204.88, 198.10, 203.30, 199.12, 196.08, 199.18, 195.37,
)
CHANNEL_GAINS = (
    0.0022434934, 0.0, 0.0, 0.0, 0.0, 0.0072407993, 0.0009621994, 0.0,
    0.0048243232, 0.0, 0.0, 0.0, 0.0064035806, 0.0, 0.0, 0.0,
    0.0, 0.0023829758, 0.0, 0.0052357255, 0.0086121657, 0.0, 0.0, 0.0047295760,
    0.0, 0.0, 0.0095982758, 0.0, 0.0, 0.0024642233, 0.0013027092, 0.0,
)
WEIGHT_INTERCEPT = -0.11061102


OUTPUT_DIR = Path(__file__).resolve().parent / "data"


def region_loads(channels: list[float]) -> list[float]:
    """每个区的负荷：区内各点高出空载的部分乘以该点的非负灵敏度。"""
    loads = []
    for _name, members in REGIONS:
        load = 0.0
        for number in members:
            index = number - 1
            gain = CHANNEL_GAINS[index]
            if gain:
                load += gain * max(0.0, channels[index] - BASELINE[index])
        loads.append(load)
    return loads


def estimate_weight(channels: list[float]) -> float:
    """半秒平均后，五个区的负荷相加，再加标定常数。"""
    return WEIGHT_INTERCEPT + sum(region_loads(channels))


class FrameRecorder:
    """把每一帧解析结果追加到 CSV。蓝牙回调只入队，写盘在单独线程。"""

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
        path = directory / f"insole_{datetime.now():%Y%m%d_%H%M%S}.csv"
        with self._lock:
            self.path = path
            self.count = 0
            self.note = f"记录中 0 帧  {path.name}"
            self._queue = queue.Queue()
            self.active = True
            self._thread = threading.Thread(target=self._loop, args=(path, self._queue), daemon=True)
            self._thread.start()
        print(f"开始记录 {path}", flush=True)
        return path

    def add(self, stamp: str, values: list[int]) -> None:
        with self._lock:
            if not self.active or self._queue is None:
                return
            self._queue.put((stamp, values))
            self.count += 1

    def stop(self) -> None:
        with self._lock:
            if not self.active:
                return
            self.active = False
            pending = self._queue
            if self.count:
                self.note = f"正在保存 {self.count} 帧..."
        if pending is not None:
            pending.put(None)

    def _loop(self, path: Path, pending: queue.Queue) -> None:
        saved = 0
        try:
            with path.open("w", newline="", encoding="utf-8-sig") as handle:
                writer = csv.writer(handle)
                writer.writerow(["时间", *[f"ch{index:02d}" for index in range(32)]])
                last_flush = time.monotonic()
                while True:
                    item = pending.get()
                    if item is None:
                        break
                    stamp, values = item
                    writer.writerow([stamp, *values])
                    saved += 1
                    now = time.monotonic()
                    if now - last_flush >= 1.0:
                        handle.flush()
                        last_flush = now
                handle.flush()
            self.count = saved
            self.note = f"已保存 {saved} 帧  {path.name}"
        except Exception as exc:
            self.active = False
            self.note = f"记录失败：{exc}"
        print(self.note, flush=True)


def parse_weight_line(raw: bytes) -> float | None:
    text = raw.decode("ascii", errors="ignore").strip()
    if not text:
        return None
    try:
        return float(text.split(",", 1)[0].strip())
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


def cell_colors(value: int | None) -> tuple[str, str]:
    if value is None:
        return "#f3f3f3", "#9a9a9a"
    span = max(0.0, min(1.0, value / 4095.0))
    red = int(246 + (168 - 246) * span)
    green = int(246 + (28 - 246) * span)
    blue = int(246 + (28 - 246) * span)
    foreground = "#ffffff" if span > 0.62 else "#1c1c1c"
    return f"#{red:02x}{green:02x}{blue:02x}", foreground


class InsoleView(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("32点鞋垫 / 体重秤")
        self.resizable(False, False)
        self.configure(bg="#f7f7f7")

        self.lock = threading.Lock()
        self.stop_flag = threading.Event()
        self.worker: threading.Thread | None = None
        self.values: list[int] | None = None
        self._estimate_frames: deque[tuple[float, list[int]]] = deque()
        self.estimate_kg: float | None = None
        self.region_kg: list[float] | None = None
        self.fps = 0.0
        self.frame_total = 0
        self.xor_fail = 0
        self.status_text = "未连接"
        self.battery: int | None = None
        self.charging: int | None = None
        self._rate_count = 0
        self._rate_started = time.monotonic()

        self.keyword = tk.StringVar(value="SENINSOLE")
        self._cells: list[tuple[int, int, int]] = []
        self._shown: list[tuple[int | None, bool]] = [(None, False)] * 32
        self._status_shown = ""
        self._weight_shown = ""
        self._region_shown = ""
        self._summary_shown = ""
        self._buttons_live: bool | None = None
        self._record_btn_shown: tuple[str, str] | None = None
        self.connected = False
        self.recorder = FrameRecorder()

        self.scale_lock = threading.Lock()
        self.scale_stop = threading.Event()
        self.scale_worker: threading.Thread | None = None
        self.scale_conn = None
        self.scale_connected = False
        self._scale_closed = False
        self.scale_weight: float | None = None
        self.scale_raw = ""
        self.scale_count = 0
        self.scale_status = "未连接" if serial is not None else "缺少 pyserial"
        self.scale_recorder = WeightRecorder()
        self._scale_status_shown = ""
        self._scale_weight_shown = ""
        self._scale_raw_shown = ""
        self._scale_summary_shown = ""
        self._scale_record_shown: tuple[str, str] | None = None
        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="9600")
        self._ports: dict[str, str] = {}

        self._build()
        self._refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(UI_INTERVAL_MS, self._refresh)
        self.after(200, self._connect)

    def _foot_xy(self, px: float, py: float) -> tuple[float, float]:
        return FOOT_PAD + px * FOOT_SCALE, FOOT_PAD + py * FOOT_SCALE

    def _build(self) -> None:
        body = tk.Frame(self, bg="#f7f7f7")
        body.pack(fill="both", expand=True)

        side = tk.Frame(body, bg="#f7f7f7", width=236)
        side.pack(side="left", fill="y", padx=(12, 0), pady=12)
        side.pack_propagate(False)

        tk.Label(side, text="关键字", bg="#f7f7f7").pack(anchor="w")
        ttk.Entry(side, textvariable=self.keyword, width=16).pack(anchor="w", pady=(4, 8))
        buttons = tk.Frame(side, bg="#f7f7f7")
        buttons.pack(anchor="w")
        self.connect_btn = ttk.Button(buttons, text="连接", command=self._connect)
        self.connect_btn.pack(side="left")
        self.disconnect_btn = ttk.Button(buttons, text="断开", command=self._disconnect, state="disabled")
        self.disconnect_btn.pack(side="left", padx=(6, 0))
        self.record_btn = ttk.Button(buttons, text="记录", command=self._toggle_record, state="disabled")
        self.record_btn.pack(side="left", padx=(6, 0))

        self.status = tk.Label(
            side, text=self.status_text, anchor="w", justify="left", wraplength=220,
            bg="#f7f7f7", fg="#333333",
        )
        self.status.pack(fill="x", pady=(10, 8))

        tk.Label(side, text="估算重量", bg="#f7f7f7", fg="#666666", font=("Microsoft YaHei UI", 12)).pack(anchor="w")
        meter = tk.Frame(side, bg="#f7f7f7")
        meter.pack(anchor="w", pady=(2, 8))
        self.weight_label = tk.Label(
            meter, text="--", bg="#f7f7f7", fg="#1c1c1c", font=("Consolas", 28, "bold")
        )
        self.weight_label.pack(side="left")
        tk.Label(meter, text="kg", bg="#f7f7f7", fg="#666666", font=("Microsoft YaHei UI", 12)).pack(side="left", padx=(6, 0))
        self.region_label = tk.Label(
            side,
            text="\n".join(f"{name} --" for name, _members in REGIONS),
            anchor="w", justify="left", bg="#f7f7f7", fg="#333333",
        )
        self.region_label.pack(anchor="w", pady=(0, 8))

        tk.Label(side, text="脚尖在上", bg="#f7f7f7", fg="#888888").pack(anchor="w", pady=(8, 0))
        self.summary = tk.Label(
            side, text="总和 --\n最大 --\n帧率 --\nXOR失败 0",
            anchor="w", justify="left", bg="#f7f7f7",
        )
        self.summary.pack(fill="x", pady=(8, 0))

        points = {number: (px, py) for number, px, py in FOOT_POINTS}
        width = int(FOOT_PAD * 2 + 274 * FOOT_SCALE)
        height = int(FOOT_PAD * 2 + 725 * FOOT_SCALE)
        self.canvas = tk.Canvas(
            body, width=width, height=height, bg="#f7f7f7", highlightthickness=0, bd=0,
        )

        center_x = sum(px for px, _py in points.values()) / len(points)
        center_y = sum(py for _px, py in points.values()) / len(points)
        outline: list[float] = []
        for number in FOOT_OUTLINE:
            px, py = points[number]
            dx, dy = px - center_x, py - center_y
            norm = (dx * dx + dy * dy) ** 0.5 or 1.0
            outline.extend(self._foot_xy(px + dx / norm * 28, py + dy / norm * 28))
        self.canvas.create_polygon(*outline, outline="#e7b6ea", fill="#fff7fb", smooth=True, width=2)

        placed = [None] * 32
        radius = FOOT_RADIUS
        for number, px, py in FOOT_POINTS:
            cx, cy = self._foot_xy(px, py)
            oval = self.canvas.create_oval(
                cx - radius, cy - radius, cx + radius, cy + radius,
                fill="#f3f3f3", outline="#d0d0d0", width=2,
            )
            name = self.canvas.create_text(
                cx, cy - 7, text=str(number), font=("Microsoft YaHei UI", 8), fill="#9a9a9a",
            )
            value = self.canvas.create_text(
                cx, cy + 8, text="--", font=("Consolas", 8, "bold"), fill="#9a9a9a",
            )
            placed[number - 1] = (oval, name, value)
        self._cells = placed

        scale = tk.Frame(body, bg="#f7f7f7", width=268)
        scale.pack(side="right", fill="y", padx=(0, 12), pady=12)
        scale.pack_propagate(False)
        self.canvas.pack(side="left", padx=(8, 8), pady=12)
        tk.Label(scale, text="体重秤", bg="#f7f7f7", font=("Microsoft YaHei UI", 14, "bold")).pack(anchor="w")

        port_row = tk.Frame(scale, bg="#f7f7f7")
        port_row.pack(fill="x", pady=(10, 4))
        tk.Label(port_row, text="串口", bg="#f7f7f7").pack(side="left")
        self.port_combo = ttk.Combobox(port_row, textvariable=self.port_var, width=22, state="readonly")
        self.port_combo.pack(side="left", padx=(6, 0))
        ttk.Button(scale, text="刷新串口", command=self._refresh_ports).pack(anchor="w", pady=(0, 6))

        baud_row = tk.Frame(scale, bg="#f7f7f7")
        baud_row.pack(fill="x", pady=(0, 8))
        tk.Label(baud_row, text="波特率", bg="#f7f7f7").pack(side="left")
        self.baud_combo = ttk.Combobox(
            baud_row, textvariable=self.baud_var, width=8, values=BAUDRATES, state="readonly"
        )
        self.baud_combo.pack(side="left", padx=(6, 0))

        scale_buttons = tk.Frame(scale, bg="#f7f7f7")
        scale_buttons.pack(anchor="w")
        self.scale_open_btn = ttk.Button(scale_buttons, text="打开", command=self._scale_open)
        self.scale_open_btn.pack(side="left")
        self.scale_close_btn = ttk.Button(scale_buttons, text="关闭", command=self._scale_close, state="disabled")
        self.scale_close_btn.pack(side="left", padx=(6, 0))
        self.scale_record_btn = ttk.Button(scale_buttons, text="记录", command=self._scale_toggle_record, state="disabled")
        self.scale_record_btn.pack(side="left", padx=(6, 0))
        if serial is None:
            self.scale_open_btn.configure(state="disabled")

        self.scale_status_label = tk.Label(
            scale, text=self.scale_status, anchor="w", justify="left", wraplength=250,
            bg="#f7f7f7", fg="#333333",
        )
        self.scale_status_label.pack(fill="x", pady=(10, 8))
        tk.Label(scale, text="秤上重量", bg="#f7f7f7", fg="#666666", font=("Microsoft YaHei UI", 12)).pack(anchor="w")
        scale_meter = tk.Frame(scale, bg="#f7f7f7")
        scale_meter.pack(anchor="w", pady=(2, 6))
        self.scale_weight_label = tk.Label(
            scale_meter, text="--", bg="#f7f7f7", fg="#1c1c1c", font=("Consolas", 28, "bold")
        )
        self.scale_weight_label.pack(side="left")
        tk.Label(scale_meter, text="kg", bg="#f7f7f7", fg="#666666", font=("Microsoft YaHei UI", 12)).pack(
            side="left", padx=(6, 0)
        )
        self.scale_raw_label = tk.Label(
            scale, text="等待数据", anchor="w", bg="#f7f7f7", fg="#666666", font=("Consolas", 10)
        )
        self.scale_raw_label.pack(fill="x", pady=(8, 0))
        self.scale_summary = tk.Label(scale, text="采样 0", anchor="w", justify="left", bg="#f7f7f7")
        self.scale_summary.pack(fill="x", pady=(8, 0))

    def _set_status(self, text: str) -> None:
        with self.lock:
            self.status_text = text
        print(text, flush=True)

    def _connect(self) -> None:
        if self.worker and self.worker.is_alive():
            return
        self.stop_flag.clear()
        keyword = self.keyword.get().strip() or "SENINSOLE"
        self.connect_btn.configure(state="disabled")
        self.disconnect_btn.configure(state="normal")
        self.worker = threading.Thread(target=self._ble_thread, args=(keyword,), daemon=True)
        self.worker.start()

    def _disconnect(self) -> None:
        self.recorder.stop()
        self.stop_flag.set()
        self._set_status("正在断开...")

    def _toggle_record(self) -> None:
        if self.recorder.active:
            self.recorder.stop()
            return
        if not self.connected:
            return
        self.recorder.start(OUTPUT_DIR)

    def _refresh_ports(self) -> None:
        if serial is None or self.scale_connected:
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
        if self.port_var.get() not in labels:
            self.port_var.set(labels[0] if labels else "")

    def _selected_port(self) -> str:
        label = self.port_var.get().strip()
        return self._ports.get(label, label.split(":", 1)[0].strip())

    def _set_scale_status(self, text: str) -> None:
        with self.scale_lock:
            self.scale_status = text
        print(text, flush=True)

    def _scale_open(self) -> None:
        if serial is None or (self.scale_worker and self.scale_worker.is_alive()):
            return
        port = self._selected_port()
        if not port:
            self._set_scale_status("没有可用串口")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            self._set_scale_status("波特率无效")
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
            self._set_scale_status(f"打开失败：{exc}")
            return
        self.scale_conn = conn
        self.scale_stop.clear()
        with self.scale_lock:
            self.scale_count = 0
            self.scale_weight = None
            self.scale_raw = ""
        self.scale_connected = True
        self.scale_open_btn.configure(state="disabled")
        self.scale_close_btn.configure(state="normal")
        self.port_combo.configure(state="disabled")
        self.baud_combo.configure(state="disabled")
        self._set_scale_status(f"已打开 {port}  {baud} 8N1")
        self.scale_worker = threading.Thread(target=self._scale_read_loop, daemon=True)
        self.scale_worker.start()

    def _scale_close(self) -> None:
        self.scale_recorder.stop()
        self.scale_stop.set()
        self._set_scale_status("正在关闭...")

    def _scale_toggle_record(self) -> None:
        if self.scale_recorder.active:
            self.scale_recorder.stop()
            return
        if not self.scale_connected:
            return
        self.scale_recorder.start(OUTPUT_DIR)

    def _scale_read_loop(self) -> None:
        conn = self.scale_conn
        try:
            while conn is not None and not self.scale_stop.is_set():
                raw = conn.readline()
                if not raw:
                    continue
                text = raw.decode("ascii", errors="ignore").strip()
                weight = parse_weight_line(raw)
                stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                with self.scale_lock:
                    self.scale_raw = text
                    if weight is not None:
                        self.scale_weight = weight
                        self.scale_count += 1
                if weight is not None:
                    self.scale_recorder.add(stamp, weight)
        except Exception as exc:
            if not self.scale_stop.is_set():
                self._set_scale_status(f"串口中断：{exc}")
        finally:
            conn = self.scale_conn
            self.scale_conn = None
            if conn is not None:
                try:
                    if conn.is_open:
                        conn.close()
                except Exception:
                    pass
            self.scale_connected = False
            self.scale_recorder.stop()
            self._scale_closed = True
            if self.scale_stop.is_set():
                self._set_scale_status("已关闭")

    def _on_close(self) -> None:
        self.recorder.stop()
        thread = self.recorder._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self.scale_recorder.stop()
        scale_thread = self.scale_recorder._thread
        if scale_thread is not None and scale_thread.is_alive():
            scale_thread.join(timeout=2.0)
        self.stop_flag.set()
        self.scale_stop.set()
        conn = self.scale_conn
        if conn is not None and conn.is_open:
            try:
                conn.close()
            except Exception:
                pass
        self.destroy()

    def _ble_thread(self, keyword: str) -> None:
        try:
            asyncio.run(self._ble_main(keyword))
        except Exception as exc:
            self._set_status(f"蓝牙失败：{exc}")

    async def _find_device(self, keyword: str):
        self._set_status(f"正在扫描 {keyword} ...")
        loop = asyncio.get_running_loop()
        found: asyncio.Future = loop.create_future()

        def on_advert(device, adv) -> None:
            name = device.name or adv.local_name or ""
            if keyword.lower() in name.lower() and not found.done():
                found.set_result((name, device))

        scanner = BleakScanner(detection_callback=on_advert)
        await scanner.start()
        try:
            name, device = await asyncio.wait_for(found, SCAN_TIMEOUT)
        except TimeoutError:
            return None
        finally:
            await scanner.stop()
        if self.stop_flag.is_set():
            return None
        await asyncio.sleep(0.3)
        return name, device

    def _on_notify(self, data: bytearray) -> None:
        raw = bytes(data)
        now = time.monotonic()
        with self.lock:
            if len(raw) == FRAME_LEN and raw[:2] == FRAME_HEADER:
                if not xor_ok(raw):
                    self.xor_fail += 1
                    return
                parsed = parse_frame(raw)
                self.values = parsed
                self._estimate_frames.append((now, parsed))
                cutoff = now - ESTIMATE_WINDOW_S
                while self._estimate_frames and self._estimate_frames[0][0] < cutoff:
                    self._estimate_frames.popleft()
                count = len(self._estimate_frames)
                means = [0.0] * 32
                for _stamp, frame in self._estimate_frames:
                    for index, value in enumerate(frame):
                        means[index] += value
                averaged = [value / count for value in means]
                self.region_kg = region_loads(averaged)
                self.estimate_kg = WEIGHT_INTERCEPT + sum(self.region_kg)
                self.frame_total += 1
                self._rate_count += 1
                elapsed = now - self._rate_started
                if elapsed >= 1.0:
                    self.fps = self._rate_count / elapsed
                    self._rate_count = 0
                    self._rate_started = now
            else:
                parsed = None
            if len(raw) >= 7 and raw[:2] == STATUS_HEADER:
                self.battery = raw[4]
                self.charging = raw[6]
        if parsed is not None:
            stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            self.recorder.add(stamp, parsed)

    async def _ble_main(self, keyword: str) -> None:
        match = await self._find_device(keyword)
        if match is None:
            self._set_status("已断开" if self.stop_flag.is_set() else f"未发现名称包含 {keyword} 的设备")
            return

        name, device = match
        self._set_status(f"正在连接 {name}  {device.address}")
        try:
            async with BleakClient(device) as client:
                if not client.is_connected:
                    self._set_status("连接失败")
                    return
                mtu = client.mtu_size
                note = f"已连接 {name}  {device.address}    MTU {mtu}"
                if mtu < MIN_MTU:
                    note += f"（低于 {MIN_MTU}，压力帧发不出来）"
                self._set_status(note)
                with self.lock:
                    self._rate_count = 0
                    self._rate_started = time.monotonic()
                    self.fps = 0.0
                    self.values = None
                    self._estimate_frames.clear()
                    self.estimate_kg = None
                    self.region_kg = None
                await client.start_notify(NOTIFY_UUID, lambda _sender, data: self._on_notify(data))
                self.connected = True
                while not self.stop_flag.is_set():
                    await asyncio.sleep(0.05)
                await client.stop_notify(NOTIFY_UUID)
        except BleakError as exc:
            self._set_status(f"蓝牙失败：{exc}")
        finally:
            self.connected = False
            self.recorder.stop()
        if self.stop_flag.is_set():
            self._set_status("已断开")

    def _refresh(self) -> None:
        if not self.winfo_exists():
            return
        with self.lock:
            values = None if self.values is None else list(self.values)
            status = self.status_text
            fps = self.fps
            xor_fail = self.xor_fail
            battery = self.battery
            charging = self.charging
            estimate_kg = self.estimate_kg
            region_kg = None if self.region_kg is None else list(self.region_kg)
            recording = self.recorder.active
            recorded = self.recorder.count
            record_note = self.recorder.note
            connected = self.connected

        if status != self._status_shown:
            self._status_shown = status
            self.status.configure(text=status)
        shown_weight = "--" if estimate_kg is None else f"{estimate_kg:.1f}"
        if shown_weight != self._weight_shown:
            self._weight_shown = shown_weight
            self.weight_label.configure(text=shown_weight)
        if region_kg is None:
            region_text = "\n".join(f"{name} --" for name, _members in REGIONS)
        else:
            region_text = "\n".join(
                f"{name} {load:.1f}" for (name, _members), load in zip(REGIONS, region_kg)
            )
        if region_text != self._region_shown:
            self._region_shown = region_text
            self.region_label.configure(text=region_text)
        peak = None
        if values:
            peak = max(range(32), key=values.__getitem__)
        for index, (rect, name, label) in enumerate(self._cells):
            value = None if values is None else values[index]
            is_peak = peak == index
            if self._shown[index] == (value, is_peak):
                continue
            self._shown[index] = (value, is_peak)
            background, foreground = cell_colors(value)
            self.canvas.itemconfig(rect, fill=background, outline="#1565c0" if is_peak else "#d0d0d0")
            self.canvas.itemconfig(name, fill=foreground)
            self.canvas.itemconfig(label, text="--" if value is None else str(value), fill=foreground)

        if values is None:
            summary = f"总和 --\n最大 --\n帧率 {fps:.0f}\nXOR失败 {xor_fail}"
        else:
            summary = (
                f"总和 {sum(values)}\n最大 点{peak + 1}={values[peak]}"
                f"\n帧率 {fps:.0f}\nXOR失败 {xor_fail}"
            )
        if battery is not None:
            summary += f"    电量 {battery} {'充电' if charging else '未充电'}"
        if recording:
            summary += f"    记录 {recorded} 帧"
        elif record_note:
            summary += f"    {record_note}"
        if summary != self._summary_shown:
            self._summary_shown = summary
            self.summary.configure(text=summary)
        alive = self.worker is not None and self.worker.is_alive()
        if alive != self._buttons_live:
            self._buttons_live = alive
            self.connect_btn.configure(state="disabled" if alive else "normal")
            self.disconnect_btn.configure(state="normal" if alive else "disabled")
        if connected and recording:
            record_shown = ("normal", "停止")
        elif connected:
            record_shown = ("normal", "记录")
        else:
            record_shown = ("disabled", "记录")
        if record_shown != self._record_btn_shown:
            self._record_btn_shown = record_shown
            self.record_btn.configure(state=record_shown[0], text=record_shown[1])

        with self.scale_lock:
            scale_status = self.scale_status
            scale_weight = self.scale_weight
            scale_raw = self.scale_raw
            scale_count = self.scale_count
        scale_recording = self.scale_recorder.active
        scale_recorded = self.scale_recorder.count
        scale_note = self.scale_recorder.note
        if scale_status != self._scale_status_shown:
            self._scale_status_shown = scale_status
            self.scale_status_label.configure(text=scale_status)
        shown_scale = "--" if scale_weight is None else f"{scale_weight:.3f}"
        if shown_scale != self._scale_weight_shown:
            self._scale_weight_shown = shown_scale
            self.scale_weight_label.configure(text=shown_scale)
        shown_raw = scale_raw or "等待数据"
        if shown_raw != self._scale_raw_shown:
            self._scale_raw_shown = shown_raw
            self.scale_raw_label.configure(text=shown_raw)
        scale_summary = f"采样 {scale_count}"
        if scale_recording:
            scale_summary += f"\n记录 {scale_recorded} 条"
        elif scale_note:
            scale_summary += f"\n{scale_note}"
        if scale_summary != self._scale_summary_shown:
            self._scale_summary_shown = scale_summary
            self.scale_summary.configure(text=scale_summary)
        if self._scale_closed:
            self._scale_closed = False
            self.scale_open_btn.configure(state="normal" if serial is not None else "disabled")
            self.scale_close_btn.configure(state="disabled")
            self.port_combo.configure(state="readonly")
            self.baud_combo.configure(state="readonly")
        if self.scale_connected and scale_recording:
            scale_record_shown = ("normal", "停止")
        elif self.scale_connected:
            scale_record_shown = ("normal", "记录")
        else:
            scale_record_shown = ("disabled", "记录")
        if scale_record_shown != self._scale_record_shown:
            self._scale_record_shown = scale_record_shown
            self.scale_record_btn.configure(state=scale_record_shown[0], text=scale_record_shown[1])
        self.after(UI_INTERVAL_MS, self._refresh)


def main() -> None:
    try:
        from ctypes import windll

        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass
    InsoleView().mainloop()


if __name__ == "__main__":
    main()
