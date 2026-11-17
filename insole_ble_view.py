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
CELL_W = 108
CELL_H = 68
CELL_GAP = 6
ESTIMATE_WINDOW_S = 0.5

# 2026-11-17 上午三次直接放在秤上的记录一起拟合。空载底数取第一次开头的空放段。
# 各点系数不小于 0。
BASELINE = (
    207.66, 197.03, 196.50, 198.59, 199.97, 198.53, 202.41, 201.97,
    197.47, 199.00, 201.66, 196.69, 197.19, 197.31, 207.06, 196.22,
    205.78, 205.94, 196.91, 198.62, 195.97, 197.44, 196.97, 196.78,
    203.38, 203.66, 198.00, 209.38, 200.38, 197.12, 198.97, 193.41,
)
CHANNEL_GAINS = (
    0.0035096618, 0.0, 0.0, 0.0, 0.0, 0.0139557928, 0.0, 0.0,
    0.0006424328, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0030167519, 0.0, 0.0218251896, 0.0060995795, 0.0, 0.0, 0.0011305142,
    0.0, 0.0038674507, 0.0031887420, 0.0, 0.0, 0.0024911169, 0.0, 0.0,
)
WEIGHT_INTERCEPT = -0.08923090


OUTPUT_DIR = Path(__file__).resolve().parent / "data"


def estimate_weight(channels: list[float]) -> float:
    """半秒平均后的 32 点，各点高出空载的部分按三次标定得到的非负灵敏度相加。"""
    estimate = WEIGHT_INTERCEPT
    for value, base, gain in zip(channels, BASELINE, CHANNEL_GAINS):
        if gain:
            estimate += gain * max(0.0, value - base)
    return estimate


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
        self.title("32点鞋垫")
        self.resizable(False, False)
        self.configure(bg="#f7f7f7")

        self.lock = threading.Lock()
        self.stop_flag = threading.Event()
        self.worker: threading.Thread | None = None
        self.values: list[int] | None = None
        self._estimate_frames: deque[tuple[float, list[int]]] = deque()
        self.estimate_kg: float | None = None
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
        self._summary_shown = ""
        self._buttons_live: bool | None = None
        self._record_btn_shown: tuple[str, str] | None = None
        self.connected = False
        self.recorder = FrameRecorder()
        self._build()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(UI_INTERVAL_MS, self._refresh)
        self.after(200, self._connect)

    def _build(self) -> None:
        top = tk.Frame(self, bg="#f7f7f7")
        top.pack(fill="x", padx=12, pady=(12, 6))
        tk.Label(top, text="关键字", bg="#f7f7f7").pack(side="left")
        ttk.Entry(top, textvariable=self.keyword, width=16).pack(side="left", padx=(6, 10))
        self.connect_btn = ttk.Button(top, text="连接", command=self._connect)
        self.connect_btn.pack(side="left")
        self.disconnect_btn = ttk.Button(top, text="断开", command=self._disconnect, state="disabled")
        self.disconnect_btn.pack(side="left", padx=(6, 0))
        self.record_btn = ttk.Button(top, text="记录", command=self._toggle_record, state="disabled")
        self.record_btn.pack(side="left", padx=(6, 0))

        self.status = tk.Label(self, text=self.status_text, anchor="w", bg="#f7f7f7", fg="#333333")
        self.status.pack(fill="x", padx=12, pady=(0, 4))

        meter = tk.Frame(self, bg="#f7f7f7")
        meter.pack(fill="x", padx=12, pady=(0, 6))
        tk.Label(
            meter, text="估算重量", bg="#f7f7f7", fg="#666666", font=("Microsoft YaHei UI", 12)
        ).pack(side="left")
        self.weight_label = tk.Label(
            meter, text="--", bg="#f7f7f7", fg="#1c1c1c", font=("Consolas", 28, "bold")
        )
        self.weight_label.pack(side="left", padx=(12, 6))
        tk.Label(meter, text="kg", bg="#f7f7f7", fg="#666666", font=("Microsoft YaHei UI", 12)).pack(side="left")

        width = 8 * CELL_W + 7 * CELL_GAP
        height = 4 * CELL_H + 3 * CELL_GAP
        self.canvas = tk.Canvas(
            self,
            width=width,
            height=height,
            bg="#f7f7f7",
            highlightthickness=0,
            bd=0,
        )
        self.canvas.pack(padx=12, pady=4)
        for index in range(32):
            row, col = divmod(index, 8)
            x = col * (CELL_W + CELL_GAP)
            y = row * (CELL_H + CELL_GAP)
            rect = self.canvas.create_rectangle(
                x, y, x + CELL_W, y + CELL_H,
                fill="#f3f3f3", outline="#d0d0d0", width=2,
            )
            name = self.canvas.create_text(
                x + CELL_W / 2, y + 20,
                text=f"ch{index:02d}",
                font=("Microsoft YaHei UI", 9),
                fill="#9a9a9a",
            )
            value = self.canvas.create_text(
                x + CELL_W / 2, y + 44,
                text="--",
                font=("Consolas", 16, "bold"),
                fill="#9a9a9a",
            )
            self._cells.append((rect, name, value))

        self.summary = tk.Label(self, text="总和 --    最大 --    帧率 --    XOR失败 0", anchor="w", bg="#f7f7f7")
        self.summary.pack(fill="x", padx=12, pady=(8, 12))

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

    def _on_close(self) -> None:
        self.recorder.stop()
        thread = self.recorder._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self.stop_flag.set()
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
                self.estimate_kg = estimate_weight([value / count for value in means])
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
            summary = f"总和 --    最大 --    帧率 {fps:.0f}    XOR失败 {xor_fail}"
        else:
            summary = (
                f"总和 {sum(values)}    最大 ch{peak:02d}={values[peak]}"
                f"    帧率 {fps:.0f}    XOR失败 {xor_fail}"
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
