"""实时显示鞋垫蓝牙网关转发的 32 点压力。

接收与 insole_ble_print.py 相同：订阅 FFE4，只保留最新一帧合法的
AA 55 压力帧。界面约每 30 ms 刷新，颜色按原始 ADC 0～4095 从浅到深。
"""

import asyncio
import sys
import threading
import time
import tkinter as tk
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
        self._summary_shown = ""
        self._buttons_live: bool | None = None
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

        self.status = tk.Label(self, text=self.status_text, anchor="w", bg="#f7f7f7", fg="#333333")
        self.status.pack(fill="x", padx=12, pady=(0, 8))

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
        self.stop_flag.set()
        self._set_status("正在断开...")

    def _on_close(self) -> None:
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
                self.values = parse_frame(raw)
                self.frame_total += 1
                self._rate_count += 1
                elapsed = now - self._rate_started
                if elapsed >= 1.0:
                    self.fps = self._rate_count / elapsed
                    self._rate_count = 0
                    self._rate_started = now
                return
            if len(raw) >= 7 and raw[:2] == STATUS_HEADER:
                self.battery = raw[4]
                self.charging = raw[6]

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
                await client.start_notify(NOTIFY_UUID, lambda _sender, data: self._on_notify(data))
                while not self.stop_flag.is_set():
                    await asyncio.sleep(0.05)
                await client.stop_notify(NOTIFY_UUID)
        except BleakError as exc:
            self._set_status(f"蓝牙失败：{exc}")
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

        if status != self._status_shown:
            self._status_shown = status
            self.status.configure(text=status)
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
        if summary != self._summary_shown:
            self._summary_shown = summary
            self.summary.configure(text=summary)
        alive = self.worker is not None and self.worker.is_alive()
        if alive != self._buttons_live:
            self._buttons_live = alive
            self.connect_btn.configure(state="disabled" if alive else "normal")
            self.disconnect_btn.configure(state="normal" if alive else "disabled")
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
