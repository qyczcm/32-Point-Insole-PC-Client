"""连接鞋垫蓝牙网关，每秒打印一次最新的 32 点压力值。

鞋垫串口帧由 CH592 校验后，从 FFE4 原样 Notify：
    AA 55 | ch0..ch31 各 uint16 小端 | XOR(偏移 2..65)
同一特征上还会出现 11 字节状态包 D5 5D，这里只计数，不当作压力帧。
"""

import argparse
import asyncio
import sys
from datetime import datetime

try:
    from bleak import BleakClient, BleakScanner
    from bleak.exc import BleakError
except ImportError:
    print("缺少 bleak。请先执行: pip install bleak", file=sys.stderr)
    raise SystemExit(1)

NOTIFY_UUID = "0000ffe4-0000-1000-8000-00805f9b34fb"
FRAME_LEN = 67
FRAME_HEADER = b"\xAA\x55"
STATUS_HEADER = b"\xD5\x5D"
MIN_MTU = 70


def xor_ok(buf: bytes) -> bool:
    checksum = 0
    for byte in buf[2:66]:
        checksum ^= byte
    return checksum == buf[66]


def parse_frame(buf: bytes) -> list[int]:
    return [buf[2 + i * 2] | (buf[3 + i * 2] << 8) for i in range(32)]


class FrameStats:
    def __init__(self) -> None:
        self.latest: list[int] | None = None
        self.latest_at: datetime | None = None
        self.frames_this_second = 0
        self.frames_total = 0
        self.xor_fail = 0
        self.status_count = 0
        self.other_count = 0
        self.battery: int | None = None
        self.charging: int | None = None
        self._printed_first = False

    def on_notify(self, data: bytes) -> None:
        if len(data) == FRAME_LEN and data[:2] == FRAME_HEADER:
            if not xor_ok(data):
                self.xor_fail += 1
                return
            self.latest = parse_frame(data)
            self.latest_at = datetime.now()
            self.frames_this_second += 1
            self.frames_total += 1
            if not self._printed_first:
                self._printed_first = True
                print(f"首帧 HEX: {data.hex(' ').upper()}")
            return

        if len(data) >= 2 and data[:2] == STATUS_HEADER:
            self.status_count += 1
            if len(data) >= 7:
                self.battery = data[4]
                self.charging = data[6]
            return

        self.other_count += 1

    def take_rate(self) -> int:
        count = self.frames_this_second
        self.frames_this_second = 0
        return count


def format_row(values: list[int], start: int) -> str:
    cells = "  ".join(f"ch{start + i:02d}={values[start + i]:4d}" for i in range(8))
    return cells


def device_name(device, adv) -> str:
    return device.name or getattr(adv, "local_name", None) or ""


def device_rssi(device, adv) -> int | None:
    rssi = getattr(device, "rssi", None)
    if rssi is None:
        rssi = getattr(adv, "rssi", None)
    return rssi


async def find_device(keyword: str, address: str, timeout: float):
    print(f"扫描 {timeout:.0f} 秒，关键字 {keyword!r} ...")
    found = await BleakScanner.discover(timeout=timeout, return_adv=True)
    matches = []
    address = address.upper()

    for adv_address, (device, adv) in found.items():
        name = device_name(device, adv)
        rssi = device_rssi(device, adv)
        if address and adv_address.upper() != address:
            continue
        if address or keyword.lower() in name.lower():
            matches.append((name or "(无名称)", device, rssi))

    if not matches:
        print(f"未发现设备。附近共 {len(found)} 个广播。")
        return None

    print(f"匹配到 {len(matches)} 个设备：")
    for name, device, rssi in matches:
        rssi_text = f"{rssi} dBm" if rssi is not None else "RSSI 未知"
        print(f"  {name}  {device.address}  {rssi_text}")

    if len(matches) > 1 and not address:
        print("有多台设备。请用 --address 指定其中一台。")
        return None

    return matches[0]


async def run(keyword: str, address: str, scan_timeout: float, seconds: float) -> None:
    match = await find_device(keyword, address, scan_timeout)
    if match is None:
        return

    name, device, _rssi = match
    stats = FrameStats()
    print(f"连接 {name}  {device.address}")

    async with BleakClient(device) as client:
        if not client.is_connected:
            print("连接失败。")
            return

        mtu = getattr(client, "mtu_size", None)
        print(f"已连接。MTU = {mtu if mtu is not None else '未知'}")
        if mtu is not None and mtu < MIN_MTU:
            print(f"MTU {mtu} 小于 {MIN_MTU}，67 字节压力帧发不出来。")

        def on_notify(_sender, data: bytearray) -> None:
            stats.on_notify(bytes(data))

        await client.start_notify(NOTIFY_UUID, on_notify)
        print(f"已订阅 {NOTIFY_UUID}")
        print("每秒打印最新一帧。Ctrl+C 结束。")

        started = asyncio.get_running_loop().time()
        while True:
            await asyncio.sleep(1.0)
            rate = stats.take_rate()
            now = datetime.now().strftime("%H:%M:%S")
            extra = ""
            if stats.battery is not None:
                charging = "充电" if stats.charging else "未充电"
                extra = f"  电量={stats.battery} {charging}"
            print(
                f"[{now}] {rate:5.1f} 帧/秒  累计={stats.frames_total}"
                f"  XOR失败={stats.xor_fail}  状态包={stats.status_count}"
                f"  其他={stats.other_count}{extra}"
            )
            if stats.latest is None:
                print("  尚无有效压力帧")
            else:
                values = stats.latest
                for start in (0, 8, 16, 24):
                    print(f"  {format_row(values, start)}")
                peak = max(range(32), key=values.__getitem__)
                print(f"  总和={sum(values)}  最大=ch{peak:02d}={values[peak]}")

            if seconds > 0 and asyncio.get_running_loop().time() - started >= seconds:
                break

        await client.stop_notify(NOTIFY_UUID)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="打印鞋垫蓝牙网关转发的 32 点压力")
    parser.add_argument("--keyword", default="SENINSOLE", help="广播名关键字，默认 SENINSOLE")
    parser.add_argument("--address", default="", help="指定设备地址时跳过重名选择")
    parser.add_argument("--timeout", type=float, default=5.0, help="扫描秒数，默认 5")
    parser.add_argument("--seconds", type=float, default=0.0, help="打印多久后退出，0 表示一直打印")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        asyncio.run(run(args.keyword, args.address.strip(), args.timeout, args.seconds))
    except KeyboardInterrupt:
        print("\n已停止。")
    except BleakError as exc:
        print(f"蓝牙失败：{exc}", file=sys.stderr)
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
