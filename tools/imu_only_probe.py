"""ImuOnly 虚拟下位机：创建 PTY，读取上位机下发的串口报文并解码波形。

报文格式（packet.hpp，#pragma pack(1)）：
  SendSimpleVisionData: [0x5A][len=4][id=0x04][float target_yaw(deg)][0xA5]  共 8 字节

用法：
  1. 运行本脚本，终端会打印虚拟串口路径，例如 /dev/pts/3；
  2. 临时把 config/robot/<robot>.toml 的 [mcu] 改为：
       mcu_backend = "Serial"
       serial_device = "/dev/pts/3"
       serial_baud_rate = 115200
  3. at_vision.toml 设置 backend = "ImuOnly"；
  4. 启动 ./build/bin/talos，本脚本会持续解码并打印 yaw 统计。
"""
import os
import pty
import select
import struct
import time

DURATION_S = 20.0
PATH_FILE = "/tmp/imu_only_slave_path"

master, slave = pty.openpty()
slave_path = os.ttyname(slave)
with open(PATH_FILE, "w", encoding="utf-8") as f:
    f.write(slave_path)
print(f"[probe] pty ready: {slave_path}", flush=True)

buf = b""
samples = []
t0 = time.time()
bad = 0

while time.time() - t0 < DURATION_S:
    r, _, _ = select.select([master], [], [], 0.1)
    if master not in r:
        continue
    try:
        data = os.read(master, 4096)
    except OSError:
        break
    if not data:
        break
    buf += data
    while True:
        i = buf.find(b"\x5a")
        if i < 0:
            buf = b""
            break
        buf = buf[i:]
        if len(buf) < 3:
            break
        sof, length, pid = buf[0], buf[1], buf[2]
        if pid != 0x04 or length != 4:
            bad += 1
            buf = buf[1:]
            continue
        need = 3 + length + 1
        if len(buf) < need:
            break
        if buf[need - 1] != 0xA5:
            bad += 1
            buf = buf[1:]
            continue
        yaw = struct.unpack("<f", buf[3:3 + length])[0]
        samples.append((time.time() - t0, yaw))
        buf = buf[need:]

print(f"[probe] frames={len(samples)} bad={bad}", flush=True)
if samples:
    ys = [s[1] for s in samples]
    print(f"[probe] yaw_deg min={min(ys):.3f} max={max(ys):.3f} "
          f"first={ys[0]:.3f} last={ys[-1]:.3f}", flush=True)
    for i in range(0, len(samples), max(1, len(samples) // 12)):
        print(f"[probe]   t={samples[i][0]:6.2f}s yaw={samples[i][1]:8.3f} deg", flush=True)
else:
    print("[probe] NO FRAME RECEIVED", flush=True)

os.close(master)
os.close(slave)
