#!/usr/bin/env python3
"""야붐 보드 시뮬레이터.

pty 를 열어 가상 시리얼 포트를 만들고, 실물 보드와 같은 부호 규약으로
동작한다. jongky_hardware 의 부호 변환을 하드웨어 없이 검증하는 용도.

실물 보드의 규약 (야붐보드-레퍼런스.md 4절):
  받은 +vx  = 물리적 후진
  받은 +vz  = 반시계
  엔코더 +  = 후진
  ch2 = 오른쪽, ch3 = 왼쪽
"""
import math
import os
import pty
import termios
import tty
import struct
import sys
import threading
import time

HEAD, DEV, RXDEV = 0xFF, 0xFC, 0xFB
CPR = 3182.0
R = 0.0335
L = 0.11625

state = {
    "enc": [0.0, 0.0, 0.0, 0.0],   # ch1..ch4 (실수로 누적, 보고 시 int)
    "board_vx": 0.0,               # 보드가 받은 값 그대로
    "board_vz": 0.0,
    "last_cmd": None,
}
lock = threading.Lock()


def tx_frame(fd, ftype, payload):
    body = list(payload)
    # LEN = 페이로드 + 3. 수신자는 LEN-2 바이트를 읽고 마지막을 체크섬으로 본다.
    # (실물 REPORT_SPEED: 페이로드 7바이트 -> LEN 0x0A)
    ln = len(body) + 3
    chk = (ln + ftype + sum(body)) % 256
    os.write(fd, bytes([HEAD, RXDEV, ln, ftype] + body + [chk]))


def i16(v):
    return list(struct.pack("<h", max(-32768, min(32767, int(v)))))


def i32(v):
    return list(struct.pack("<i", int(v)))


def reporter(fd):
    """25 Hz 로 엔코더·속도·IMU 프레임을 쏜다."""
    prev = time.time()
    while True:
        time.sleep(0.04)
        now = time.time()
        dt = now - prev
        prev = now
        with lock:
            bvx, bvz = state["board_vx"], state["board_vz"]
            # 보드가 받은 +vx 는 물리적 후진이므로 실제 전진속도는 -bvx
            vx_phys = -bvx
            vz_phys = bvz
            v_right = vx_phys + vz_phys * L / 2.0
            v_left = vx_phys - vz_phys * L / 2.0
            # 엔코더는 후진에서 증가하므로 전진 회전에 대해 음수로 쌓는다
            k = CPR / (2.0 * math.pi) / R
            state["enc"][1] += -v_right * k * dt   # ch2 = 오른쪽
            state["enc"][2] += -v_left * k * dt    # ch3 = 왼쪽
            enc = [int(e) for e in state["enc"]]
            report_vx, report_vz = bvx, bvz

        tx_frame(fd, 0x0D, sum([i32(e) for e in enc], []))
        tx_frame(fd, 0x0A, i16(report_vx * 1000) + i16(0) + i16(report_vz * 1000) + [121])
        # ICM20948: 자이로 z 는 시계가 양수이므로 반시계인 vz_phys 를 뒤집어 보고
        gyro = i16(0) + i16(0) + i16(-vz_phys * 1000)
        accel = i16(0) + i16(0) + i16(-9810)
        tx_frame(fd, 0x0E, gyro + accel + i16(0) + i16(0) + i16(0))


def main():
    master, slave = pty.openpty()
    # raw 모드 필수. 기본 모드면 0x0A(LF)·0x0D(CR) 가 변환돼 프레임이 깨진다.
    # 하필 REPORT_SPEED=0x0A, REPORT_ENCODER=0x0D 라 정통으로 맞는다.
    for fd in (master, slave):
        tty.setraw(fd, termios.TCSANOW)
    name = os.ttyname(slave)
    print(name, flush=True)

    threading.Thread(target=reporter, args=(master,), daemon=True).start()

    buf = b""
    while True:
        buf += os.read(master, 256)
        while len(buf) >= 5:
            if buf[0] != HEAD or buf[1] != DEV:
                buf = buf[1:]
                continue
            ln = buf[2]
            total = ln + 2
            if len(buf) < total:
                break
            frame, buf = buf[:total], buf[total:]
            func, data = frame[3], frame[4:-1]
            if func == 0x12 and len(data) >= 7:      # MOTION
                vx = struct.unpack("<h", data[1:3])[0] / 1000.0
                vz = struct.unpack("<h", data[5:7])[0] / 1000.0
                with lock:
                    state["board_vx"], state["board_vz"] = vx, vz
                    state["last_cmd"] = (vx, vz)
                print(f"RX MOTION vx={vx:+.4f} vz={vz:+.4f}", file=sys.stderr, flush=True)
            elif func == 0x01:
                print("RX AUTO_REPORT on", file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
