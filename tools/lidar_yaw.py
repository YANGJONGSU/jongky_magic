#!/usr/bin/env python3
"""차분으로 라이다 0도 방향을 찾는다.

기준 스캔 -> (정면에 물체를 놓는다) -> 두 번째 스캔.
거리가 가장 크게 줄어든 각도가 로봇 정면이다.
주변이 복잡해도 확실하게 잡힌다.
"""
import serial, sys, time, collections

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 460800
BUCKET = 10

def scan(ser, secs):
    ser.write(bytes([0xA5, 0x25])); time.sleep(0.05)
    ser.reset_input_buffer()
    ser.write(bytes([0xA5, 0x20]))
    d = ser.read(7)
    if len(d) < 7 or d[0] != 0xA5:
        return None
    out = collections.defaultdict(lambda: 1e9)
    buf = b''
    t0 = time.time()
    while time.time() - t0 < secs:
        buf += ser.read(max(1, ser.in_waiting))
        while len(buf) >= 5:
            b0 = buf[0]
            if (b0 & 0x01) == ((b0 >> 1) & 0x01) or not (buf[1] & 0x01):
                buf = buf[1:]; continue
            s = buf[:5]; buf = buf[5:]
            q = s[0] >> 2
            a = ((s[1] >> 1) | (s[2] << 7)) / 64.0
            dist = (s[3] | (s[4] << 8)) / 4.0
            if q > 0 and dist > 0:
                k = int(a // BUCKET) * BUCKET
                out[k] = min(out[k], dist)
    ser.write(bytes([0xA5, 0x25])); time.sleep(0.05)
    return out

ser = serial.Serial(port, baud, timeout=2.0)
ser.dtr = False
ser.write(bytes([0xA5, 0x40])); time.sleep(1.2)
ser.reset_input_buffer()

print(">>> 1단계: 기준 스캔 (5초). 지금은 아무것도 놓지 마세요")
base = scan(ser, 5.0)
if not base:
    print("스캔 실패"); ser.dtr = True; ser.close(); sys.exit(1)

print("\n>>> 2단계: 지금 로봇 정면 20~30cm 에 물체를 놓으세요")
for i in range(12, 0, -1):
    print("    %d초..." % i, flush=True); time.sleep(1)

print("\n>>> 3단계: 두 번째 스캔 (5초)")
after = scan(ser, 5.0)
ser.write(bytes([0xA5, 0x25])); time.sleep(0.05)
ser.dtr = True; ser.close()

if not after:
    print("2차 스캔 실패"); sys.exit(1)

print("\n각도   기준(mm)  이후(mm)   변화")
rows = []
for k in sorted(set(base) | set(after)):
    b = base.get(k, 1e9); a = after.get(k, 1e9)
    if b > 1e8 or a > 1e8:
        continue
    diff = a - b
    rows.append((k, b, a, diff))
    mark = ""
    if diff < -80:
        mark = "  <<< 가까워짐 " + "#" * min(30, int(-diff / 30))
    print("  %3d   %7.0f  %7.0f  %+7.0f%s" % (k, b, a, diff, mark))

drops = [r for r in rows if r[3] < -80]
print()
if not drops:
    print("뚜렷하게 가까워진 각도가 없습니다. 물체를 더 가까이(15~20cm) 놓고 다시 하세요.")
else:
    best = min(drops, key=lambda r: r[3])
    print("가장 크게 가까워진 각도 = %d도  (%+.0f mm)" % (best[0], best[3]))
    print("=> 로봇 정면이 라이다 스캔의 %d도에 해당한다\n" % best[0])
    a = best[0]
    if a < 45 or a > 315:
        print("판정: laser_yaw_deg:=0   (라이다 0도가 로봇 정면)")
    elif 135 < a < 225:
        print("판정: laser_yaw_deg:=180  (현재 기본값이 맞다)")
    else:
        print("판정: laser_yaw_deg:=%d  (라이다가 옆으로 돌아 달렸다)" % ((-a) % 360))
