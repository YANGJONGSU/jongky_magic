#!/usr/bin/env python3
"""RPLIDAR 에 직접 물어본다 — 모델·펌웨어·상태, 그리고 모터 기동."""
import serial, struct, sys, time

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 460800

ser = serial.Serial(port, baud, timeout=1.5)
print("포트 %s @ %d 열림" % (port, baud))

def req(cmd):
    ser.reset_input_buffer()
    ser.write(bytes([0xA5, cmd]))
    time.sleep(0.05)

def read_descriptor():
    d = ser.read(7)
    if len(d) < 7 or d[0] != 0xA5 or d[1] != 0x5A:
        return None
    length = d[2] | (d[3] << 8) | (d[4] << 16) | ((d[5] & 0x3F) << 24)
    return length, d[6]

# 모터를 먼저 켠다. DTR 을 내리면 모터가 돈다 (SDK 동작과 동일).
print("\n[1] 모터 기동 (DTR low)")
ser.dtr = False
time.sleep(1.0)

print("[2] STOP + RESET")
ser.write(bytes([0xA5, 0x25])); time.sleep(0.05)
ser.write(bytes([0xA5, 0x40])); time.sleep(0.8)
ser.reset_input_buffer()

print("\n[3] 장치 정보 조회")
req(0x50)
d = read_descriptor()
if d:
    n, dtype = d
    data = ser.read(n)
    if len(data) >= 20:
        model = data[0]
        fw = "%d.%02d" % (data[2], data[1])
        hw = data[3]
        sn = "".join("%02X" % b for b in data[4:20])
        names = {0x18: "A1", 0x28: "A2", 0x31: "A3", 0x41: "S1",
                 0x61: "C1", 0x62: "C1", 0x97: "S2"}
        print("  모델번호 0x%02X  (%s)" % (model, names.get(model, "알 수 없음")))
        print("  펌웨어  %s   하드웨어 rev %d" % (fw, hw))
        print("  시리얼  %s" % sn)
    else:
        print("  응답 짧음: %s" % data.hex(' '))
else:
    print("  응답 없음 — 통신 실패")

print("\n[4] 상태 조회")
req(0x52)
d = read_descriptor()
if d:
    n, _ = d
    data = ser.read(n)
    if len(data) >= 3:
        st = data[0]
        err = data[1] | (data[2] << 8)
        名 = {0: "정상", 1: "경고", 2: "오류"}.get(st, "?")
        print("  상태 %d (%s)  에러코드 0x%04X" % (st, 名, err))
        if st == 2:
            print("  -> 오류 상태다. RESET 후에도 남으면 하드웨어 고장이다")
    else:
        print("  응답 짧음: %s" % data.hex(' '))
else:
    print("  응답 없음")

print("\n[5] 모터 10초 구동 — 원통이 도는지 보세요")
ser.dtr = False
for i in range(10):
    print("  %d초..." % (i + 1), flush=True)
    time.sleep(1)

print("\n[6] 모터 정지 (DTR high)")
ser.dtr = True
time.sleep(0.5)
ser.close()
print("완료")
