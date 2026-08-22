"""mcap 스트리밍 리더 + 필요한 메시지만 손으로 푸는 CDR 디코더.

deploy/runpod/make_seed.py 의 청크 워커를 일반화한 것. 현장 bag 은 기록이
정상 종료되지 않아 footer/인덱스가 없고, 파이썬 mcap 라이브러리의 탐색
경로가 실패한다. 그래서 청크를 하나씩 풀어 필요한 토픽만 꺼낸다 (make_seed
의 검증된 방식). rosbag2 API 는 젯슨에만 있고 노트북 파이썬에는 없으므로
여기 의존성은 numpy (+ 압축 bag 이면 zstandard/lz4) 뿐이다.

CDR 정렬 주의: 8바이트(double) 정렬은 4바이트 encapsulation 헤더 **뒤**를
원점으로 계산한다. make_seed 의 이미지 파서는 4바이트 필드뿐이라 절대
오프셋 정렬로도 맞았지만, Odometry/Twist 의 double 은 그러면 틀린다.
"""
import os
import struct

import numpy as np


def _rd_str(b, o):
    n = struct.unpack_from("<I", b, o)[0]
    o += 4
    return b[o:o + n - 1].decode("utf-8", "replace"), o + n


class Cursor:
    """CDR 페이로드 커서. 정렬 원점은 encapsulation 헤더(4바이트) 직후."""

    def __init__(self, buf):
        self.b = buf
        self.p = 4

    def align(self, n):
        r = (self.p - 4) % n
        if r:
            self.p += n - r

    def u32(self):
        self.align(4)
        v = struct.unpack_from("<I", self.b, self.p)[0]
        self.p += 4
        return v

    def i32(self):
        self.align(4)
        v = struct.unpack_from("<i", self.b, self.p)[0]
        self.p += 4
        return v

    def f32(self, n=1):
        self.align(4)
        v = struct.unpack_from("<%df" % n, self.b, self.p)
        self.p += 4 * n
        return v[0] if n == 1 else v

    def f64(self, n=1):
        self.align(8)
        v = struct.unpack_from("<%dd" % n, self.b, self.p)
        self.p += 8 * n
        return v[0] if n == 1 else v

    def string(self):
        self.align(4)
        s, self.p = _rd_str(self.b, self.p)
        return s

    def stamp(self):
        """builtin_interfaces/Time → 초 (float)."""
        return self.i32() + self.u32() / 1e9

    def header(self):
        t = self.stamp()
        frame = self.string()
        return t, frame


def _parse_chunk(payload):
    o = 8 + 8 + 8 + 4
    comp, o = _rd_str_raw(payload, o)
    rlen = struct.unpack_from("<Q", payload, o)[0]
    o += 8
    raw = payload[o:o + rlen]
    if comp == "zstd":
        import zstandard
        return zstandard.ZstdDecompressor().decompressobj().decompress(raw)
    if comp == "lz4":
        import lz4.frame
        return lz4.frame.decompress(raw)
    if comp not in ("", "none"):
        raise RuntimeError("모르는 압축: %s" % comp)
    return raw


def _rd_str_raw(b, o):
    # mcap 레코드 내부 문자열 (CDR 이 아님 — null 종료 없음)
    n = struct.unpack_from("<I", b, o)[0]
    o += 4
    return b[o:o + n].decode("utf-8", "replace"), o + n


def iter_messages(path, topics):
    """(topic, log_time[ns], cdr_payload) 를 기록 순서로 내놓는다.

    topics 는 집합/리스트. 청크 하나만 메모리에 든다.
    """
    want = set(topics)
    chans = {}
    size = os.path.getsize(path)
    with open(path, "rb") as f:
        f.read(8)
        while True:
            hdr = f.read(9)
            if len(hdr) < 9:
                return
            op = hdr[0]
            ln = struct.unpack("<Q", hdr[1:])[0]
            if ln > size:
                return
            if op != 6:                       # Chunk 만 본다
                f.seek(ln, 1)
                continue
            recs = _parse_chunk(f.read(ln))
            o = 0
            while o < len(recs):
                rop = recs[o]
                rln = struct.unpack_from("<Q", recs, o + 1)[0]
                o += 9
                body = recs[o:o + rln]
                o += rln
                if rop == 4:                  # Channel
                    cid = struct.unpack_from("<H", body, 0)[0]
                    chans[cid] = _rd_str_raw(body, 4)[0]
                elif rop == 5:                # Message
                    cid = struct.unpack_from("<H", body, 0)[0]
                    topic = chans.get(cid)
                    if topic not in want:
                        continue
                    lt = struct.unpack_from("<Q", body, 6)[0]
                    yield topic, lt, body[22:]


# ── 메시지별 디코더 ───────────────────────────────────────────────────────

def decode_twist_stamped(d):
    """geometry_msgs/TwistStamped → (stamp, v, omega)."""
    c = Cursor(d)
    t, _ = c.header()
    lin = c.f64(3)
    ang = c.f64(3)
    return t, lin[0], ang[2]


def decode_twist(d):
    """geometry_msgs/Twist → (v, omega)."""
    c = Cursor(d)
    lin = c.f64(3)
    ang = c.f64(3)
    return lin[0], ang[2]


def decode_odometry(d):
    """nav_msgs/Odometry → (stamp, x, y, yaw, v, omega)."""
    c = Cursor(d)
    t, _ = c.header()
    c.string()                                # child_frame_id
    px, py, _pz = c.f64(3)
    qx, qy, qz, qw = c.f64(4)
    c.f64(36)                                 # pose covariance
    lin = c.f64(3)
    ang = c.f64(3)
    c.f64(36)                                 # twist covariance
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return t, px, py, yaw, lin[0], ang[2]


def decode_laserscan(d):
    """sensor_msgs/LaserScan → (stamp, angle_min, angle_inc, ranges[float32])."""
    c = Cursor(d)
    t, _ = c.header()
    angle_min = c.f32()
    _angle_max = c.f32()
    angle_inc = c.f32()
    _time_inc = c.f32()
    _scan_time = c.f32()
    range_min = c.f32()
    range_max = c.f32()
    n = c.u32()
    c.align(4)
    ranges = np.frombuffer(c.b, dtype="<f4", count=n, offset=c.p).copy()
    # 0·inf·범위 밖은 무효 → nan 으로 통일
    bad = (ranges < range_min) | (ranges > range_max) | ~np.isfinite(ranges)
    ranges[bad] = np.nan
    return t, angle_min, angle_inc, ranges


def decode_image(d):
    """sensor_msgs/Image (rgb8/bgr8) → (stamp, HxWx3 uint8, encoding)."""
    c = Cursor(d)
    t, _ = c.header()
    h = c.u32()
    w = c.u32()
    enc = c.string()
    c.p += 1                                  # is_bigendian
    c.u32()                                   # step
    n = c.u32()
    if n != h * w * 3 or len(c.b) - c.p < n:
        return t, None, enc                   # 잘린 프레임 (make_seed 와 동일 처리)
        # 두 번째 조건: 기록이 중간에 끊긴 bag 은 선언 길이(n)는 멀쩡한데
        # 페이로드가 짧은 메시지가 있다 (10f_0819_2016 실측)
    img = np.frombuffer(c.b, dtype=np.uint8, count=n, offset=c.p).reshape(h, w, 3)
    return t, img, enc
