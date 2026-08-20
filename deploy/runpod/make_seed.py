#!/usr/bin/env python3
"""촬영 bag 에서 Cosmos 씨앗 클립을 뽑는다.

    # 어디가 쓸 만한지 먼저 훑는다 (30초마다 한 장 + 밝기 통계)
    python3 make_seed.py scan  BAG.mcap  --out /tmp/scan

    # 마음에 드는 시각에서 5초를 뽑는다
    python3 make_seed.py clip  BAG.mcap  --at 330 --seconds 5 --out seeds/corridor_10f.mp4

## 왜 이 파일이 mcap 을 직접 읽는가

현장 bag 은 기록이 정상 종료되지 않아 mcap 의 요약(footer) 섹션이 없다.
`ros2 bag reindex` 로 metadata.yaml 은 복구되지만 mcap 자체의 인덱스는 없어서,
파이썬 mcap 라이브러리의 탐색 경로가 실패하고 순차 읽기는 이미지 때문에 메모리를
터뜨린다. 그래서 청크를 하나씩 풀어 필요한 프레임만 꺼낸다.

## 규격을 지켜야 하는 이유

Cosmos 는 생성 모델이라 기하를 보장하지 않는다. 복도 폭·벽 위치·카메라 높이·FOV
가 바뀌면 depth 와 액션의 관계가 깨져 "이 상황에서 이 액션은 안전하다" 는 거짓
신호가 된다. 그래서

  · 아스트라가 640x480 (4:3) 이므로 Cosmos 의 4:3 해상도를 쓴다
    1104x832 (720p 급) 또는 832x624 (540p 급). 자르거나 늘리지 않는다
  · 한 번에 121프레임(약 5초)만 만든다. 길수록 기하가 흘러간다
  · 범위는 길이가 아니라 씨앗 지점 수로 넓힌다
"""
import argparse
import os
import struct
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("opencv 가 필요하다:  pip install opencv-python-headless")


def rd_str(b, o):
    n = struct.unpack_from("<I", b, o)[0]
    o += 4
    return b[o:o + n].decode("utf-8", "replace"), o + n


def parse_chunk(payload):
    o = 8 + 8 + 8 + 4                       # start, end, uncompressed_size, crc
    comp, o = rd_str(payload, o)
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


def iter_images(path, topic):
    """(상대시각[s], HxWx3 RGB) 를 순서대로 내놓는다. 청크 하나만 메모리에 든다."""
    chans, t0 = {}, None
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
            if op != 6:
                f.seek(ln, 1)
                continue
            recs = parse_chunk(f.read(ln))
            o = 0
            while o < len(recs):
                rop = recs[o]
                rln = struct.unpack_from("<Q", recs, o + 1)[0]
                o += 9
                body = recs[o:o + rln]
                o += rln
                if rop == 4:                                  # Channel
                    cid = struct.unpack_from("<H", body, 0)[0]
                    chans[cid] = rd_str(body, 4)[0]
                elif rop == 5:                                # Message
                    cid = struct.unpack_from("<H", body, 0)[0]
                    if chans.get(cid) != topic:
                        continue
                    lt = struct.unpack_from("<Q", body, 6)[0]
                    if t0 is None:
                        t0 = lt
                    # Message: channel(2) sequence(4) log_time(8) publish_time(8)
                    d = body[22:]
                    p = 4 + 8                                 # CDR 헤더 + stamp
                    _fid, p = rd_str(d, p)
                    p = (p + 3) & ~3
                    h, w = struct.unpack_from("<II", d, p)
                    p += 8
                    _enc, p = rd_str(d, p)
                    p += 1                                    # is_bigendian
                    p = (p + 3) & ~3
                    p += 4                                    # step
                    dlen = struct.unpack_from("<I", d, p)[0]
                    p += 4
                    yield (lt - t0) / 1e9, np.frombuffer(
                        d[p:p + dlen], dtype=np.uint8).reshape(h, w, 3)


def cmd_scan(a):
    os.makedirs(a.out, exist_ok=True)
    nxt, rows = 0.0, []
    for t, img in iter_images(a.bag, a.topic):
        if t < nxt:
            continue
        nxt = t + a.every
        g = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        rows.append((t, g.mean(), (g < 20).mean() * 100))
        cv2.imwrite("%s/t%04d.jpg" % (a.out, int(t)),
                    cv2.cvtColor(img, cv2.COLOR_RGB2BGR),
                    [cv2.IMWRITE_JPEG_QUALITY, 85])
    print("%7s %8s %8s" % ("t[s]", "밝기", "암부%"))
    for t, m, d in rows:
        flag = "  <- 어둡다" if m < 40 else ""
        print("%7.0f %8.1f %7.1f%%%s" % (t, m, d, flag))
    if rows:
        arr = np.array([r[1] for r in rows])
        print("\n표본 %d · 밝기 40 이상 %d개 (%.0f%%)"
              % (len(arr), (arr >= 40).sum(), 100 * (arr >= 40).mean()))
        print("밝은 구간의 t 를 골라  clip --at <t>  로 뽑는다")


def cmd_clip(a):
    W, H = (int(x) for x in a.resolution.split("x"))
    need = int(a.seconds * a.fps)
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (W, H))
    n = 0
    for t, img in iter_images(a.bag, a.topic):
        if t < a.at:
            continue
        src_h, src_w = img.shape[:2]
        if abs(src_w / src_h - W / H) > 0.02:
            print("경고: 씨앗 %dx%d 와 출력 %dx%d 의 화면비가 다르다. "
                  "기하가 왜곡되므로 4:3 해상도를 쓸 것" % (src_w, src_h, W, H))
        up = cv2.resize(cv2.cvtColor(img, cv2.COLOR_RGB2BGR), (W, H),
                        interpolation=cv2.INTER_LANCZOS4)
        if n == 0:
            cv2.imwrite(os.path.splitext(a.out)[0] + "_first.png", up)
        vw.write(up)
        n += 1
        if n >= need:
            break
    vw.release()
    print("%s · %d프레임 %dx%d @%dfps (%.1f초)" % (a.out, n, W, H, a.fps, n / a.fps))
    if n < need:
        print("경고: 요청한 %d프레임 중 %d 만 나왔다 — bag 이 그 시각에 끝난다" % (need, n))


def main():
    p = argparse.ArgumentParser(description="bag 에서 Cosmos 씨앗 클립 만들기")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("scan", help="주기적으로 표본을 뽑아 쓸 만한 구간을 찾는다")
    s.add_argument("bag")
    s.add_argument("--topic", default="/camera/rgb/image_raw")
    s.add_argument("--every", type=float, default=30.0, help="표본 간격 [s]")
    s.add_argument("--out", default="/tmp/seed_scan")
    s.set_defaults(func=cmd_scan)

    c = sub.add_parser("clip", help="특정 시각에서 클립을 뽑는다")
    c.add_argument("bag")
    c.add_argument("--topic", default="/camera/rgb/image_raw")
    c.add_argument("--at", type=float, required=True, help="시작 시각 [s]")
    c.add_argument("--seconds", type=float, default=5.0)
    c.add_argument("--fps", type=int, default=30)
    c.add_argument("--resolution", default="1104x832",
                   help="4:3 만 쓸 것. 1104x832(720p) 또는 832x624(540p)")
    c.add_argument("--out", required=True)
    c.set_defaults(func=cmd_clip)

    a = p.parse_args()
    a.func(a)


if __name__ == "__main__":
    main()
