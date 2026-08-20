#!/usr/bin/env python3
"""후면 카메라 사람 탐지 서비스. **젯슨 호스트에서** 돈다.

안내로봇은 앞장서 간다. 그래서 "따라오고 있나" 를 스스로 확인하지 못하면
사람을 두고 혼자 가버린다. 이 서비스가 그 눈이다.

[왜 컨테이너가 아니라 호스트인가]
`jongky:jazzy` 컨테이너의 cv2 는 numpy 2.0 ABI 충돌로 임포트가 안 된다
(`numpy.core.multiarray failed to import`). 반면 호스트에는 torch 2.8 ·
torchvision 0.23 · TensorRT 10.3 · cv2 4.8 이 정상으로 깔려 있고 CUDA 도 붙는다.
컨테이너를 건드려 16GB 이미지를 흔드는 것보다 호스트에서 돌리고 HTTP 로
넘기는 편이 싸다 — brain.py 가 관제 노트북 LLM 을 부르는 것과 같은 구조다.

[탐지기]
torchvision `ssdlite320_mobilenet_v3_large` (COCO 사전학습, person = label 1).
젯슨 Orin Nano 에서 워밍업 후 185ms — 약 5Hz 다. 따라오는지 보는 데는 충분하다.
30Hz 가 필요한 일이 아니다.

[거리 추정의 한계]
bbox 높이로 역산하므로 **전신이 보일 때만 맞는다.** 후면 카메라가 낮게 달려
있으면 가까운 사람은 몸이 잘려 실제보다 멀게 나온다. 그래서 판단의 1차 신호는
`present` 이고, 거리는 보조다. 정확한 거리가 필요하면 ToF 나 depth 를 쓸 것.

[사용]
    python3 follow_service.py                    # 카메라 + 탐지
    python3 follow_service.py --fake             # 카메라 없이 인터페이스만
    curl localhost:8641/follower
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

log = logging.getLogger("follow")

# IMX219 후면 카메라. **잠정값이다.**
# 아스트라는 camera_info 의 K 행렬에서 HFOV 57.86 도를 실측해 썼지만
# IMX219 는 아직 그 경로가 없다. 캘리브레이션 전까지는 렌즈 공칭값을 쓴다.
# 거리 추정이 계통적으로 어긋나면 여기부터 의심할 것.
HFOV_DEG = 62.2
PERSON_HEIGHT_M = 1.70  # 전신이 보일 때의 역산 기준

CAP_W, CAP_H = 640, 360  # 탐지 입력. 1280x720 은 낭비다
PERSON_LABEL = 1         # COCO
MIN_SCORE = 0.45


class Detector:
    def __init__(self, fake: bool = False):
        self.fake = fake
        if fake:
            return
        import torch
        from torchvision.models.detection import (
            SSDLite320_MobileNet_V3_Large_Weights as W,
        )
        from torchvision.models.detection import ssdlite320_mobilenet_v3_large as M

        self.torch = torch
        self.dev = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = M(weights=W.COCO_V1).eval().to(self.dev)
        # 첫 추론은 3.4초 걸린다. 여기서 태워 두지 않으면 첫 판단이 늦는다.
        with torch.no_grad():
            self.model(torch.rand(1, 3, CAP_H, CAP_W, device=self.dev))
        log.info(f"탐지기 준비 완료 ({self.dev})")

    def persons(self, bgr) -> list[tuple[float, float, float, float, float]]:
        """(x1, y1, x2, y2, score) 목록."""
        if self.fake:
            return []
        t = self.torch
        rgb = bgr[:, :, ::-1].copy()
        x = t.from_numpy(rgb).permute(2, 0, 1).float().div_(255.0).to(self.dev)
        with t.no_grad():
            out = self.model([x])[0]
        keep = (out["labels"] == PERSON_LABEL) & (out["scores"] >= MIN_SCORE)
        boxes = out["boxes"][keep].cpu().numpy()
        scores = out["scores"][keep].cpu().numpy()
        return [(*b.tolist(), float(s)) for b, s in zip(boxes, scores)]


class Tracker(threading.Thread):
    """카메라를 계속 읽어 최신 판단만 들고 있는다."""

    daemon = True

    def __init__(self, det: Detector, sensor_id: int = 0):
        super().__init__()
        self.det = det
        self.sensor_id = sensor_id
        self.lock = threading.Lock()
        self.state = {
            "present": False,
            "score": 0.0,
            "bearing_deg": 0.0,
            "distance_m": 0.0,
            "bbox": None,
            "stamp": 0.0,
            "fps": 0.0,
            "error": "" if not det.fake else "fake 모드 — 탐지 안 함",
        }
        self._focal_px = (CAP_W / 2.0) / math.tan(math.radians(HFOV_DEG) / 2.0)

    def _open(self):
        import cv2

        pipe = (
            f"nvarguscamerasrc sensor-id={self.sensor_id} ! "
            f"video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
            f"nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! "
            f"video/x-raw,format=BGR,width={CAP_W},height={CAP_H} ! "
            f"appsink drop=1 max-buffers=1"
        )
        return cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)

    def snapshot(self) -> dict:
        with self.lock:
            s = dict(self.state)
        s["age_s"] = round(time.time() - s["stamp"], 2) if s["stamp"] else -1.0
        return s

    def run(self):
        if self.det.fake:
            return
        cap = self._open()
        if not cap.isOpened():
            with self.lock:
                self.state["error"] = "카메라를 열 수 없다 (리본 체결·i2cdetect 확인)"
            log.error(self.state["error"])
            return
        log.info("카메라 열림")
        n, t0 = 0, time.time()
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                continue
            try:
                dets = self.det.persons(frame)
            except Exception as e:  # 탐지 실패가 주행을 막으면 안 된다
                log.warning(f"탐지 실패: {e}")
                dets = []
            n += 1
            fps = n / max(1e-6, time.time() - t0)
            if n > 60:
                n, t0 = 0, time.time()

            if dets:
                # 가장 큰 것을 고른다 — 여럿이면 제일 가까운 사람이 따라오는 사람이다
                x1, y1, x2, y2, sc = max(dets, key=lambda d: (d[2] - d[0]) * (d[3] - d[1]))
                cx = (x1 + x2) / 2.0
                bearing = math.degrees(math.atan2(cx - CAP_W / 2.0, self._focal_px))
                h = max(1.0, y2 - y1)
                dist = PERSON_HEIGHT_M * self._focal_px / h
                st = {
                    "present": True, "score": round(sc, 3),
                    "bearing_deg": round(bearing, 1),
                    "distance_m": round(dist, 2),
                    "bbox": [round(v, 1) for v in (x1, y1, x2, y2)],
                }
            else:
                st = {"present": False, "score": 0.0, "bearing_deg": 0.0,
                      "distance_m": 0.0, "bbox": None}
            st.update(stamp=time.time(), fps=round(fps, 1), error="")
            with self.lock:
                self.state = st


def make_handler(tracker: Tracker):
    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path.rstrip("/") not in ("/follower", ""):
                self.send_error(404)
                return
            body = json.dumps(tracker.snapshot(), ensure_ascii=False).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *a):  # 폴링이 5Hz 라 기본 로그는 시끄럽다
            pass

    return H


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8641)
    ap.add_argument("--sensor-id", type=int, default=0, help="IMX219 CAM0 = 0")
    ap.add_argument("--fake", action="store_true", help="카메라·탐지 없이 인터페이스만")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    tracker = Tracker(Detector(fake=args.fake), args.sensor_id)
    tracker.start()

    srv = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(tracker))
    log.info(f"http://0.0.0.0:{args.port}/follower 에서 대기")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
