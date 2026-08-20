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

[가중치는 미리 받아 둬야 한다]
torchvision 가중치(약 14MB)는 **첫 실행 때 인터넷에서 받는다.** 저장소에도
컨테이너 이미지에도 없다 — 이 서비스가 호스트에서 도니 이미지에 구워도 소용없다.
층 격리 서브넷에서는 그 다운로드가 안 된다. 인터넷 되는 자리에서 미리:

    robot/jongky_bringup/scripts/fetch_models.sh

캐시가 없으면 이 서비스는 **네트워크를 건드리기 전에 그 사실을 알리고 죽는다.**
받으러 나가게 두면 막힌 망에서 응답 없이 매달릴 수 있고(torch.hub 다운로드에는
타임아웃이 없다), 그러면 "서비스가 안 뜬다" 로만 보여 원인을 찾기 어렵다.
개발 책상처럼 인터넷이 되는 자리에서는 --allow-download 로 받게 할 수 있다.

[사용]
    python3 follow_service.py                    # 카메라 + 탐지
    python3 follow_service.py --fake             # 카메라 없이 인터페이스만
    python3 follow_service.py --hfov 61.4        # 실측한 화각을 적용
    curl localhost:8641/follower                 # calib 필드에 적용값이 보인다
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

log = logging.getLogger("follow")

# ── 거리 추정 캘리브레이션 ──────────────────────────────────────────────────
#
# 둘 다 **아직 실측 전 잠정값이다.** 그리고 거리 추정이 여기에 직접 비례한다
# (dist = PERSON_HEIGHT_M * focal_px / bbox_h, focal_px 는 HFOV 로 정해진다).
# HFOV 가 10% 틀리면 거리도 10% 틀린다.
#
# 아스트라는 camera_info 의 K 행렬에서 HFOV 57.86 도를 실측해 썼지만 IMX219 는
# 아직 그 경로가 없다. 그때까지는 렌즈 공칭값이다.
#
# 소스에 상수로 박아 두면 현장에서 실측하고도 값을 못 넣는다 — 코드를 고치고
# 다시 배포해야 하기 때문이다. 그래서 환경변수와 인자로 뺐다. 실측 절차는
# robot/jongky_bringup/README.md 의 "IMX219 화각(HFOV) 실측" 절.
#
#   JONGKY_HFOV_DEG / --hfov                  카메라 수평 화각(도)
#   JONGKY_PERSON_HEIGHT_M / --person-height  역산 기준 신장(m)
#
# 실측한 뒤에는 아래 기본값도 같이 고치고 "실측" 이라고 적을 것. 그래야
# 다음 사람이 공칭값을 실측값으로 착각하지 않는다.
DEFAULT_HFOV_DEG = 62.2          # IMX219 렌즈 공칭값 (미실측)
DEFAULT_PERSON_HEIGHT_M = 1.70   # 전신이 보일 때의 역산 기준 (미실측)

CAP_W, CAP_H = 640, 360  # 탐지 입력. 1280x720 은 낭비다
PERSON_LABEL = 1         # COCO
MIN_SCORE = 0.45

# torchvision SSDLite320 MobileNetV3-Large COCO_V1 의 캐시 파일명.
# torch.hub 가 URL 마지막 조각을 그대로 파일명으로 쓴다.
WEIGHTS_FILE = "ssdlite320_mobilenet_v3_large_coco-a79551df.pth"
FETCH_SCRIPT = "robot/jongky_bringup/scripts/fetch_models.sh"


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name, "").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        log.warning(f"{name}='{raw}' 를 숫자로 못 읽는다 — 기본값 {default} 를 쓴다")
        return default


HFOV_DEG = _env_float("JONGKY_HFOV_DEG", DEFAULT_HFOV_DEG)
PERSON_HEIGHT_M = _env_float("JONGKY_PERSON_HEIGHT_M", DEFAULT_PERSON_HEIGHT_M)


class Detector:
    def __init__(self, fake: bool = False, allow_download: bool = False):
        self.fake = fake
        if fake:
            return
        import torch
        from torchvision.models.detection import (
            SSDLite320_MobileNet_V3_Large_Weights as W,
        )
        from torchvision.models.detection import ssdlite320_mobilenet_v3_large as M

        # 가중치가 캐시에 있는지 **네트워크를 건드리기 전에** 본다.
        #
        # 없으면 torchvision 이 download.pytorch.org 로 나간다. 층 격리
        # 서브넷에서는 그게 실패하는데, torch.hub 다운로드에는 타임아웃이 없어
        # 막힌 망에서는 에러도 없이 매달린 채로 남는다. 그러면 현장에서는
        # "서비스가 안 뜬다" 로만 보이고 원인이 안 보인다.
        # 그래서 나가기 전에 멈추고, 무엇을 해야 하는지 말하고 죽는다.
        cache = os.path.join(torch.hub.get_dir(), "checkpoints", WEIGHTS_FILE)
        if not os.path.isfile(cache) and not allow_download:
            raise SystemExit(
                f"SSDLite 가중치가 없다: {cache}\n"
                f"  인터넷 되는 자리에서 미리 받을 것:  {FETCH_SCRIPT}\n"
                f"  **follow_service.py 를 돌릴 그 계정으로** 받아야 한다"
                f" (캐시가 ~/.cache/torch 라 sudo 로 받으면 못 찾는다).\n"
                f"  지금 인터넷이 되는 자리라면:  --allow-download\n"
                f"  탐지 없이 인터페이스만 볼 거면:  --fake"
            )

        self.torch = torch
        self.dev = "cuda" if torch.cuda.is_available() else "cpu"
        try:
            self.model = M(weights=W.COCO_V1).eval().to(self.dev)
        except OSError as exc:  # urllib.error.URLError 도 여기로 온다
            # 캐시가 있었는데 실패했다면 다운로드 문제가 아니다 — 그대로 올린다.
            if os.path.isfile(cache):
                raise
            raise SystemExit(
                f"SSDLite 가중치를 받지 못했다: {exc}\n"
                f"  인터넷 되는 자리에서:  {FETCH_SCRIPT}"
            ) from exc
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

    def __init__(
        self,
        det: Detector,
        sensor_id: int = 0,
        hfov_deg: float = HFOV_DEG,
        person_height_m: float = PERSON_HEIGHT_M,
    ):
        super().__init__()
        self.det = det
        self.sensor_id = sensor_id
        self.hfov_deg = hfov_deg
        self.person_height_m = person_height_m
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
        self._focal_px = (CAP_W / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)

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
        # 지금 어떤 값으로 거리를 내고 있는지 curl 한 번으로 보이게 한다.
        # 환경변수로 덮을 수 있게 된 뒤로는 "소스를 보면 안다" 가 성립하지 않는다.
        s["calib"] = {
            "hfov_deg": self.hfov_deg,
            "person_height_m": self.person_height_m,
            "focal_px": round(self._focal_px, 1),
            "measured": self.hfov_deg != DEFAULT_HFOV_DEG,
        }
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
                dist = self.person_height_m * self._focal_px / h
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
    ap.add_argument("--allow-download", action="store_true",
                    help="가중치가 없으면 인터넷에서 받는다. 인터넷 되는 자리에서만. "
                         "기본은 받지 않고 안내하고 죽는다 — 막힌 망에서 매달리지 않게")
    ap.add_argument("--hfov", type=float, default=HFOV_DEG,
                    help=f"카메라 수평 화각(도). 기본 {HFOV_DEG} "
                         f"(JONGKY_HFOV_DEG). 거리 추정이 여기에 비례한다")
    ap.add_argument("--person-height", type=float, default=PERSON_HEIGHT_M,
                    help=f"역산 기준 신장(m). 기본 {PERSON_HEIGHT_M} "
                         f"(JONGKY_PERSON_HEIGHT_M)")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")

    # 어떤 캘리브레이션으로 도는지 로그 첫 줄에 남긴다. 거리가 계통적으로
    # 어긋났을 때 현장 로그만 보고 값을 되짚을 수 있어야 한다.
    tag = "실측" if args.hfov != DEFAULT_HFOV_DEG else "미실측(렌즈 공칭값)"
    log.info(
        f"캘리브레이션: HFOV {args.hfov}도 [{tag}], 기준 신장 {args.person_height}m "
        f"— 실측 절차는 jongky_bringup/README.md 참조"
    )

    tracker = Tracker(
        Detector(fake=args.fake, allow_download=args.allow_download),
        args.sensor_id,
        hfov_deg=args.hfov,
        person_height_m=args.person_height,
    )
    tracker.start()

    srv = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(tracker))
    log.info(f"http://0.0.0.0:{args.port}/follower 에서 대기")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
