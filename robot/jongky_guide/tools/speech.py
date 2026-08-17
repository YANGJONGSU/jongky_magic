#!/usr/bin/env python3
"""오프라인 한국어 TTS. piper 를 쓰고, 없으면 조용히 로그만 남긴다.

piper 를 고른 이유는 젯슨 온보드에서 돌고 네트워크가 필요 없기 때문이다.
건물 10·11층이 격리된 서브넷이라 클라우드 TTS 는 층을 넘는 순간 끊긴다.

음성 모델은 이미지에 굽지 않고 실행 시 경로로 준다. 라이선스가 모델마다
다르고 크기도 수십 MB 라 저장소에 넣기 부적절하다.

    # 모델 받기 (한국어)
    huggingface.co/rhasspy/piper-voices/tree/main/ko/ko_KR
    ros2 run jongky_guide guide_node.py --voice ~/voices/ko_KR-xxx.onnx

piper 가 없으면 예외를 던지지 않고 로그만 찍는다. TTS 때문에 주행이 멈추면
안 되기 때문이다 — 안내 음성은 있으면 좋은 것이지 안전에 필요한 게 아니다.
"""
from __future__ import annotations

import logging
import os
import queue
import shutil
import subprocess
import threading

log = logging.getLogger("speech")


class Speaker:
    """말하기 요청을 큐에 넣고 한 줄씩 재생한다.

    비동기인 이유: 재생이 끝날 때까지 기다리면 그 동안 Nav2 피드백을 못 받는다.
    """

    def __init__(self, voice: str = "", device: str = "", rate: int = 22050):
        self._voice = os.path.expanduser(voice) if voice else ""
        self._device = device
        self._rate = rate
        self._queue: queue.Queue[str] = queue.Queue()
        self._piper = shutil.which("piper") or shutil.which("piper-tts")

        if not self._piper:
            log.warning("piper 가 없다. 음성 안내를 건너뛴다 (주행에는 지장 없음)")
        elif not self._voice or not os.path.exists(self._voice):
            log.warning(f"음성 모델을 못 찾았다: {self._voice or '(미지정)'} — 음성 안내를 건너뛴다")
            self._piper = None

        threading.Thread(target=self._worker, daemon=True).start()

    @property
    def available(self) -> bool:
        return self._piper is not None

    def say(self, text: str) -> None:
        """비차단. 큐에 쌓고 바로 돌아온다."""
        log.info(f"[말] {text}")
        if self._piper:
            self._queue.put(text)

    def _worker(self) -> None:
        while True:
            text = self._queue.get()
            try:
                self._synthesize_and_play(text)
            except Exception as exc:  # 음성 실패가 주행을 막으면 안 된다
                log.warning(f"음성 재생 실패 (무시하고 계속): {exc}")
            finally:
                self._queue.task_done()

    def _synthesize_and_play(self, text: str) -> None:
        aplay = ["aplay", "-q", "-r", str(self._rate), "-f", "S16_LE", "-t", "raw", "-"]
        if self._device:
            aplay[1:1] = ["-D", self._device]

        piper = subprocess.Popen(
            [self._piper, "--model", self._voice, "--output-raw"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        player = subprocess.Popen(aplay, stdin=piper.stdout, stderr=subprocess.DEVNULL)
        piper.stdout.close()          # 재생기가 EOF 를 받게 한다
        piper.stdin.write(text.encode())
        piper.stdin.close()
        player.wait()
        piper.wait()
