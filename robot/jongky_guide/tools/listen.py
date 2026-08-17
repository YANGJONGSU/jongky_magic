#!/usr/bin/env python3
"""눌러서 말하기(PTT). 버튼을 누른 뒤 한 마디를 받아 적는다.

voice_node.py 의 상시 감지와 다르다. 그쪽은 볼륨 임계로 발화를 잡는데,
복도 소음과 로봇 자신의 안내 음성까지 받아쓰는 문제가 있다. PTT 는 사용자가
누른 순간부터만 듣는다. 안내로봇처럼 시끄러운 데서는 이쪽이 확실하다.

말이 끝나는 지점은 조용해지는 것으로 판정한다 — 터치스크린에서 버튼을
누르고 있기가 불편해서, "눌러서 시작하고 말이 끝나면 알아서 멈추는" 쪽으로 했다.
"""
from __future__ import annotations

import logging
import subprocess
import tempfile
import threading
import wave

import numpy as np

log = logging.getLogger("listen")

SAMPLE_RATE = 16000
CHUNK = 1024
SILENCE_LEVEL = 400        # 이보다 조용하면 말이 끊긴 것으로 본다
SILENCE_CHUNKS = 20        # 약 1.3초
MAX_CHUNKS = 240           # 약 15초. 그 이상은 잘라 넘긴다
MIN_CHUNKS = 5             # 너무 짧은 건 버튼 오작동이다


class Listener:
    """PTT 한 번 = 녹음 한 번 + 인식 한 번.

    Whisper 로딩이 몇 초 걸리므로 처음 한 번만 하고 들고 있는다.
    """

    def __init__(self, model_name: str = "tiny", mic: str = ""):
        self._mic = mic
        self._model_name = model_name
        self._model = None
        self._lock = threading.Lock()
        self._busy = False
        threading.Thread(target=self._preload, daemon=True).start()

    def _preload(self) -> None:
        try:
            import whisper

            log.info(f"Whisper '{self._model_name}' 로딩 중...")
            self._model = whisper.load_model(self._model_name)
            log.info("Whisper 준비됨")
        except Exception as exc:      # 음성이 안 되어도 버튼 UI 는 살아야 한다
            log.warning(f"Whisper 로딩 실패 (음성 입력을 건너뛴다): {exc}")

    @property
    def ready(self) -> bool:
        return self._model is not None

    @property
    def busy(self) -> bool:
        return self._busy

    def listen_once(self) -> tuple[str | None, bytes | None]:
        """녹음하고 (받아쓴 문장, WAV 원본) 을 돌려준다.

        원본도 같이 주는 이유: Whisper 가 잘못 받아썼을 때 호출부가 그 오디오를
        그대로 LLM 에 넘길 수 있어야 한다 (3단계 폴백의 마지막).
        """
        if not self.ready:
            log.warning("Whisper 가 아직 준비되지 않았다")
            return None, None
        if not self._lock.acquire(blocking=False):
            log.info("이미 듣고 있다")
            return None, None

        self._busy = True
        try:
            raw = self._record()
            if raw is None:
                return None, None
            wav = self._to_wav(raw)
            return self._transcribe(wav), wav
        except Exception as exc:
            log.warning(f"음성 입력 실패 (무시하고 계속): {exc}")
            return None, None
        finally:
            self._busy = False
            self._lock.release()

    @staticmethod
    def _to_wav(raw: bytes) -> bytes:
        import io

        buf = io.BytesIO()
        with wave.open(buf, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(SAMPLE_RATE)
            wf.writeframes(raw)
        return buf.getvalue()

    def _record(self) -> bytes | None:
        cmd = ["arecord", "-q", "-f", "S16_LE", "-r", str(SAMPLE_RATE), "-c", "1", "-t", "raw"]
        if self._mic:
            cmd[1:1] = ["-D", self._mic]

        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        chunks: list[bytes] = []
        silence = 0
        spoke = False

        try:
            while len(chunks) < MAX_CHUNKS:
                data = proc.stdout.read(CHUNK * 2)
                if not data:
                    break
                chunks.append(data)
                level = np.abs(np.frombuffer(data, dtype=np.int16)).mean()

                if level > SILENCE_LEVEL:
                    spoke = True
                    silence = 0
                elif spoke:
                    silence += 1
                    if silence >= SILENCE_CHUNKS:
                        break
        finally:
            proc.terminate()
            proc.wait(timeout=2)

        if not spoke or len(chunks) < MIN_CHUNKS:
            log.info("들린 말이 없다")
            return None
        return b"".join(chunks)

    def _transcribe(self, wav: bytes) -> str | None:
        with tempfile.NamedTemporaryFile(suffix=".wav") as tmp:
            tmp.write(wav)
            tmp.flush()
            result = self._model.transcribe(tmp.name, language="ko", fp16=False)

        text = result.get("text", "").strip()
        log.info(f"[들림] {text}")
        return text or None
