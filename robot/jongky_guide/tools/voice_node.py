#!/usr/bin/env python3
"""음성으로 목적지를 받는다. Whisper 로 듣고 /guide/destination 에 흘린다.

버튼과 병렬로 동작한다 — 이쪽이 죽어도 터치스크린은 그대로 쓸 수 있다.

    ros2 run jongky_guide voice_node.py --model tiny --mic plughw:1,0

[모델 크기] 젯슨 온보드 CPU 기준이다. tiny 로 시작할 것.
  tiny(39M) 가장 빠르다. 강의장 이름 정도의 짧은 발화에는 충분한 편이다.
  base(74M) 조금 낫지만 그만큼 느리다.
  small 이상은 온보드 실시간에 무리다.

[마이크] `arecord -l` 로 카드 번호를 확인할 것. 아스트라에도 마이크가 있어서
(card 0) 지정하지 않으면 엉뚱한 장치를 잡는다. USB PnP 마이크가 보통 card 1 이다.

[인식 방식] 상시 인식이 아니다. 볼륨이 임계를 넘으면 녹음을 시작하고
조용해지면 끊어서 그 구간만 Whisper 에 넘긴다. 상시로 돌리면 CPU 를 계속
먹고 복도 소음까지 다 받아쓴다.
"""
from __future__ import annotations

import argparse
import queue
import subprocess
import threading
import wave

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

SAMPLE_RATE = 16000        # Whisper 가 기대하는 값
CHUNK = 1024
SILENCE_CHUNKS = 25        # 약 1.6초 조용하면 발화가 끝난 것으로 본다
MIN_SPEECH_CHUNKS = 8      # 너무 짧은 건 기침·문 닫히는 소리다
MAX_SPEECH_CHUNKS = 160    # 약 10초. 그 이상은 잘라 넘긴다


class VoiceNode(Node):
    def __init__(self, model_name: str, mic: str, threshold: int, destinations: list[str]):
        super().__init__("jongky_voice")
        self._pub = self.create_publisher(String, "/guide/destination", 10)
        self._threshold = threshold
        self._mic = mic
        self._destinations = destinations
        self._audio_q: queue.Queue[bytes] = queue.Queue()

        self.get_logger().info(f"Whisper '{model_name}' 로딩 중...")
        import whisper                                   # 로딩이 느려서 여기서 임포트한다

        self._model = whisper.load_model(model_name)
        self.get_logger().info(f"준비됨. 마이크 {mic or '(기본)'} 임계 {threshold}")

        threading.Thread(target=self._capture_loop, daemon=True).start()
        threading.Thread(target=self._recognize_loop, daemon=True).start()

    # ── 녹음 ──────────────────────────────────────────────────────────────
    def _capture_loop(self) -> None:
        """arecord 로 계속 읽으면서 발화 구간만 잘라 큐에 넣는다."""
        cmd = ["arecord", "-q", "-f", "S16_LE", "-r", str(SAMPLE_RATE), "-c", "1", "-t", "raw"]
        if self._mic:
            cmd[1:1] = ["-D", self._mic]

        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        buf: list[bytes] = []
        silence = 0

        while rclpy.ok():
            data = proc.stdout.read(CHUNK * 2)           # S16 이라 샘플당 2바이트
            if not data:
                break
            level = np.abs(np.frombuffer(data, dtype=np.int16)).mean()

            if level > self._threshold:
                buf.append(data)
                silence = 0
            elif buf:
                buf.append(data)
                silence += 1
                if silence >= SILENCE_CHUNKS or len(buf) > MAX_SPEECH_CHUNKS:
                    if len(buf) >= MIN_SPEECH_CHUNKS:
                        self._audio_q.put(b"".join(buf))
                    buf, silence = [], 0

        proc.terminate()

    # ── 인식 ──────────────────────────────────────────────────────────────
    def _recognize_loop(self) -> None:
        import tempfile

        while rclpy.ok():
            raw = self._audio_q.get()
            try:
                with tempfile.NamedTemporaryFile(suffix=".wav") as tmp:
                    with wave.open(tmp.name, "wb") as wf:
                        wf.setnchannels(1)
                        wf.setsampwidth(2)
                        wf.setframerate(SAMPLE_RATE)
                        wf.writeframes(raw)
                    result = self._model.transcribe(tmp.name, language="ko", fp16=False)

                text = result.get("text", "").strip()
                if not text:
                    continue
                self.get_logger().info(f"[들림] {text}")

                dest = self._match(text)
                if dest:
                    self.get_logger().info(f"[목적지] {dest}")
                    msg = String()
                    msg.data = dest
                    self._pub.publish(msg)
            except Exception as exc:                     # 인식 실패가 노드를 죽이면 안 된다
                self.get_logger().warn(f"인식 실패 (무시하고 계속): {exc}")

    def _match(self, text: str) -> str | None:
        """받아쓴 문장에서 목적지를 찾는다.

        Whisper 가 띄어쓰기를 제멋대로 넣으므로 공백을 지우고 비교한다.
        "304호 어디예요" 에서 "304호" 를 건지는 정도면 충분하다.
        """
        flat = text.replace(" ", "")
        for name in self._destinations:
            if name.replace(" ", "") in flat:
                return name
        return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", default="tiny", help="whisper 모델 (tiny/base/small)")
    parser.add_argument("--mic", default="", help="arecord -D 값 (예: plughw:1,0)")
    parser.add_argument("--threshold", type=int, default=500, help="발화 판정 임계. 오작동하면 올릴 것")
    parser.add_argument("--waypoints", default="~/waypoints.yaml")
    args, ros_args = parser.parse_known_args()

    import os

    import yaml

    path = os.path.expanduser(args.waypoints)
    destinations = list(yaml.safe_load(open(path)) or {}) if os.path.exists(path) else []
    if not destinations:
        print(f"경고: waypoint 가 없다 ({path}). 인식해도 매칭할 대상이 없다")

    rclpy.init(args=ros_args)
    node = VoiceNode(args.model, args.mic, args.threshold, destinations)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
