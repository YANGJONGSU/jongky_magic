"""guide_mission — 안내로봇 임무 상태머신 (층 전환).

ROS 없이 도는 순수 파이썬이다. `guide_node.py` 가 임포트해 쓰고, 시험은
로봇도 Isaac 도 없이 `python3 -m unittest discover fleet/guide_mission/test`
로 돈다.

  floors.py    층별 지도·waypoint 대장과 짝 검증
  detect.py    층 판정 (SSID / 사람)
  transfer.py  엘리베이터 상태머신
  effects.py   상태머신이 바깥에 하는 일의 경계면
"""

__all__ = ["floors", "detect", "transfer", "effects"]
