#!/usr/bin/env python3
"""층 판정 시험 — 특히 **핫스팟에 붙어 있을 때 찍지 않는가**.

이 경우가 예외가 아니라 정상 구성이다. VLM 이 관제 노트북에 있어서 11층에서
돌발상황 판단을 쓰려면 젯슨이 노트북 핫스팟에 붙어 있어야 하고, 그러면 건물
SSID 가 보이지 않는다.
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from guide_mission import detect as D            # noqa: E402
from guide_mission.floors import FloorBook       # noqa: E402


def book():
    doc = {"hotspot_ssids": ["jongky"], "floors": {
        "10f": {"label": "10층", "ssid": ["FASTCAMPUS_10F"], "order": 10,
                "elevator": {"board": "ev1"}},
        "11f": {"label": "11층", "ssid": ["FASTCAMPUS_11F", "FC_11F_5G"], "order": 11,
                "elevator": {"board": "ev1"}},
    }}
    return FloorBook.from_dict(doc, validate_now=False)


def runner(table):
    """argv[0] → (코드, 출력). 없으면 127 (명령 자체가 없음)."""
    def run(argv, timeout=2.0):
        return table.get(argv[0], (127, ""))
    return run


class TestSsid(unittest.TestCase):
    def test_iwgetid_를_먼저_쓴다(self):
        d = D.FloorDetector(book(), runner=runner({"iwgetid": (0, "FASTCAMPUS_10F")}))
        self.assertEqual(d.ssid(), ("FASTCAMPUS_10F", D.MATCHED))

    def test_iwgetid_가_없으면_nmcli_로_떨어진다(self):
        d = D.FloorDetector(book(), runner=runner({
            "nmcli": (0, "no:OTHER\nyes:FASTCAMPUS_11F\nno:\n")}))
        self.assertEqual(d.ssid()[0], "FASTCAMPUS_11F")

    def test_둘_다_없으면_도구_없음(self):
        d = D.FloorDetector(book(), runner=runner({}))
        self.assertEqual(d.ssid(), (None, D.NO_TOOL))

    def test_붙어_있지_않으면_무선_없음(self):
        d = D.FloorDetector(book(), runner=runner({
            "iwgetid": (255, ""), "nmcli": (0, "no:AAA\n")}))
        self.assertEqual(d.ssid(), (None, D.NO_WIFI))

    def test_명령이_터져도_예외를_안_던진다(self):
        def boom(argv, timeout=2.0):
            raise RuntimeError("nope")
        d = D.FloorDetector(book(), runner=lambda a, timeout=2.0: D._run(["/no/such/cmd"]))
        self.assertEqual(d.ssid(), (None, D.NO_TOOL))
        self.assertEqual(D._run(["/no/such/binary/xyz"]), (127, ""))


class TestGuess(unittest.TestCase):
    def test_아는_ssid_면_확신한다(self):
        d = D.FloorDetector(book(), runner=runner({"iwgetid": (0, "FASTCAMPUS_11F")}))
        g = d.guess()
        self.assertEqual(g.floor, "11f")
        self.assertTrue(g.confident)
        self.assertTrue(g.known)
        self.assertEqual(g.why, D.MATCHED)

    def test_한_층에_ssid_가_여럿이어도_된다(self):
        d = D.FloorDetector(book(), runner=runner({"iwgetid": (0, "FC_11F_5G")}))
        self.assertEqual(d.guess().floor, "11f")

    def test_핫스팟이면_모른다고_말한다(self):
        d = D.FloorDetector(book(), runner=runner({"iwgetid": (0, "jongky")}))
        g = d.guess()
        self.assertIsNone(g.floor)
        self.assertFalse(g.confident)
        self.assertEqual(g.why, D.HOTSPOT)
        self.assertIn("핫스팟", g.reason)
        self.assertIn("골라", g.reason)     # 사람이 할 일을 알려준다

    def test_모르는_ssid_면_그_이름을_보여_준다(self):
        d = D.FloorDetector(book(), runner=runner({"iwgetid": (0, "GUEST_WIFI")}))
        g = d.guess()
        self.assertIsNone(g.floor)
        self.assertEqual(g.why, D.UNKNOWN_SSID)
        self.assertIn("GUEST_WIFI", g.reason)

    def test_무선이_없으면_모른다(self):
        d = D.FloorDetector(book(), runner=runner({"iwgetid": (255, "")}))
        self.assertIsNone(d.guess().floor)

    def test_always_면_아는_ssid_라도_사람_확인을_받는다(self):
        d = D.FloorDetector(book(), policy="always",
                            runner=runner({"iwgetid": (0, "FASTCAMPUS_11F")}))
        g = d.guess()
        self.assertEqual(g.floor, "11f")     # 제안은 한다
        self.assertFalse(g.confident)        # 그러나 스스로 넘어가지 않는다

    def test_off_면_아예_안_본다(self):
        called = []

        def run(argv, timeout=2.0):
            called.append(argv)
            return (0, "FASTCAMPUS_11F")

        g = D.FloorDetector(book(), policy="off", runner=run).guess()
        self.assertEqual(called, [])
        self.assertEqual(g.why, D.DISABLED)
        self.assertFalse(g.confident)


if __name__ == "__main__":
    unittest.main()
