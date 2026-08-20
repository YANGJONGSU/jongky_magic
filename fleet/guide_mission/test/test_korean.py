#!/usr/bin/env python3
"""조사 시험. 화면과 음성에 그대로 나가는 문장이다."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from guide_mission.korean import eul, i_ga, ro   # noqa: E402


class TestRo(unittest.TestCase):
    def test_받침이_있으면_으로(self):
        self.assertEqual(ro("11층"), "11층으로")
        self.assertEqual(ro("정문"), "정문으로")

    def test_받침이_없으면_로(self):
        self.assertEqual(ro("로비"), "로비로")
        self.assertEqual(ro("1004호"), "1004호로")

    def test_ㄹ_받침은_로(self):
        self.assertEqual(ro("교실"), "교실로")
        self.assertEqual(ro("서울"), "서울로")

    def test_숫자로_끝나면_읽는_소리로_정한다(self):
        self.assertEqual(ro("A동 1"), "A동 1로")     # 일 → ㄹ 받침
        self.assertEqual(ro("승강기 3"), "승강기 3으로")  # 삼
        self.assertEqual(ro("2"), "2로")             # 이

    def test_영문으로_끝나면_로(self):
        self.assertEqual(ro("11a"), "11a로")

    def test_빈_문자열(self):
        self.assertEqual(ro(""), "")


class TestOthers(unittest.TestCase):
    def test_을를(self):
        self.assertEqual(eul("엘리베이터"), "엘리베이터를")
        self.assertEqual(eul("지도"), "지도를")
        self.assertEqual(eul("버튼"), "버튼을")

    def test_이가(self):
        self.assertEqual(i_ga("문"), "문이")
        self.assertEqual(i_ga("로비"), "로비가")


if __name__ == "__main__":
    unittest.main()
