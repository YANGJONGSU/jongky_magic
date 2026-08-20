#!/usr/bin/env python3
"""상태머신과 로봇 어댑터의 계약 시험.

transfer.py 는 Effects 의 메서드만 부른다. 로봇 쪽 구현은 guide_node.py 의
`NavEffects` 다. 그 둘이 어긋나면 **엘리베이터 앞에서 AttributeError 로 죽는다**
— 현장에서 가장 늦게, 가장 비싸게 발견되는 종류의 고장이다.

그래서 여기서 소스를 읽어(임포트하지 않는다 — rclpy 가 없어도 돌아야 한다)
메서드 이름이 맞는지 본다.
"""
import ast
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from guide_mission import transfer as T          # noqa: E402
from guide_mission.effects import Effects, FakeEffects   # noqa: E402

GUIDE_NODE = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..", "robot", "jongky_guide", "tools", "guide_node.py"))

REQUIRED = [n for n in vars(Effects) if not n.startswith("_")]


class TestContract(unittest.TestCase):
    def test_시험용_구현이_계약을_지킨다(self):
        for name in REQUIRED:
            self.assertTrue(callable(getattr(FakeEffects, name, None)),
                            f"FakeEffects 에 {name} 이 없다")

    def test_상태머신은_계약_밖의_것을_부르지_않는다(self):
        src = ast.parse(open(T.__file__).read())
        called = set()
        for node in ast.walk(src):
            if (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and isinstance(node.func.value, ast.Attribute)
                    and node.func.value.attr == "fx"):
                called.add(node.func.attr)
        self.assertTrue(called, "fx 호출을 하나도 못 찾았다 — 시험이 낡았다")
        self.assertTrue(called <= set(REQUIRED),
                        f"계약에 없는 것을 부른다: {sorted(called - set(REQUIRED))}")

    @unittest.skipUnless(os.path.exists(GUIDE_NODE), "guide_node.py 가 없다")
    def test_로봇_어댑터가_계약을_지킨다(self):
        tree = ast.parse(open(GUIDE_NODE).read())
        cls = next((n for n in ast.walk(tree)
                    if isinstance(n, ast.ClassDef) and n.name == "NavEffects"), None)
        self.assertIsNotNone(cls, "guide_node.py 에 NavEffects 가 없다")
        have = {n.name for n in cls.body if isinstance(n, ast.FunctionDef)}
        missing = set(REQUIRED) - have
        self.assertFalse(missing, f"NavEffects 에 없다: {sorted(missing)}")

    @unittest.skipUnless(os.path.exists(GUIDE_NODE), "guide_node.py 가 없다")
    def test_로봇_쪽이_쓰는_상수가_상태머신에_있다(self):
        """guide_node 가 mission.XXX 로 부르는 이름이 실제로 있는가."""
        tree = ast.parse(open(GUIDE_NODE).read())
        used = {n.attr for n in ast.walk(tree)
                if isinstance(n, ast.Attribute)
                and isinstance(n.value, ast.Name) and n.value.id == "mission"}
        self.assertTrue(used)
        for name in used:
            self.assertTrue(hasattr(T, name), f"transfer 에 {name} 이 없다")


if __name__ == "__main__":
    unittest.main()
