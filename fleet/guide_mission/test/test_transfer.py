#!/usr/bin/env python3
"""층 전환 상태머신 단위 시험 — 로봇도 Isaac 도 Nav2 도 없이 돈다.

    python3 -m unittest discover fleet/guide_mission/test

여기서 덮는 것은 **전이와 실패 거동**이다. 실제로 문이 열리는지, AMCL 이
정말로 수렴하는지는 현장에서만 확인된다 (README 의 '시험이 못 덮는 것').
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from guide_mission import transfer as T                      # noqa: E402
from guide_mission.detect import FloorGuess, HOTSPOT, MATCHED  # noqa: E402
from guide_mission.effects import FakeEffects                 # noqa: E402
from guide_mission.floors import FloorBook, Waypoint          # noqa: E402


DOC = {
    "hotspot_ssids": ["jongky"],
    "floors": {
        "10f": {"label": "10층", "order": 10, "map": "/m/10f.yaml",
                "waypoints": "/w/10f.yaml", "ssid": ["FASTCAMPUS_10F"],
                "elevator": {"board": "ev1", "exit": "ev1"},
                "labels": {"ev1": "엘리베이터 앞", "10a": "1004호 강의장"}},
        "11f": {"label": "11층", "order": 11, "map": "/m/11f.yaml",
                "waypoints": "/w/11f.yaml", "ssid": ["FASTCAMPUS_11F"],
                "elevator": {"board": "ev1", "exit": "ev1"},
                "labels": {"ev1": "엘리베이터 앞", "11a": "1101호 강의장"}},
    },
}


class StubBook(FloorBook):
    """파일을 안 읽는 대장. reload_waypoints 만 흉내 낸다."""

    reload_fail = ""

    def reload_waypoints(self, key):
        if self.reload_fail:
            return False, self.reload_fail
        return self.get(key) is not None, ""


def make_book() -> StubBook:
    base = FloorBook.from_dict(DOC, validate_now=False)
    book = StubBook(floors=base.floors, hotspot_ssids=base.hotspot_ssids)
    for key, names in (("10f", ("ev1", "10a")), ("11f", ("ev1", "11a"))):
        fl = book.get(key)
        fl.waypoints = {n: Waypoint(n, fl.label_of(n), 1.0, 2.0) for n in names}
    return book


def run(script, *, target="11f", here="10f", destination="11a",
        book=None, effects=None, cfg=None):
    book = book or make_book()
    fx = effects or FakeEffects(
        guess=FloorGuess(target, "FASTCAMPUS_11F", True, MATCHED, "무선으로 판정"))
    m = T.FloorTransfer(book, fx, gate=T.ScriptedGate(script), config=cfg or T.Config())
    out = m.run(here, target, destination)
    return m, fx, out


class TestHappyPath(unittest.TestCase):
    def setUp(self):
        self.m, self.fx, self.out = run(
            [T.CALLED, T.BOARDED, T.ARRIVED, T.EXITED])

    def test_끝까지_간다(self):
        self.assertEqual(self.out.state, T.DONE)
        self.assertTrue(self.out.ok)
        self.assertEqual(self.out.floor, "11f")

    def test_전이_순서(self):
        self.assertEqual(
            [h[0] for h in self.out.history],
            [T.IDLE, T.TO_ELEVATOR, T.AT_ELEVATOR, T.BOARDING, T.RIDING,
             T.EXITING, T.CONFIRM_FLOOR, T.SWAP_MAP, T.RELOCALIZE, T.LANDED,
             T.RESUME])

    def test_새_층_지도를_올린다(self):
        self.assertEqual(self.fx.loaded_maps, ["11f"])

    def test_지도를_갈면_반드시_초기_위치를_다시_준다(self):
        # 이걸 빠뜨리면 map->odom 이 안 나오고 어떤 목적지도 안 간다
        self.assertEqual(self.fx.poses, [("11f", "ev1")])
        order = [c[0] for c in self.fx.calls]
        self.assertLess(order.index("load_map"), order.index("relocalize"))

    def test_끝나면_다시_주행_가능_상태(self):
        self.assertTrue(self.out.localized)
        self.assertTrue(self.fx.localized)
        self.assertFalse(self.out.needs_manual_start)

    def test_엘리베이터_앞까지는_자율주행(self):
        self.assertIn(("navigate", "10f", "ev1"), self.fx.calls)

    def test_사람이_할_일을_말과_화면으로_알린다(self):
        joined = " ".join(self.fx.spoken)
        self.assertIn("버튼을 눌러 주세요", joined)
        self.assertIn("안으로 넣어 주세요", joined)
        self.assertIn("내려 주세요", joined)
        acts = [a["actions"] for a in self.fx.announcements]
        self.assertIn((T.CALLED, T.ABORT), acts)
        self.assertIn((T.BOARDED, T.ABORT), acts)
        self.assertIn((T.ARRIVED, T.ABORT), acts)
        self.assertIn((T.EXITED, T.ABORT), acts)

    def test_타는_순간_초기위치를_버린다(self):
        # 금속 박스 안에서 AMCL 이 내는 값은 헛것이다
        self.assertIn(("set_localized", False), self.fx.calls)
        i = self.fx.calls.index(("set_localized", False))
        j = self.fx.calls.index(("relocalize", "11f", "ev1"))
        self.assertLess(i, j)

    def test_새_층에서_안내를_이어간다(self):
        self.assertIn(("resume", "11a"), self.fx.calls)


class TestFailures(unittest.TestCase):
    def test_지도교체_실패는_주행으로_이어지지_않는다(self):
        fx = FakeEffects(
            guess=FloorGuess("11f", "S", True, MATCHED, ""),
            map_fail="RESULT_MAP_DOES_NOT_EXIST — 파일이 없다")
        m, fx, out = run([T.CALLED, T.BOARDED, T.ARRIVED, T.EXITED], effects=fx)
        self.assertEqual(out.state, T.FAULT)
        self.assertIn("지도를 불러오지 못했다", out.reason)
        # 핵심: 초기 위치가 무효인 채로 끝난다 → guide_node 가 목적지를 거부한다
        self.assertFalse(out.localized)
        self.assertFalse(fx.localized)
        self.assertTrue(out.needs_manual_start)
        self.assertNotIn(("resume", "11a"), fx.calls)
        self.assertNotIn(("relocalize", "11f", "ev1"), fx.calls)
        self.assertGreater(fx.holds, 0)

    def test_지도교체_실패도_말과_화면으로_알린다(self):
        fx = FakeEffects(guess=FloorGuess("11f", "S", True, MATCHED, ""),
                         map_fail="이유")
        m, fx, out = run([T.CALLED, T.BOARDED, T.ARRIVED, T.EXITED], effects=fx)
        last = fx.announcements[-1]
        self.assertIn("멈췄습니다", last["message"])
        self.assertTrue(any("멈췄습니다" in s for s in fx.spoken))

    def test_waypoint를_못_읽으면_지도를_건드리지_않는다(self):
        book = make_book()
        book.reload_fail = "11층 waypoint 파일이 없다"
        m, fx, out = run([T.CALLED, T.BOARDED, T.ARRIVED, T.EXITED], book=book)
        self.assertEqual(out.state, T.FAULT)
        self.assertNotIn("load_map", [c[0] for c in fx.calls])

    def test_AMCL_재초기화_실패는_정지다(self):
        fx = FakeEffects(guess=FloorGuess("11f", "S", True, MATCHED, ""),
                         pose_fail="amcl_pose 가 15초 안에 안 왔다")
        m, fx, out = run([T.CALLED, T.BOARDED, T.ARRIVED, T.EXITED], effects=fx)
        self.assertEqual(out.state, T.FAULT)
        self.assertFalse(out.localized)
        self.assertNotIn(("resume", "11a"), fx.calls)

    def test_엘리베이터_앞까지_못_가면_접는다(self):
        fx = FakeEffects(guess=FloorGuess("11f", "S", True, MATCHED, ""),
                         nav_fail="경로를 찾지 못했다")
        m, fx, out = run([T.CALLED], effects=fx)
        self.assertEqual(out.state, T.FAULT)
        self.assertEqual([h[0] for h in out.history], [T.IDLE, T.TO_ELEVATOR])
        self.assertNotIn("load_map", [c[0] for c in fx.calls])

    def test_사람이_안_누르면_시간_지나_정지(self):
        m, fx, out = run([T.TIMEOUT])
        self.assertEqual(out.state, T.FAULT)
        self.assertIn("시간이 지났다", out.reason)

    def test_새_층에_목적지가_없으면_실패로_끝난다(self):
        m, fx, out = run([T.CALLED, T.BOARDED, T.ARRIVED, T.EXITED],
                         destination="10a")   # 10층 목적지를 11층에서 찾는다
        self.assertEqual(out.state, T.FAULT)
        self.assertIn("다른 층", out.reason)

    def test_모르는_층은_시작도_안_한다(self):
        m, fx, out = run([], target="99f")
        self.assertEqual(out.state, T.FAULT)
        self.assertEqual(fx.calls.count(("navigate", "10f", "ev1")), 0)

    def test_같은_층이면_아무것도_안_한다(self):
        m, fx, out = run([], target="10f")
        self.assertEqual(out.state, T.DONE)
        self.assertEqual(fx.calls, [])


class TestAbort(unittest.TestCase):
    def test_타기_전_취소는_위치를_잃지_않는다(self):
        m, fx, out = run([T.ABORT])
        self.assertEqual(out.state, T.ABORTED)
        self.assertTrue(out.localized)
        self.assertFalse(out.needs_manual_start)
        self.assertEqual(out.floor, "10f")

    def test_탄_뒤_취소는_위치를_잃는다(self):
        m, fx, out = run([T.CALLED, T.BOARDED, T.ABORT])
        self.assertEqual(out.state, T.ABORTED)
        self.assertFalse(out.localized)
        self.assertTrue(out.needs_manual_start)
        self.assertIsNone(out.floor)
        self.assertIn("층과 위치를 골라", fx.announcements[-1]["message"])


class TestFloorConfirm(unittest.TestCase):
    def test_핫스팟이면_찍지_않고_사람에게_묻는다(self):
        # 노트북 핫스팟에 붙어 있으면 건물 SSID 가 안 보인다. 이게 정상 구성이다.
        fx = FakeEffects(guess=FloorGuess(None, "jongky", False, HOTSPOT,
                                          "핫스팟에 붙어 있어 층을 모릅니다"))
        book = make_book()
        gate = T.ScriptedGate([T.CALLED, T.BOARDED, T.ARRIVED, T.EXITED,
                               (T.PICK_FLOOR, {"floor": "11f"})])
        m = T.FloorTransfer(book, fx, gate=gate)
        out = m.run("10f", "11f", "11a")
        self.assertEqual(out.state, T.DONE)
        self.assertIn((T.PICK_FLOOR, T.ABORT), gate.asked)
        asked = [a for a in fx.announcements if a["state"] == T.CONFIRM_FLOOR]
        self.assertTrue(any("핫스팟" in a["message"] for a in asked))

    def test_층을_모르는_채로_시간이_지나면_정지(self):
        fx = FakeEffects(guess=FloorGuess(None, None, False, HOTSPOT, "모름"))
        m, fx, out = run([T.CALLED, T.BOARDED, T.ARRIVED, T.EXITED, T.TIMEOUT],
                         effects=fx)
        self.assertEqual(out.state, T.FAULT)
        self.assertNotIn("load_map", [c[0] for c in fx.calls])

    def test_사람이_모르는_층을_고르면_정지(self):
        fx = FakeEffects(guess=FloorGuess(None, None, False, HOTSPOT, "모름"))
        gate = T.ScriptedGate([T.CALLED, T.BOARDED, T.ARRIVED, T.EXITED,
                               (T.PICK_FLOOR, {"floor": "99f"})])
        m = T.FloorTransfer(make_book(), fx, gate=gate)
        out = m.run("10f", "11f", "11a")
        self.assertEqual(out.state, T.FAULT)


class TestWrongFloor(unittest.TestCase):
    """엉뚱한 층에 내렸다. SSID 는 그것을 안다."""

    def test_지도는_실제_층으로_갈고_다시_태워_달라고_한다(self):
        fx = FakeEffects(guess=FloorGuess("10f", "FASTCAMPUS_10F", True, MATCHED,
                                          "무선으로 10층"))
        gate = T.ScriptedGate([T.CALLED, T.BOARDED, T.ARRIVED, T.EXITED,
                               T.CALLED, T.BOARDED, T.ARRIVED, T.EXITED])
        m = T.FloorTransfer(make_book(), fx, gate=gate)
        out = m.run("10f", "11f", "11a")
        # 두 번 다 10층으로 판정 → 한도(2회) 초과로 정지
        self.assertEqual(out.state, T.FAULT)
        self.assertIn("10층", out.reason)
        # 그래도 지도는 **실제 서 있는 층**으로 갈아 뒀다 (거짓 위치로 두지 않는다)
        self.assertEqual(fx.loaded_maps, ["10f", "10f"])
        self.assertIn(T.WRONG_FLOOR, [h[1] for h in out.history])

    def test_두_번째에_맞으면_이어간다(self):
        class Flip(FakeEffects):
            n = 0

            def detect_floor(self):
                Flip.n += 1
                key = "10f" if Flip.n == 1 else "11f"
                return FloorGuess(key, "S", True, MATCHED, "")

        Flip.n = 0
        fx = Flip()
        gate = T.ScriptedGate([T.CALLED, T.BOARDED, T.ARRIVED, T.EXITED,
                               T.CALLED, T.BOARDED, T.ARRIVED, T.EXITED])
        m = T.FloorTransfer(make_book(), fx, gate=gate)
        out = m.run("10f", "11f", "11a")
        self.assertEqual(out.state, T.DONE)
        self.assertEqual(fx.loaded_maps, ["10f", "11f"])
        # 다시 탈 때는 엘리베이터 앞까지 또 주행하지 않는다 — 이미 거기 있다
        self.assertEqual(fx.calls.count(("navigate", "10f", "ev1")), 1)


class TestTable(unittest.TestCase):
    """전이표 자체의 무결성."""

    def test_모든_전이가_아는_상태로_간다(self):
        known = set(T.TERMINAL) | {
            T.IDLE, T.TO_ELEVATOR, T.AT_ELEVATOR, T.BOARDING, T.RIDING,
            T.EXITING, T.CONFIRM_FLOOR, T.SWAP_MAP, T.RELOCALIZE, T.LANDED,
            T.RESUME}
        for (src, ev), dst in T.TRANSITIONS.items():
            self.assertIn(src, known, f"{src} 가 상태 목록에 없다")
            self.assertIn(dst, known, f"{dst} 가 상태 목록에 없다")

    def test_처리기가_없는_상태로_가지_않는다(self):
        for (_src, _ev), dst in T.TRANSITIONS.items():
            if dst not in T.TERMINAL:
                self.assertIn(dst, T.FloorTransfer._HANDLERS,
                              f"{dst} 에 처리기가 없다")

    def test_모든_비종단_상태에_실패_출구가_있다(self):
        for state in T.FloorTransfer._HANDLERS:
            outs = {ev: dst for (s, ev), dst in T.TRANSITIONS.items() if s == state}
            self.assertTrue(
                any(dst in (T.FAULT, T.ABORTED) for dst in outs.values()),
                f"{state} 에서 빠져나갈 실패 경로가 없다")

    def test_모든_상태가_시작에서_도달_가능하다(self):
        seen, stack = {T.IDLE}, [T.IDLE]
        while stack:
            s = stack.pop()
            for (src, _ev), dst in T.TRANSITIONS.items():
                if src == s and dst not in seen:
                    seen.add(dst)
                    stack.append(dst)
        for state in list(T.FloorTransfer._HANDLERS) + list(T.TERMINAL):
            self.assertIn(state, seen, f"{state} 에 도달할 수 없다")

    def test_사람이_누르는_사건에는_버튼_이름이_있다(self):
        for ev in T.HUMAN_EVENTS:
            self.assertIn(ev, T.BUTTON_LABEL)


class TestGate(unittest.TestCase):
    def test_허용하지_않은_사건은_거부한다(self):
        g = T.Gate()
        ok, err = g.post(T.BOARDED)
        self.assertFalse(ok)
        self.assertIn("받을 수 없습니다", err)

    def test_기다리는_동안_들어온_것만_받는다(self):
        g = T.Gate()
        import threading
        result = {}

        def worker():
            result["ev"] = g.wait((T.CALLED, T.ABORT), timeout=3.0)

        t = threading.Thread(target=worker)
        t.start()
        # 창구가 열릴 때까지 잠깐 기다린다
        for _ in range(100):
            if g.allowed:
                break
            import time
            time.sleep(0.01)
        self.assertFalse(g.post(T.BOARDED)[0])
        self.assertTrue(g.post(T.CALLED)[0])
        t.join(3.0)
        self.assertEqual(result["ev"][0], T.CALLED)

    def test_아무도_안_누르면_시간이_지난다(self):
        g = T.Gate()
        ev, _ = g.wait((T.CALLED,), timeout=0.05)
        self.assertEqual(ev, T.TIMEOUT)


if __name__ == "__main__":
    unittest.main()
