#!/usr/bin/env python3
"""층별 자원 대장 시험 — 지도와 waypoint 의 짝이 어긋난 것을 잡는가."""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from guide_mission.floors import (ERROR, WARN, FloorBook,  # noqa: E402
                                  read_map, read_waypoints)


def write_map(d, name, w=200, h=100, res=0.05, origin=(-5.0, -2.5)):
    """작은 pgm 한 장과 그 YAML. 200×100 픽셀 @0.05 = 10×5 m."""
    pgm = os.path.join(d, f"{name}.pgm")
    with open(pgm, "wb") as f:
        f.write(b"P5\n# jongky test map\n%d %d\n255\n" % (w, h))
        f.write(b"\xfe" * (w * h))
    y = os.path.join(d, f"{name}.yaml")
    with open(y, "w") as f:
        f.write(f"image: {name}.pgm\nresolution: {res}\n"
                f"origin: [{origin[0]}, {origin[1]}, 0.0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    return y


def write_wps(d, name, points):
    p = os.path.join(d, f"{name}.yaml")
    with open(p, "w") as f:
        for n, (x, y) in points.items():
            f.write(f"{n}:\n  position: {{x: {x}, y: {y}, z: 0.0}}\n"
                    f"  orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}\n"
                    f"  frame_id: map\n")
    return p


class TestMapReading(unittest.TestCase):
    def test_pgm_헤더에서_크기를_읽는다(self):
        with tempfile.TemporaryDirectory() as d:
            info = read_map(write_map(d, "m"))
            self.assertEqual((info.width, info.height), (200, 100))
            self.assertAlmostEqual(info.resolution, 0.05)
            self.assertEqual(info.bounds, (-5.0, -2.5, 5.0, 2.5))

    def test_주석이_있어도_읽는다(self):
        with tempfile.TemporaryDirectory() as d:
            # write_map 이 이미 주석을 넣는다. map_saver 가 그렇게 쓴다.
            info = read_map(write_map(d, "m"))
            self.assertEqual(info.width, 200)

    def test_이미지가_없으면_예외(self):
        with tempfile.TemporaryDirectory() as d:
            y = write_map(d, "m")
            os.remove(os.path.join(d, "m.pgm"))
            with self.assertRaises(FileNotFoundError):
                read_map(y)

    def test_지도_안팎_판정(self):
        with tempfile.TemporaryDirectory() as d:
            info = read_map(write_map(d, "m"))
            self.assertTrue(info.contains(0.0, 0.0))
            self.assertTrue(info.contains(4.9, 2.4))
            self.assertFalse(info.contains(40.0, 0.0))
            self.assertTrue(info.contains(5.5, 0.0, margin=1.0))


class TestPairing(unittest.TestCase):
    """짝이 어긋난 것을 잡는가 — 이게 이 파일의 존재 이유다."""

    def _book(self, d, wps10, wps11=None):
        write_map(d, "f10")
        write_map(d, "f11")
        write_wps(d, "w10", wps10)
        write_wps(d, "w11", wps11 or {"ev1": (0.0, 0.0), "11a": (1.0, 1.0)})
        doc = {"floors": {
            "10f": {"label": "10층", "map": f"{d}/f10.yaml",
                    "waypoints": f"{d}/w10.yaml", "ssid": ["FASTCAMPUS_10F"],
                    "elevator": {"board": "ev1", "exit": "ev1"}},
            "11f": {"label": "11층", "map": f"{d}/f11.yaml",
                    "waypoints": f"{d}/w11.yaml", "ssid": ["FASTCAMPUS_11F"],
                    "elevator": {"board": "ev1", "exit": "ev1"}},
        }}
        return FloorBook.from_dict(doc)

    def test_정상이면_문제가_없다(self):
        with tempfile.TemporaryDirectory() as d:
            book = self._book(d, {"ev1": (0.0, 0.0), "10a": (2.0, 1.0)})
            self.assertEqual([p for p in book.problems if p.level == ERROR], [])
            self.assertTrue(book.get("10f").usable)

    def test_다른_층_waypoint_를_얹으면_잡는다(self):
        # 10층 지도(10×5 m)에 좌표가 통째로 다른 waypoint 를 얹었다.
        with tempfile.TemporaryDirectory() as d:
            book = self._book(d, {"ev1": (80.0, 40.0), "10a": (85.0, 41.0)})
            errs = [p for p in book.get("10f").problems if p.level == ERROR]
            self.assertTrue(errs)
            self.assertFalse(book.get("10f").usable)
            self.assertTrue(any("지도 밖" in p.text for p in errs))

    def test_일부만_밖이면_경고다(self):
        with tempfile.TemporaryDirectory() as d:
            book = self._book(d, {"ev1": (0.0, 0.0), "10a": (2.0, 1.0),
                                  "10b": (99.0, 99.0)})
            fl = book.get("10f")
            self.assertTrue(any(p.level == WARN and "지도 밖" in p.text
                                for p in fl.problems))
            self.assertTrue(fl.usable)   # 엘리베이터 지점은 멀쩡하다

    def test_엘리베이터_지점이_밖이면_못_쓴다(self):
        with tempfile.TemporaryDirectory() as d:
            book = self._book(d, {"ev1": (99.0, 99.0), "10a": (2.0, 1.0)})
            self.assertFalse(book.get("10f").usable)

    def test_엘리베이터_지점이_없으면_못_쓴다(self):
        with tempfile.TemporaryDirectory() as d:
            book = self._book(d, {"10a": (2.0, 1.0)})
            fl = book.get("10f")
            self.assertFalse(fl.usable)
            self.assertTrue(any("waypoint 파일에 없다" in p.text for p in fl.errors))

    def test_waypoint_파일이_없으면_못_쓴다(self):
        with tempfile.TemporaryDirectory() as d:
            book = self._book(d, {"ev1": (0.0, 0.0)})
            os.remove(f"{d}/w11.yaml")
            book.validate()
            self.assertFalse(book.get("11f").usable)

    def test_오염된_이름을_경고한다(self):
        # 맵핑 중 w 연타가 새어 들어간 실제 사례 (teleop_key.py:228)
        with tempfile.TemporaryDirectory() as d:
            book = self._book(d, {"ev1": (0.0, 0.0), "wwwwwwwwwwev2": (1.0, 1.0)})
            fl = book.get("10f")
            self.assertTrue(any("오염" in p.text for p in fl.problems))
            self.assertTrue(fl.usable)   # 경고지 오류가 아니다


class TestLookup(unittest.TestCase):
    def _book(self):
        doc = {"hotspot_ssids": ["jongky"], "floors": {
            "10f": {"label": "10층", "ssid": ["FASTCAMPUS_10F"], "order": 10,
                    "elevator": {"board": "ev1"},
                    "labels": {"ev1": "엘리베이터 앞", "10a": "1004호"}},
            "11f": {"label": "11층", "ssid": ["FASTCAMPUS_11F"], "order": 11,
                    "elevator": {"board": "ev1"}},
        }}
        book = FloorBook.from_dict(doc, validate_now=False)
        from guide_mission.floors import Waypoint
        for k, names in (("10f", ("ev1", "10a")), ("11f", ("ev1", "11a"))):
            fl = book.get(k)
            fl.waypoints = {n: Waypoint(n, fl.label_of(n), 0.0, 0.0) for n in names}
        return book

    def test_ssid_로_층을_찾는다(self):
        book = self._book()
        self.assertEqual(book.by_ssid("FASTCAMPUS_11F").key, "11f")
        self.assertEqual(book.by_ssid("fastcampus_11f").key, "11f")  # 대소문자 무시
        self.assertIsNone(book.by_ssid("jongky"))
        self.assertIsNone(book.by_ssid(""))

    def test_exit_은_기본값이_board_다(self):
        self.assertEqual(self._book().get("11f").exit_wp, "ev1")

    def test_목적지_목록에_엘리베이터는_없다(self):
        names = [d["name"] for d in self._book().get("10f").destinations()]
        self.assertEqual(names, ["10a"])

    def test_표시_이름을_준다(self):
        self.assertEqual(self._book().get("10f").destinations()[0]["label"], "1004호")

    def test_어느_층_목적지인지_찾는다(self):
        book = self._book()
        self.assertEqual([f.key for f in book.find_destination("11a")], ["11f"])
        self.assertEqual([f.key for f in book.find_destination("ev1")], ["10f", "11f"])
        self.assertEqual(book.find_destination("없는곳"), [])

    def test_층이_하나면_경고한다(self):
        doc = {"floors": {"10f": {"label": "10층", "elevator": {"board": "ev1"}}}}
        book = FloorBook.from_dict(doc, validate_now=False)
        self.assertTrue(any("층이 하나뿐" in p.text for p in book.validate()))


class TestWaypointReading(unittest.TestCase):
    def test_읽는다(self):
        with tempfile.TemporaryDirectory() as d:
            p = write_wps(d, "w", {"a": (1.5, -2.5)})
            wps = read_waypoints(p, {"a": "가나다"})
            self.assertAlmostEqual(wps["a"].x, 1.5)
            self.assertAlmostEqual(wps["a"].y, -2.5)
            self.assertEqual(wps["a"].label, "가나다")
            self.assertEqual(wps["a"].frame_id, "map")

    def test_position_이_없으면_예외(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "w.yaml")
            with open(p, "w") as f:
                f.write("a:\n  orientation: {w: 1.0}\n")
            with self.assertRaises(ValueError):
                read_waypoints(p)


if __name__ == "__main__":
    unittest.main()
