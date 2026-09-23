# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

import importlib.util
import io
import os
import sys
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "select_pcd_map.py"
SPEC = importlib.util.spec_from_file_location("select_pcd_map", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class PcdMapSelectorTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.fast_lio_dir = self.root / "FAST_LIO" / "PCD"
        self.archive_dir = (
            self.root / "PCT_planner-RC2026_Map_Planner" / "rsc" / "pcd"
        )
        self.tomogram_dir = (
            self.root
            / "PCT_planner-RC2026_Map_Planner"
            / "rsc"
            / "tomogram"
        )
        self.fast_lio_dir.mkdir(parents=True)
        self.archive_dir.mkdir(parents=True)
        self.tomogram_dir.mkdir(parents=True)

    def tearDown(self):
        self.temporary_directory.cleanup()

    def create_pcd(self, path, points, modified_time):
        path.write_bytes(
            (
                "# .PCD v0.7\n"
                "FIELDS x y z\n"
                "SIZE 4 4 4\n"
                "TYPE F F F\n"
                "COUNT 1 1 1\n"
                "WIDTH %d\n"
                "HEIGHT 1\n"
                "POINTS %d\n"
                "DATA binary\n" % (points, points)
            ).encode("ascii")
        )
        os.utime(path, (modified_time, modified_time))
        return path

    def test_discovers_current_and_archived_maps_but_excludes_special_files(self):
        current = self.create_pcd(
            self.fast_lio_dir / "rs_fairy_map.pcd", 1234, 300
        )
        self.create_pcd(self.fast_lio_dir / "scans.pcd", 9999, 400)
        self.create_pcd(
            self.fast_lio_dir / "new-map.pct-source.pcd", 8888, 450
        )
        archived = self.create_pcd(
            self.archive_dir / "fast_lio_factory.pcd", 567, 200
        )
        self.create_pcd(
            self.archive_dir / "fast_lio_factory.pct-source.pcd", 777, 250
        )
        self.create_pcd(self.archive_dir / "map.pcd", 1234, 500)
        (self.tomogram_dir / "fast_lio_factory.pickle").write_bytes(b"paired")

        candidates = MODULE.discover_pcd_candidates(self.root)

        self.assertEqual([item.path for item in candidates], [current, archived])
        self.assertEqual(candidates[0].point_count, 1234)
        self.assertIn("最新 FAST-LIO", candidates[0].description)
        self.assertIn("已有配套 PCT", candidates[1].description)

    def test_interactive_selection_returns_requested_candidate(self):
        first = self.create_pcd(
            self.fast_lio_dir / "rs_fairy_map.pcd", 10, 200
        )
        second = self.create_pcd(
            self.archive_dir / "fast_lio_old.pcd", 20, 100
        )
        candidates = MODULE.discover_pcd_candidates(self.root)
        output = io.StringIO()

        selected = MODULE.select_interactively(
            candidates, input_stream=io.StringIO("2\n"), output_stream=output
        )

        self.assertEqual(selected.path, second)
        self.assertIn(first.name, output.getvalue())
        self.assertIn(second.name, output.getvalue())

    def test_point_count_falls_back_to_width(self):
        path = self.fast_lio_dir / "width_only.pcd"
        path.write_text("WIDTH 42\nHEIGHT 1\nDATA ascii\n", encoding="ascii")
        self.assertEqual(MODULE.read_pcd_point_count(path), 42)


if __name__ == "__main__":
    unittest.main()
