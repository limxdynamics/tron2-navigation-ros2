# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

import hashlib
import importlib.util
import json
import pickle
import tempfile
import unittest
from pathlib import Path
from unittest import mock


MODULE_PATH = Path(__file__).resolve().parents[1] / "select_navigation_map.py"
SPEC = importlib.util.spec_from_file_location("select_navigation_map", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class NavigationMapSelectorTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.pcd_dir = (
            self.root / "PCT_planner-RC2026_Map_Planner" / "rsc" / "pcd"
        )
        self.tomogram_dir = (
            self.root
            / "PCT_planner-RC2026_Map_Planner"
            / "rsc"
            / "tomogram"
        )
        self.pcd_dir.mkdir(parents=True)
        self.tomogram_dir.mkdir(parents=True)

    def tearDown(self):
        self.temporary_directory.cleanup()

    def create_pair(self, name, content):
        pcd = self.pcd_dir / (name + ".pcd")
        tomogram = self.tomogram_dir / (name + ".pickle")
        pcd.write_bytes(content)
        digest = hashlib.sha256(content).hexdigest()
        with open(tomogram, "wb") as stream:
            pickle.dump(
                {
                    "source_pcd_name": "rs_fairy_map.pcd",
                    "source_pcd_sha256": digest,
                },
                stream,
            )
        return pcd, tomogram, digest

    def create_dual_pair(self, name, localization_content, pct_content):
        pcd = self.pcd_dir / (name + ".pcd")
        pct_source = self.pcd_dir / (name + ".pct-source.pcd")
        tomogram = self.tomogram_dir / (name + ".pickle")
        manifest = self.pcd_dir / (name + ".map-pair.json")
        pcd.write_bytes(localization_content)
        pct_source.write_bytes(pct_content)
        localization_digest = hashlib.sha256(localization_content).hexdigest()
        pct_digest = hashlib.sha256(pct_content).hexdigest()
        with open(tomogram, "wb") as stream:
            pickle.dump(
                {
                    "source_pcd_name": "mapping.pct-source.pcd",
                    "source_pcd_sha256": pct_digest,
                    "localization_pcd_name": "mapping.pcd",
                    "localization_pcd_sha256": localization_digest,
                    "navigation_map_schema_version": 2,
                    "mapping_session_name": "mapping",
                    "mapping_coordinate_frame": "camera_init",
                    "localization_max_range_m": 100.0,
                    "pct_source_max_range_m": 15.0,
                    "registration_max_range_m": 15.0,
                },
                stream,
            )
        manifest.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "map_name": "mapping",
                    "coordinate_frame": "camera_init",
                    "same_mapping_session": True,
                    "registration_max_range_m": 15.0,
                    "localization_pcd": {
                        "name": "mapping.pcd",
                        "sha256": localization_digest,
                        "max_range_m": 100.0,
                    },
                    "pct_source_pcd": {
                        "name": "mapping.pct-source.pcd",
                        "sha256": pct_digest,
                        "max_range_m": 15.0,
                    },
                }
            ),
            encoding="utf-8",
        )
        return pcd, pct_source, tomogram, localization_digest, pct_digest

    def test_discovers_and_activates_matching_fast_lio_pair(self):
        pcd, tomogram, digest = self.create_pair("fast_lio_test", b"new-map")
        pairs = MODULE.discover_map_pairs(self.root)
        self.assertEqual([pair.name for pair in pairs], ["fast_lio_test"])

        MODULE.activate_pair(self.root, pairs[0])

        active_pcd = self.pcd_dir / "map.pcd"
        active_tomogram = self.tomogram_dir / "map.pickle"
        self.assertEqual(active_pcd.read_bytes(), pcd.read_bytes())
        self.assertEqual(active_tomogram.read_bytes(), tomogram.read_bytes())
        self.assertEqual(MODULE.verify_pair(active_pcd, active_tomogram)[0], digest)

    def test_rejects_pcd_tomogram_hash_mismatch(self):
        pcd, tomogram, _ = self.create_pair("fast_lio_bad", b"expected")
        pcd.write_bytes(b"different")
        with self.assertRaisesRegex(ValueError, "PCD/PCT mismatch"):
            MODULE.verify_pair(pcd, tomogram)

    def test_discovers_verifies_and_activates_dual_map_pair(self):
        pcd, pct_source, tomogram, localization_digest, _ = (
            self.create_dual_pair(
                "fast_lio_dual", b"full-range-map", b"near-field-map"
            )
        )

        pairs = MODULE.discover_map_pairs(self.root)
        self.assertEqual([pair.name for pair in pairs], ["fast_lio_dual"])
        self.assertEqual(pairs[0].pcd, pcd)
        self.assertEqual(pairs[0].pct_source, pct_source)

        verified = MODULE.verify_pair(pcd, tomogram, pct_source)
        self.assertEqual(verified[0], localization_digest)
        MODULE.activate_pair(self.root, pairs[0])

        active_pcd = self.pcd_dir / "map.pcd"
        active_pct_source = self.pcd_dir / "map.pct-source.pcd"
        active_tomogram = self.tomogram_dir / "map.pickle"
        active_manifest = self.pcd_dir / "map.map-pair.json"
        self.assertEqual(active_pcd.read_bytes(), b"full-range-map")
        self.assertEqual(active_pct_source.read_bytes(), b"near-field-map")
        self.assertEqual(active_tomogram.read_bytes(), tomogram.read_bytes())
        self.assertTrue(active_manifest.is_file())
        MODULE.verify_pair(
            active_pcd, active_tomogram, active_pct_source, active_manifest
        )

    def test_rejects_dual_map_pct_source_hash_mismatch(self):
        pcd, pct_source, tomogram, _, _ = self.create_dual_pair(
            "fast_lio_dual_bad", b"full-range-map", b"near-field-map"
        )
        pct_source.write_bytes(b"wrong-near-field-map")

        with self.assertRaisesRegex(ValueError, "PCT source PCD/PCT mismatch"):
            MODULE.verify_pair(pcd, tomogram, pct_source)

    def test_rejects_dual_map_manifest_mismatch(self):
        pcd, pct_source, tomogram, _, _ = self.create_dual_pair(
            "fast_lio_dual_manifest_bad",
            b"full-range-map",
            b"near-field-map",
        )
        manifest = self.pcd_dir / "fast_lio_dual_manifest_bad.map-pair.json"
        payload = json.loads(manifest.read_text(encoding="utf-8"))
        payload["same_mapping_session"] = False
        manifest.write_text(json.dumps(payload), encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "one mapping session"):
            MODULE.verify_pair(pcd, tomogram, pct_source, manifest)
        self.assertEqual(MODULE.discover_map_pairs(self.root), [])

    def test_rejects_dual_map_registration_range_mismatch(self):
        pcd, pct_source, tomogram, _, _ = self.create_dual_pair(
            "fast_lio_dual_registration_bad",
            b"full-range-map",
            b"near-field-map",
        )
        manifest = self.pcd_dir / "fast_lio_dual_registration_bad.map-pair.json"
        payload = json.loads(manifest.read_text(encoding="utf-8"))
        payload["registration_max_range_m"] = 30.0
        manifest.write_text(json.dumps(payload), encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "registration range mismatch"):
            MODULE.verify_pair(pcd, tomogram, pct_source, manifest)
        self.assertEqual(MODULE.discover_map_pairs(self.root), [])

    def test_activating_legacy_pair_removes_stale_dual_source(self):
        stale = self.pcd_dir / "map.pct-source.pcd"
        stale_manifest = self.pcd_dir / "map.map-pair.json"
        stale.write_bytes(b"stale")
        stale_manifest.write_text("{}", encoding="utf-8")
        self.create_pair("fast_lio_legacy", b"legacy-map")

        pair = MODULE.discover_map_pairs(self.root)[0]
        MODULE.activate_pair(self.root, pair)

        self.assertFalse(stale.exists())
        self.assertFalse(stale_manifest.exists())

    def test_file_set_replacement_rolls_back_on_commit_failure(self):
        first = self.root / "first.active"
        second = self.root / "second.active"
        first_source = self.root / "first.new"
        second_source = self.root / "second.new"
        first.write_bytes(b"old-first")
        second.write_bytes(b"old-second")
        first_source.write_bytes(b"new-first")
        second_source.write_bytes(b"new-second")
        real_replace = MODULE.os.replace
        failed = False

        def fail_second_install(source, destination):
            nonlocal failed
            if (
                not failed
                and ".new." in Path(source).name
                and Path(destination) == second
            ):
                failed = True
                raise OSError("simulated commit failure")
            return real_replace(source, destination)

        with mock.patch.object(MODULE.os, "replace", side_effect=fail_second_install):
            with self.assertRaisesRegex(OSError, "simulated commit failure"):
                MODULE.replace_file_set(
                    ((first_source, first), (second_source, second))
                )

        self.assertEqual(first.read_bytes(), b"old-first")
        self.assertEqual(second.read_bytes(), b"old-second")

    def test_file_set_replacement_rolls_back_on_validation_failure(self):
        active = self.root / "map.active"
        source = self.root / "map.new"
        stale = self.root / "stale.active"
        active.write_bytes(b"old-map")
        source.write_bytes(b"new-map")
        stale.write_bytes(b"old-stale")

        def reject_replacement():
            self.assertEqual(active.read_bytes(), b"new-map")
            self.assertFalse(stale.exists())
            raise ValueError("simulated validation failure")

        with self.assertRaisesRegex(ValueError, "simulated validation failure"):
            MODULE.replace_file_set(
                ((source, active),), (stale,), validator=reject_replacement
            )

        self.assertEqual(active.read_bytes(), b"old-map")
        self.assertEqual(stale.read_bytes(), b"old-stale")


if __name__ == "__main__":
    unittest.main()
