# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_fast_lio_dual_map_parameters_and_range_filter_are_wired():
    source_path = ROOT / "FAST_LIO" / "src" / "laserMapping.cpp"
    if not source_path.is_file():
        pytest.skip("FAST-LIO GPL sibling has not been linked")

    source = source_path.read_text(encoding="utf-8")
    config = read("FAST_LIO/config/rs_fairy.yaml")
    launch = read("FAST_LIO/launch/mapping.launch.py")
    mapping = read("run_fast_lio_mapping_humble.sh")

    assert '"pct_map_file_path"' in source
    assert '"pcd_save.pct_max_range"' in source
    assert "range_squared <= pct_max_range_squared" in source
    assert "pcl_wait_save_pct" in source
    assert "commit_prepared_pcds" in source
    assert "dual-map save requires pcd_save.use_all_frames=true" in source
    assert '"mapping.registration_max_range"' in source
    assert "filter_registration_cloud" in source
    assert "feats_registration_body" in source
    assert "downSizeFilterSurf.setInputCloud(registration_cloud)" in source
    assert "pcd_save_en && !map_save_completed" in source
    assert "registration_max_range: 15.0" in config
    assert "pct_map_file_path" in launch
    assert "pct_max_range" in launch
    assert "registration_max_range" in launch
    assert 'MAPPING_LOCALIZATION_MAX_DISTANCE="${MAPPING_LOCALIZATION_MAX_DISTANCE:-100}"' in mapping
    assert 'MAPPING_REGISTRATION_MAX_DISTANCE="${MAPPING_REGISTRATION_MAX_DISTANCE:-15}"' in mapping
    assert 'MAPPING_PCT_MAX_DISTANCE="${MAPPING_PCT_MAX_DISTANCE:-15}"' in mapping
    assert 'pct_map_file_path:="${PCT_MAP_OUTPUT}"' in mapping
    assert 'registration_max_range:="${MAPPING_REGISTRATION_MAX_DISTANCE}"' in mapping


def test_main_workflow_keeps_full_and_near_field_outputs_distinct():
    navi = read("navi.sh")
    preparer = read("prepare_fast_lio_pcd_map.sh")
    selector = read("select_navigation_map.py")

    assert ".pct-source.pcd" in navi
    assert ".map-pair.json" in navi
    assert "same_mapping_session" in navi
    assert "registration_max_range_m" in navi
    assert "--localization-pcd" in navi
    assert "--pair-manifest" in navi
    assert "Dual-map mode requires --pair-manifest" in preparer
    assert "localization_pcd_sha256" in preparer
    assert "navigation_map_schema_version" in preparer
    assert "localization_pcd_sha256" in selector
    assert "PCT source PCD/PCT mismatch" in selector
    assert "verify_pair_manifest" in selector
    assert "replace_file_set" in selector
    assert "NAVIGATION_MAP_SELECTOR" in preparer
