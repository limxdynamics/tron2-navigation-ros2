#!/usr/bin/env python3
# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

"""Interactively select and verify a matched localization PCD/PCT map pair."""

import argparse
import hashlib
import json
import os
import pickle
import shutil
import uuid
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


@dataclass(frozen=True)
class MapPair:
    name: str
    pcd: Path
    pct_source: Path
    manifest: Path
    tomogram: Path
    source_sha256: str
    localization_sha256: str
    modified_time: float


def sha256_file(path, chunk_size=4 * 1024 * 1024):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        while True:
            chunk = stream.read(chunk_size)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def load_tomogram_identity(path):
    with open(path, "rb") as stream:
        payload = pickle.load(stream)
    if not isinstance(payload, dict):
        raise ValueError("tomogram payload is not a dictionary")
    source_sha256 = payload.get("source_pcd_sha256")
    source_name = payload.get("source_pcd_name")
    if not isinstance(source_sha256, str) or len(source_sha256) != 64:
        raise ValueError("tomogram has no valid source_pcd_sha256")
    if not isinstance(source_name, str) or not source_name:
        raise ValueError("tomogram has no valid source_pcd_name")
    return source_sha256.lower(), Path(source_name).name


def load_navigation_identity(path):
    with open(path, "rb") as stream:
        payload = pickle.load(stream)
    if not isinstance(payload, dict):
        raise ValueError("tomogram payload is not a dictionary")
    source_sha256 = payload.get("source_pcd_sha256")
    source_name = payload.get("source_pcd_name")
    if not isinstance(source_sha256, str) or len(source_sha256) != 64:
        raise ValueError("tomogram has no valid source_pcd_sha256")
    if not isinstance(source_name, str) or not source_name:
        raise ValueError("tomogram has no valid source_pcd_name")

    localization_sha256 = payload.get("localization_pcd_sha256")
    localization_name = payload.get("localization_pcd_name")
    if localization_sha256 is None and localization_name is None:
        localization_sha256 = source_sha256
        localization_name = source_name
    elif (
        not isinstance(localization_sha256, str)
        or len(localization_sha256) != 64
        or not isinstance(localization_name, str)
        or not localization_name
    ):
        raise ValueError("tomogram has invalid localization PCD identity")

    return {
        "source_sha256": source_sha256.lower(),
        "source_name": Path(source_name).name,
        "localization_sha256": localization_sha256.lower(),
        "localization_name": Path(localization_name).name,
        "schema_version": payload.get("navigation_map_schema_version"),
        "mapping_session_name": payload.get("mapping_session_name"),
        "mapping_coordinate_frame": payload.get("mapping_coordinate_frame"),
        "localization_max_range_m": payload.get("localization_max_range_m"),
        "pct_source_max_range_m": payload.get("pct_source_max_range_m"),
        "registration_max_range_m": payload.get("registration_max_range_m"),
    }


def load_pair_manifest(path):
    with open(path, "r", encoding="utf-8") as stream:
        manifest = json.load(stream)
    if not isinstance(manifest, dict):
        raise ValueError("map-pair manifest is not an object")
    if manifest.get("schema_version") != 1:
        raise ValueError("map-pair manifest has unsupported schema_version")
    if manifest.get("same_mapping_session") is not True:
        raise ValueError("map-pair manifest does not assert one mapping session")
    if manifest.get("coordinate_frame") != "camera_init":
        raise ValueError("map-pair manifest has an invalid coordinate frame")
    if not isinstance(manifest.get("map_name"), str) or not manifest["map_name"]:
        raise ValueError("map-pair manifest has no map_name")

    for key in ("localization_pcd", "pct_source_pcd"):
        entry = manifest.get(key)
        if not isinstance(entry, dict):
            raise ValueError("map-pair manifest has no %s object" % key)
        if not isinstance(entry.get("name"), str) or not entry["name"]:
            raise ValueError("map-pair manifest has no %s name" % key)
        digest = entry.get("sha256")
        if not isinstance(digest, str) or len(digest) != 64:
            raise ValueError("map-pair manifest has an invalid %s hash" % key)
        try:
            entry["max_range_m"] = float(entry["max_range_m"])
        except (KeyError, TypeError, ValueError):
            raise ValueError("map-pair manifest has an invalid %s range" % key)
        if entry["max_range_m"] <= 0.0:
            raise ValueError("map-pair manifest ranges must be positive")
    if (
        manifest["pct_source_pcd"]["max_range_m"]
        > manifest["localization_pcd"]["max_range_m"]
    ):
        raise ValueError("PCT source range exceeds localization range")
    registration_range = manifest.get("registration_max_range_m")
    if registration_range is not None:
        try:
            registration_range = float(registration_range)
        except (TypeError, ValueError):
            raise ValueError("map-pair manifest has an invalid registration range")
        if not (
            0.2
            < registration_range
            <= manifest["localization_pcd"]["max_range_m"]
        ):
            raise ValueError("map-pair manifest has an invalid registration range")
        manifest["registration_max_range_m"] = registration_range
    return manifest


def verify_pair_manifest(path, identity):
    manifest = load_pair_manifest(path)
    localization = manifest["localization_pcd"]
    pct_source = manifest["pct_source_pcd"]
    checks = (
        (Path(localization["name"]).name, identity["localization_name"], "localization name"),
        (localization["sha256"].lower(), identity["localization_sha256"], "localization hash"),
        (Path(pct_source["name"]).name, identity["source_name"], "PCT source name"),
        (pct_source["sha256"].lower(), identity["source_sha256"], "PCT source hash"),
        (manifest["map_name"], identity["mapping_session_name"], "mapping session"),
        (manifest["coordinate_frame"], identity["mapping_coordinate_frame"], "coordinate frame"),
        (localization["max_range_m"], identity["localization_max_range_m"], "localization range"),
        (pct_source["max_range_m"], identity["pct_source_max_range_m"], "PCT source range"),
    )
    if identity["schema_version"] != 2:
        raise ValueError("dual-map tomogram has an invalid schema version")
    if manifest.get("registration_max_range_m") is not None:
        checks += (
            (
                manifest["registration_max_range_m"],
                identity["registration_max_range_m"],
                "registration range",
            ),
        )
    for actual, expected, label in checks:
        if actual != expected:
            raise ValueError("map-pair manifest/tomogram %s mismatch" % label)
    return manifest


def pct_source_path_for(localization_pcd):
    localization_pcd = Path(localization_pcd)
    if localization_pcd.name == "map.pcd":
        return localization_pcd.with_name("map.pct-source.pcd")
    return localization_pcd.with_name(
        localization_pcd.stem + ".pct-source.pcd"
    )


def discover_map_pairs(root):
    root = Path(root).resolve()
    pcd_dir = root / "PCT_planner-RC2026_Map_Planner" / "rsc" / "pcd"
    tomogram_dir = (
        root / "PCT_planner-RC2026_Map_Planner" / "rsc" / "tomogram"
    )
    pairs = []
    for tomogram in tomogram_dir.glob("*.pickle"):
        if tomogram.name == "map.pickle" or ".legacy-" in tomogram.name:
            continue
        try:
            identity = load_navigation_identity(tomogram)
        except Exception:
            continue

        localization_candidates = []
        if tomogram.stem.startswith("fast_lio_"):
            localization_candidates.append(
                pcd_dir / (tomogram.stem + ".pcd")
            )
        localization_candidates.append(
            pcd_dir / identity["localization_name"]
        )
        pcd = next(
            (candidate for candidate in localization_candidates if candidate.is_file()),
            None,
        )
        if pcd is None:
            continue

        if identity["localization_sha256"] == identity["source_sha256"]:
            pct_source = pcd
            manifest = None
        else:
            pct_source_candidates = []
            if tomogram.stem.startswith("fast_lio_"):
                pct_source_candidates.append(
                    pcd_dir / (tomogram.stem + ".pct-source.pcd")
                )
            pct_source_candidates.append(pcd_dir / identity["source_name"])
            pct_source = next(
                (
                    candidate
                    for candidate in pct_source_candidates
                    if candidate.is_file()
                ),
                None,
            )
            if pct_source is None:
                continue
            manifest = pcd_dir / (tomogram.stem + ".map-pair.json")
            if not manifest.is_file():
                continue
            try:
                verify_pair_manifest(manifest, identity)
            except Exception:
                continue
        pairs.append(
            MapPair(
                name=tomogram.stem,
                pcd=pcd,
                pct_source=pct_source,
                manifest=manifest,
                tomogram=tomogram,
                source_sha256=identity["source_sha256"],
                localization_sha256=identity["localization_sha256"],
                modified_time=max(
                    pcd.stat().st_mtime,
                    pct_source.stat().st_mtime,
                    tomogram.stat().st_mtime,
                ),
            )
        )
    pairs.sort(key=lambda pair: (pair.modified_time, pair.name), reverse=True)
    return pairs


def verify_pair(pcd, tomogram, pct_source=None, pair_manifest=None):
    pcd = Path(pcd)
    tomogram = Path(tomogram)
    if not pcd.is_file():
        raise FileNotFoundError(pcd)
    if not tomogram.is_file():
        raise FileNotFoundError(tomogram)
    identity = load_navigation_identity(tomogram)
    dual_map = identity["localization_sha256"] != identity["source_sha256"]
    if dual_map:
        if pair_manifest is None:
            pair_manifest = pcd.with_name(pcd.stem + ".map-pair.json")
        pair_manifest = Path(pair_manifest)
        if not pair_manifest.is_file():
            raise FileNotFoundError(pair_manifest)
        verify_pair_manifest(pair_manifest, identity)
    actual_sha256 = sha256_file(pcd)
    if actual_sha256 != identity["localization_sha256"]:
        raise ValueError(
            "Localization PCD/PCT mismatch: PCD sha256=%s, expected sha256=%s"
            % (actual_sha256, identity["localization_sha256"])
        )
    if dual_map:
        if pct_source is None:
            pct_source = pct_source_path_for(pcd)
        pct_source = Path(pct_source)
        if not pct_source.is_file():
            raise FileNotFoundError(pct_source)
        actual_source_sha256 = sha256_file(pct_source)
        if actual_source_sha256 != identity["source_sha256"]:
            raise ValueError(
                "PCT source PCD/PCT mismatch: PCD sha256=%s, expected sha256=%s"
                % (actual_source_sha256, identity["source_sha256"])
            )
    return actual_sha256, identity["source_name"]


def replace_file_set(copies, removals=(), validator=None):
    """Stage, replace and validate a file set, rolling back on any failure."""
    transaction = uuid.uuid4().hex
    staged = {}
    backups = {}
    destinations = [Path(destination) for _, destination in copies]
    destinations.extend(Path(destination) for destination in removals)
    if len(set(destinations)) != len(destinations):
        raise ValueError("duplicate destination in active map transaction")
    try:
        for source, destination in copies:
            source = Path(source)
            destination = Path(destination)
            temporary = destination.with_name(
                "%s.new.%s" % (destination.name, transaction)
            )
            if temporary.exists():
                temporary.unlink()
            shutil.copy2(source, temporary)
            temporary.chmod(0o664)
            staged[destination] = temporary

        for destination in destinations:
            backup = destination.with_name(
                "%s.backup.%s" % (destination.name, transaction)
            )
            if backup.exists():
                backup.unlink()
            if destination.exists():
                os.replace(destination, backup)
                backups[destination] = backup

        for destination, temporary in staged.items():
            os.replace(temporary, destination)
        if validator is not None:
            return validator()
    except Exception:
        for destination in staged:
            if destination.exists():
                destination.unlink()
        for destination, backup in backups.items():
            if backup.exists():
                os.replace(backup, destination)
        raise
    finally:
        for temporary in staged.values():
            if temporary.exists():
                temporary.unlink()
        for backup in backups.values():
            if backup.exists():
                backup.unlink()


def activate_pair(root, pair):
    root = Path(root).resolve()
    active_pcd = root / "PCT_planner-RC2026_Map_Planner" / "rsc" / "pcd" / "map.pcd"
    active_pct_source = (
        root
        / "PCT_planner-RC2026_Map_Planner"
        / "rsc"
        / "pcd"
        / "map.pct-source.pcd"
    )
    active_pair_manifest = active_pct_source.with_name("map.map-pair.json")
    active_tomogram = (
        root
        / "PCT_planner-RC2026_Map_Planner"
        / "rsc"
        / "tomogram"
        / "map.pickle"
    )
    actual_sha256, _ = verify_pair(
        pair.pcd, pair.tomogram, pair.pct_source, pair.manifest
    )
    dual_map = pair.localization_sha256 != pair.source_sha256
    copies = [(pair.pcd, active_pcd), (pair.tomogram, active_tomogram)]
    removals = []
    if dual_map:
        copies.extend(
            (
                (pair.pct_source, active_pct_source),
                (pair.manifest, active_pair_manifest),
            )
        )
    else:
        removals.extend((active_pct_source, active_pair_manifest))

    def verify_active_pair():
        active_sha256, source_name = verify_pair(
            active_pcd,
            active_tomogram,
            active_pct_source if dual_map else None,
            active_pair_manifest if dual_map else None,
        )
        if active_sha256 != actual_sha256:
            raise RuntimeError("active map verification changed unexpectedly")
        return active_sha256, source_name

    active_sha256, source_name = replace_file_set(
        copies, removals, validator=verify_active_pair
    )
    print("NAVIGATION_MAP_SELECTED=%s" % pair.name)
    print("NAVIGATION_MAP_SOURCE_PCD=%s" % source_name)
    print("NAVIGATION_MAP_MODE=%s" % ("dual" if dual_map else "single"))
    if dual_map:
        print("NAVIGATION_PCT_SOURCE_PCD=%s" % pair.pct_source.name)
    print("NAVIGATION_MAP_SHA256=%s" % active_sha256)


def print_active_identity(root):
    root = Path(root).resolve()
    active_pcd = root / "PCT_planner-RC2026_Map_Planner" / "rsc" / "pcd" / "map.pcd"
    active_pct_source = (
        root
        / "PCT_planner-RC2026_Map_Planner"
        / "rsc"
        / "pcd"
        / "map.pct-source.pcd"
    )
    active_tomogram = (
        root
        / "PCT_planner-RC2026_Map_Planner"
        / "rsc"
        / "tomogram"
        / "map.pickle"
    )
    identity = load_navigation_identity(active_tomogram)
    dual_map = identity["localization_sha256"] != identity["source_sha256"]
    actual_sha256, source_name = verify_pair(
        active_pcd,
        active_tomogram,
        active_pct_source if dual_map else None,
        active_pcd.with_name("map.map-pair.json") if dual_map else None,
    )
    timestamp = datetime.fromtimestamp(
        max(active_pcd.stat().st_mtime, active_tomogram.stat().st_mtime)
    ).strftime("%Y-%m-%d %H:%M:%S")
    print("NAVIGATION_MAP_SELECTED=current")
    print("NAVIGATION_MAP_SOURCE_PCD=%s" % source_name)
    print("NAVIGATION_MAP_MODE=%s" % ("dual" if dual_map else "single"))
    if dual_map:
        print("NAVIGATION_PCT_SOURCE_PCD=%s" % active_pct_source.name)
    print("NAVIGATION_MAP_TIME=%s" % timestamp)
    print("NAVIGATION_MAP_SHA256=%s" % actual_sha256)


def select_interactively(root, pairs):
    print("\n可用导航地图（PCD 与 PCT 已配对）：")
    print("  0) 保持当前生效地图")
    for index, pair in enumerate(pairs, start=1):
        timestamp = datetime.fromtimestamp(pair.modified_time).strftime(
            "%Y-%m-%d %H:%M:%S"
        )
        latest = " [最新]" if index == 1 else ""
        mode = (
            "双图：完整定位 + 近场 PCT"
            if pair.localization_sha256 != pair.source_sha256
            else "单图配对"
        )
        print(
            "  %d) %s  %s  %s%s"
            % (index, pair.name, timestamp, mode, latest)
        )
    while True:
        answer = input("选择地图编号 [0]: ").strip() or "0"
        try:
            index = int(answer)
        except ValueError:
            print("请输入列表中的数字。")
            continue
        if index == 0:
            print_active_identity(root)
            return
        if 1 <= index <= len(pairs):
            activate_pair(root, pairs[index - 1])
            return
        print("地图编号超出范围。")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root", default=str(Path(__file__).resolve().parent)
    )
    parser.add_argument(
        "--mode", choices=("prompt", "current", "latest"), default="prompt"
    )
    parser.add_argument("--name", default="")
    return parser.parse_args()


def main():
    args = parse_args()
    pairs = discover_map_pairs(args.root)
    if args.name:
        match = next((pair for pair in pairs if pair.name == args.name), None)
        if match is None:
            raise ValueError("unknown or incomplete map pair: %s" % args.name)
        activate_pair(args.root, match)
    elif args.mode == "current":
        print_active_identity(args.root)
    elif args.mode == "latest":
        if not pairs:
            raise RuntimeError("no complete named map pair was found")
        activate_pair(args.root, pairs[0])
    else:
        select_interactively(args.root, pairs)


if __name__ == "__main__":
    main()
