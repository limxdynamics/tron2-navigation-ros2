#!/usr/bin/env python3
# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

"""List and select source PCD files for FAST-LIO/PCT map preparation."""

import argparse
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


@dataclass(frozen=True)
class PcdCandidate:
    path: Path
    relative_path: str
    modified_time: float
    size: int
    point_count: int
    description: str


def read_pcd_point_count(path):
    """Read POINTS (or WIDTH) from the PCD header without loading point data."""
    points = 0
    width = 0
    with open(path, "rb") as stream:
        for _ in range(256):
            line = stream.readline(4096)
            if not line:
                break
            text = line.decode("ascii", errors="ignore").strip()
            fields = text.split()
            if len(fields) >= 2 and fields[0].upper() == "POINTS":
                try:
                    points = int(fields[1])
                except ValueError:
                    pass
            elif len(fields) >= 2 and fields[0].upper() == "WIDTH":
                try:
                    width = int(fields[1])
                except ValueError:
                    pass
            elif fields and fields[0].upper() == "DATA":
                break
    return points or width


def format_size(size):
    value = float(size)
    for unit in ("B", "KiB", "MiB", "GiB", "TiB"):
        if value < 1024.0 or unit == "TiB":
            if unit == "B":
                return "%d %s" % (int(value), unit)
            return "%.1f %s" % (value, unit)
        value /= 1024.0
    return "%d B" % size


def discover_pcd_candidates(root):
    root = Path(root).resolve()
    fast_lio_dir = root / "FAST_LIO" / "PCD"
    archive_dir = (
        root / "PCT_planner-RC2026_Map_Planner" / "rsc" / "pcd"
    )
    tomogram_dir = (
        root / "PCT_planner-RC2026_Map_Planner" / "rsc" / "tomogram"
    )
    latest_saved = fast_lio_dir / "rs_fairy_map.pcd"
    candidates = []
    seen = set()

    def add_candidate(path, description):
        path = Path(path)
        if not path.is_file() or path.name.startswith("."):
            return
        resolved = path.resolve()
        if resolved in seen:
            return
        seen.add(resolved)
        stat = path.stat()
        try:
            relative_path = str(path.relative_to(root))
        except ValueError:
            relative_path = str(path)
        candidates.append(
            PcdCandidate(
                path=resolved,
                relative_path=relative_path,
                modified_time=stat.st_mtime,
                size=stat.st_size,
                point_count=read_pcd_point_count(path),
                description=description,
            )
        )

    if fast_lio_dir.is_dir():
        for path in fast_lio_dir.iterdir():
            if (
                path.suffix.lower() != ".pcd"
                or path.name == "scans.pcd"
                or path.name.endswith(".pct-source.pcd")
            ):
                continue
            if path.resolve() == latest_saved.resolve():
                description = "最新 FAST-LIO /map_save 保存"
            else:
                description = "FAST-LIO PCD"
            add_candidate(path, description)

    if archive_dir.is_dir():
        for path in archive_dir.iterdir():
            if (
                path.suffix.lower() != ".pcd"
                or path.name in ("map.pcd", "map.pct-source.pcd")
                or path.name.endswith(".pct-source.pcd")
            ):
                continue
            paired_tomogram = tomogram_dir / (path.stem + ".pickle")
            if paired_tomogram.is_file():
                description = "历史归档（已有配套 PCT，可直接导航）"
            else:
                description = "历史 PCD 归档"
            add_candidate(path, description)

    candidates.sort(
        key=lambda item: (
            item.modified_time,
            item.path == latest_saved.resolve(),
            item.path.name,
        ),
        reverse=True,
    )
    return candidates


def print_candidates(candidates, stream):
    print("\n可处理的 PCD 地图：", file=stream)
    if not candidates:
        print("  （没有找到 PCD 地图）", file=stream)
        return
    for index, candidate in enumerate(candidates, start=1):
        timestamp = datetime.fromtimestamp(candidate.modified_time).strftime(
            "%Y-%m-%d %H:%M:%S"
        )
        point_count = (
            format(candidate.point_count, ",") if candidate.point_count else "未知"
        )
        recommended = " [推荐]" if index == 1 else ""
        print(
            "  %d) %s | %s | %s | %s 点 | %s%s"
            % (
                index,
                candidate.path.name,
                timestamp,
                format_size(candidate.size),
                point_count,
                candidate.description,
                recommended,
            ),
            file=stream,
        )
        print("     %s" % candidate.relative_path, file=stream)
    print(
        "  注：scans.pcd 和 *.pct-source.pcd 默认排除；近场 PCD 只能通过双图清单与完整定位图配对。",
        file=stream,
    )


def select_interactively(candidates, input_stream=sys.stdin, output_stream=sys.stderr):
    print_candidates(candidates, output_stream)
    if not candidates:
        return None
    print("  0) 取消，不处理地图", file=output_stream)
    while True:
        print("选择待处理 PCD 编号 [1]: ", end="", file=output_stream, flush=True)
        answer = input_stream.readline()
        if answer == "":
            return None
        answer = answer.strip() or "1"
        try:
            index = int(answer)
        except ValueError:
            print("请输入列表中的数字。", file=output_stream)
            continue
        if index == 0:
            return None
        if 1 <= index <= len(candidates):
            return candidates[index - 1]
        print("PCD 编号超出范围。", file=output_stream)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root", default=str(Path(__file__).resolve().parent)
    )
    parser.add_argument("--mode", choices=("prompt", "latest", "list"), default="prompt")
    return parser.parse_args()


def main():
    args = parse_args()
    candidates = discover_pcd_candidates(args.root)
    if args.mode == "list":
        print_candidates(candidates, sys.stdout)
        return 0 if candidates else 1
    if not candidates:
        raise RuntimeError("no source PCD map was found")
    if args.mode == "latest":
        selected = candidates[0]
    else:
        selected = select_interactively(candidates)
        if selected is None:
            print("未选择 PCD，地图处理已取消。", file=sys.stderr)
            return 130
    print(str(selected.path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
