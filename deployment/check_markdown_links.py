#!/usr/bin/env python3
# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

"""Fail when Markdown files contain missing local link targets.

External URLs and fragment-only links are intentionally not fetched. The check
is deterministic and uses only the Python standard library.
"""

import argparse
import os
from pathlib import Path
import re
import sys
from typing import Iterable, List, Optional, Tuple
from urllib.parse import unquote, urlsplit


INLINE_LINK_RE = re.compile(
    r"!?\[[^\]\n]*\]\(\s*"
    r"(?P<target><[^>\n]+>|\"[^\"\n]+\"|'[^'\n]+'|[^\s)]+)"
)
REFERENCE_LINK_RE = re.compile(
    r"^\s*\[[^\]\n]+\]:\s*(?P<target><[^>\n]+>|\S+)", re.MULTILINE
)
HTML_LINK_RE = re.compile(
    r"<(?:a|img)\b[^>]*?\b(?:href|src)\s*=\s*"
    r"(?P<quote>['\"])(?P<target>.*?)(?P=quote)",
    re.IGNORECASE,
)
FENCE_RE = re.compile(r"^\s*(`{3,}|~{3,})")
HTML_COMMENT_RE = re.compile(r"<!--.*?-->", re.DOTALL)
EXTERNAL_SCHEME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9+.-]*:")


def _without_ignored_regions(text: str) -> str:
    """Blank fenced blocks and HTML comments while preserving line numbers."""
    output: List[str] = []
    fence_character: Optional[str] = None
    for line in text.splitlines(keepends=True):
        match = FENCE_RE.match(line)
        if match:
            marker_character = match.group(1)[0]
            if fence_character is None:
                fence_character = marker_character
            elif marker_character == fence_character:
                fence_character = None
            output.append("\n" if line.endswith("\n") else "")
        elif fence_character is None:
            output.append(line)
        else:
            output.append("\n" if line.endswith("\n") else "")

    visible = "".join(output)
    return HTML_COMMENT_RE.sub(
        lambda match: "\n" * match.group(0).count("\n"), visible
    )


def _targets(text: str) -> Iterable[Tuple[int, str]]:
    for pattern in (INLINE_LINK_RE, REFERENCE_LINK_RE, HTML_LINK_RE):
        for match in pattern.finditer(text):
            yield match.start("target"), match.group("target")


def _local_target(raw_target: str) -> Optional[str]:
    target = raw_target.strip().strip("<>").strip("\"'")
    if not target or target.startswith(("#", "//")):
        return None
    if EXTERNAL_SCHEME_RE.match(target):
        return None

    parsed = urlsplit(target)
    if parsed.scheme or parsed.netloc or not parsed.path:
        return None
    return unquote(parsed.path)


def find_broken_links(root: Path) -> Tuple[int, List[str]]:
    root = root.resolve()
    checked_links = 0
    broken: List[str] = []

    markdown_files = sorted(
        path
        for path in root.rglob("*")
        if path.is_file() and path.suffix.lower() in {".md", ".markdown"}
    )
    for markdown_file in markdown_files:
        text = _without_ignored_regions(
            markdown_file.read_text(encoding="utf-8", errors="replace")
        )
        for offset, raw_target in _targets(text):
            local_target = _local_target(raw_target)
            if local_target is None:
                continue

            checked_links += 1
            resolved = (markdown_file.parent / local_target).resolve()
            try:
                inside_root = os.path.commonpath((str(root), str(resolved))) == str(root)
            except ValueError:
                inside_root = False
            if not inside_root or not resolved.exists():
                line = text.count("\n", 0, offset) + 1
                relative_file = markdown_file.relative_to(root).as_posix()
                broken.append(f"{relative_file}:{line}: {raw_target}")

    return checked_links, broken


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check that Markdown local links resolve inside a source tree."
    )
    parser.add_argument("source_tree", type=Path)
    arguments = parser.parse_args()

    if not arguments.source_tree.is_dir():
        parser.error(f"source tree not found: {arguments.source_tree}")

    checked_links, broken = find_broken_links(arguments.source_tree)
    if broken:
        for item in broken:
            print(f"BROKEN_MARKDOWN_LINK: {item}", file=sys.stderr)
        print(
            f"MARKDOWN_LINK_AUDIT=FAIL checked={checked_links} broken={len(broken)}",
            file=sys.stderr,
        )
        return 1

    print(f"MARKDOWN_LINK_AUDIT=PASS checked={checked_links} broken=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
