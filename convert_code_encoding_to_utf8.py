#!/usr/bin/env python3
"""
Recursively convert source/code text files from GBK/GB18030 to UTF-8.

Default mode is dry-run. Add --apply to rewrite files.

Examples:
  python convert_code_encoding_to_utf8.py
  python convert_code_encoding_to_utf8.py --apply
  python convert_code_encoding_to_utf8.py --apply --root code user
"""

from __future__ import annotations

import argparse
import datetime as _dt
from pathlib import Path
from typing import Iterable


DEFAULT_EXTENSIONS = {
    ".c",
    ".h",
    ".cc",
    ".cpp",
    ".cxx",
    ".hpp",
    ".hxx",
    ".s",
    ".asm",
    ".lsl",
    ".ld",
    ".py",
    ".bat",
    ".cmd",
    ".ps1",
    ".mk",
    ".mak",
    ".project",
    ".cproject",
    ".launch",
    ".properties",
    ".ini",
    ".cfg",
    ".json",
    ".xml",
    ".yml",
    ".yaml",
    ".md",
    ".txt",
}

DEFAULT_EXCLUDED_DIRS = {
    ".git",
    ".svn",
    ".hg",
    ".metadata",
    ".encoding_backup",
    "__pycache__",
    "Debug",
    "Release",
    "build",
    "dist",
}


def parse_extensions(values: Iterable[str]) -> set[str]:
    exts: set[str] = set()
    for value in values:
        for item in value.split(","):
            item = item.strip()
            if not item:
                continue
            if not item.startswith("."):
                item = "." + item
            exts.add(item)
    return exts


def should_skip_dir(path: Path, excluded_dirs: set[str]) -> bool:
    return any(part in excluded_dirs for part in path.parts)


def iter_targets(roots: list[Path], extensions: set[str], excluded_dirs: set[str]):
    for root in roots:
        if root.is_file():
            if root.suffix.lower() in extensions or root.name.lower() in extensions:
                yield root
            continue

        for path in root.rglob("*"):
            if path.is_dir():
                continue
            if should_skip_dir(path.relative_to(root), excluded_dirs):
                continue
            if path.suffix.lower() in extensions or path.name.lower() in extensions:
                yield path


def decode_source(data: bytes, source_encoding: str) -> tuple[str, str]:
    if b"\x00" in data:
        raise UnicodeError("looks like a binary file")

    try:
        return data.decode("utf-8-sig"), "utf-8"
    except UnicodeDecodeError:
        pass

    return data.decode(source_encoding), source_encoding


def backup_file(path: Path, root_anchor: Path, backup_root: Path, data: bytes) -> None:
    rel = path.resolve().relative_to(root_anchor.resolve())
    backup_path = backup_root / rel
    backup_path.parent.mkdir(parents=True, exist_ok=True)
    backup_path.write_bytes(data)


def convert_file(
    path: Path,
    *,
    root_anchor: Path,
    source_encoding: str,
    apply: bool,
    backup_root: Path | None,
) -> str:
    original = path.read_bytes()

    try:
        text, detected = decode_source(original, source_encoding)
    except UnicodeError as exc:
        return f"FAIL    {path}  ({exc})"

    if detected == "utf-8":
        return f"OK      {path}  (already UTF-8)"

    converted = text.encode("utf-8")
    if converted == original:
        return f"OK      {path}  (no byte change)"

    if apply:
        if backup_root is not None:
            backup_file(path, root_anchor, backup_root, original)
        path.write_bytes(converted)
        return f"CONVERT {path}  ({source_encoding} -> UTF-8)"

    return f"WOULD    {path}  ({source_encoding} -> UTF-8)"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert code/text files from GBK/GB18030 to UTF-8."
    )
    parser.add_argument(
        "--root",
        nargs="+",
        default=["."],
        help="Root directories/files to scan. Default: current directory.",
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Actually rewrite files. Without this flag the script only prints a dry-run.",
    )
    parser.add_argument(
        "--from-encoding",
        default="gb18030",
        help="Source encoding for non-UTF-8 files. gb18030 is a GBK superset. Default: gb18030.",
    )
    parser.add_argument(
        "--ext",
        action="append",
        default=[],
        help="Additional extension(s), comma-separated. Example: --ext .inc,.src",
    )
    parser.add_argument(
        "--only-ext",
        action="append",
        default=[],
        help="Use only these extension(s), comma-separated, instead of the built-in list.",
    )
    parser.add_argument(
        "--exclude-dir",
        action="append",
        default=[],
        help="Additional directory name to skip. Can be repeated.",
    )
    parser.add_argument(
        "--no-backup",
        action="store_true",
        help="Do not create backups when --apply is used.",
    )
    args = parser.parse_args()

    cwd = Path.cwd()
    roots = [(cwd / item).resolve() for item in args.root]

    if args.only_ext:
        extensions = parse_extensions(args.only_ext)
    else:
        extensions = set(DEFAULT_EXTENSIONS)
        extensions.update(parse_extensions(args.ext))

    excluded_dirs = set(DEFAULT_EXCLUDED_DIRS)
    excluded_dirs.update(args.exclude_dir)

    backup_root: Path | None = None
    if args.apply and not args.no_backup:
        stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_root = cwd / ".encoding_backup" / stamp

    print("Mode:", "APPLY" if args.apply else "DRY-RUN")
    print("Source encoding for non-UTF-8 files:", args.from_encoding)
    print("Roots:", ", ".join(str(path) for path in roots))
    print("Extensions:", ", ".join(sorted(extensions)))
    if backup_root is not None:
        print("Backup root:", backup_root)
    print()

    root_anchor = cwd.resolve()
    counts = {"OK": 0, "WOULD": 0, "CONVERT": 0, "FAIL": 0}

    for path in iter_targets(roots, extensions, excluded_dirs):
        result = convert_file(
            path,
            root_anchor=root_anchor,
            source_encoding=args.from_encoding,
            apply=args.apply,
            backup_root=backup_root,
        )
        print(result)
        status = result.split(maxsplit=1)[0]
        counts[status] = counts.get(status, 0) + 1

    print()
    print(
        "Summary:",
        f"OK={counts.get('OK', 0)}",
        f"WOULD={counts.get('WOULD', 0)}",
        f"CONVERT={counts.get('CONVERT', 0)}",
        f"FAIL={counts.get('FAIL', 0)}",
    )

    if not args.apply:
        print("Dry-run only. Re-run with --apply to rewrite files.")

    return 1 if counts.get("FAIL", 0) else 0


if __name__ == "__main__":
    raise SystemExit(main())
