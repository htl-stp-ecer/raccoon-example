#!/usr/bin/env python3
"""Move the raccoon pins in a template branch's pyproject.toml to exact versions.

Used by .github/workflows/bump-raccoon.yml. Both pins are set exactly to the
given versions, also when that is older than what is pinned now: the bundle is
the source of truth and its versions are the tested ones.

    bump_pins.py --pyproject pyproject.toml --cli 173 --lib 155 [--bundle 2026.11.07]

With --bundle, the project `version` is set to the bundle name too.
"""
from __future__ import annotations

import argparse
import os
import re
import sys
from pathlib import Path

VERSION = re.compile(r"[0-9]+(\.[0-9]+)*")
BUNDLE = re.compile(r"\d{4}\.\d{2}\.\d{2}(-\d+)?")
PACKAGES = {"cli": "raccoon-cli", "lib": "raccoon-library"}


def normalize_version(value: str) -> str:
    """A release tag (`v173`) or a PyPI version (`173`) -> `173`."""
    value = value.strip()
    if value[:1] in ("v", "V"):
        value = value[1:]
    if not VERSION.fullmatch(value):
        raise ValueError(f"{value!r} is not a plain version number")
    return value


def set_pin(text: str, package: str, version: str) -> tuple[str, str]:
    """Return (new text, old version). Matches the pin and the commented-out `[sim]` line."""
    pin = re.compile(r"(" + re.escape(package) + r"(?:\[[a-z]+\])?==)([0-9][0-9.]*)")
    found = pin.findall(text)
    if not found:
        raise ValueError(f"no pin for {package} in pyproject.toml")
    return pin.sub(lambda m: m.group(1) + version, text), found[0][1]


def set_version(text: str, version: str) -> str:
    """Set `version = "..."` in the [project] table."""
    if not BUNDLE.fullmatch(version):
        raise ValueError(f"{version!r} is not a bundle name")
    match = re.search(r"(?ms)^\[project\]\s*$.*?^(version\s*=\s*)\"[^\"\n]*\"", text)
    if not match:
        raise ValueError("no version in the [project] table of pyproject.toml")
    return text[:match.start(1)] + match.group(1) + f'"{version}"' + text[match.end():]


def bump(text: str, cli: str, lib: str, bundle: str = "") -> tuple[str, dict[str, str]]:
    old = {}
    for key, version in (("cli", cli), ("lib", lib)):
        text, old[key] = set_pin(text, PACKAGES[key], normalize_version(version))
    if bundle:
        text = set_version(text, bundle)
    return text, old


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pyproject", required=True, type=Path)
    parser.add_argument("--cli", required=True)
    parser.add_argument("--lib", required=True)
    parser.add_argument("--bundle", default="", help="bundle name to use as the project version (empty = leave it)")
    args = parser.parse_args(argv)
    try:
        original = args.pyproject.read_text()
        updated, old = bump(original, args.cli, args.lib, args.bundle)
    except (OSError, ValueError) as error:
        print(f"::error::{error}")
        return 1
    changed = updated != original
    if changed:
        args.pyproject.write_text(updated)
    print(f"raccoon-cli {old['cli']} -> {normalize_version(args.cli)}, "
          f"raccoon-library {old['lib']} -> {normalize_version(args.lib)} ({'changed' if changed else 'already there'})")
    if os.environ.get("GITHUB_OUTPUT"):
        with open(os.environ["GITHUB_OUTPUT"], "a") as out:
            out.write(f"changed={'true' if changed else 'false'}\n")
            out.write(f"old_cli={old['cli']}\nold_lib={old['lib']}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
