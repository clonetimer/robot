#!/usr/bin/env python3
"""Standard-library-only delivery audit for the v1.0 project package."""
from __future__ import annotations

import argparse
import hashlib
import re
import sys
from pathlib import Path

EXPECTED_DOCS = [
    "README.md",
    "CHANGELOG.md",
    "docs/REQUIREMENTS.md",
    "docs/DESIGN.md",
    "docs/TEST_PLAN.md",
    "docs/TEST_REPORT.md",
    "docs/PROJECT_CLOSURE_REPORT.md",
    "docs/RESULTS.md",
    "docs/REPRODUCIBILITY.md",
    "docs/VERIFICATION_STATUS.md",
    "deliverables/01_测试大纲.docx",
    "deliverables/02_测试报告.docx",
    "deliverables/03_项目结题报告.docx",
]
EXPECTED_RESULTS = [
    "parameters.csv",
    "singular_values.csv",
    "validation_metrics.csv",
    "control_metrics.csv",
    "trace_pd_only.csv",
    "trace_ct_nominal.csv",
    "trace_ct_identified.csv",
    "acceptance.csv",
    "ctest.txt",
    "ctest_verbose.txt",
    "demo_stdout.txt",
    "environment.txt",
    "reproducibility_check.txt",
    "SHA256SUMS.txt",
]


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path, nargs="?", default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    root = args.root.resolve()
    errors: list[str] = []

    for rel in EXPECTED_DOCS:
        p = root / rel
        if not p.is_file() or p.stat().st_size == 0:
            errors.append(f"missing/empty document: {rel}")

    ref = root / "results/reference_v1.0"
    for rel in EXPECTED_RESULTS:
        p = ref / rel
        if not p.is_file() or p.stat().st_size == 0:
            errors.append(f"missing/empty reference artifact: results/reference_v1.0/{rel}")

    # Validate normal Markdown links/images with local relative destinations.
    link_re = re.compile(r"!?\[[^\]]*\]\(([^)]+)\)")
    checked_links = 0
    for md in sorted(root.rglob("*.md")):
        # Exclude any accidentally retained build tree.
        if any(part.startswith("build") for part in md.relative_to(root).parts):
            continue
        text = md.read_text(encoding="utf-8")
        for target in link_re.findall(text):
            target = target.strip().split("#", 1)[0]
            if not target or "://" in target or target.startswith("#"):
                continue
            dest = (md.parent / target).resolve()
            checked_links += 1
            try:
                dest.relative_to(root)
            except ValueError:
                errors.append(f"link escapes project root: {md.relative_to(root)} -> {target}")
                continue
            if not dest.exists():
                errors.append(f"broken link: {md.relative_to(root)} -> {target}")

    sums = ref / "SHA256SUMS.txt"
    checked_hashes = 0
    if sums.is_file():
        for raw in sums.read_text(encoding="utf-8").splitlines():
            raw = raw.strip()
            if not raw:
                continue
            parts = raw.split(maxsplit=1)
            if len(parts) != 2:
                errors.append(f"malformed SHA256SUMS line: {raw}")
                continue
            expected, name = parts[0], parts[1].lstrip("* ")
            p = ref / name
            if not p.is_file():
                errors.append(f"SHA256SUMS references missing file: {name}")
                continue
            actual = sha256(p)
            checked_hashes += 1
            if actual != expected:
                errors.append(f"hash mismatch: {name}")

    # Acceptance file itself is an explicit release gate.
    acceptance = ref / "acceptance.csv"
    if acceptance.is_file():
        text = acceptance.read_text(encoding="utf-8")
        for criterion in ("A0", "A1", "A2"):
            if not re.search(rf"^{criterion},.*?,PASS$", text, re.MULTILINE):
                errors.append(f"{criterion} is not PASS in reference acceptance.csv")

    if errors:
        print("DELIVERY AUDIT FAIL")
        for error in errors:
            print(f"  - {error}")
        return 1

    print("DELIVERY AUDIT PASS")
    print(f"  documents: {len(EXPECTED_DOCS)}")
    print(f"  reference artifacts: {len(EXPECTED_RESULTS)}")
    print(f"  local markdown links checked: {checked_links}")
    print(f"  SHA-256 entries verified: {checked_hashes}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
