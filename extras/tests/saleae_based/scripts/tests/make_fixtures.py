#!/usr/bin/env python3
"""
make_fixtures.py — regenerate the golden VCD waveforms.

    python3 scripts/tests/make_fixtures.py

See `vcd_fixtures.py` for why these exist (analyzer negative testing) and
`test_analyzer_fixtures.py` for what asserts them.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from vcd_fixtures import FIXTURE_DIR, write_all  # noqa: E402


def main():
    paths = write_all()
    print(f"wrote {len(paths)} fixtures to {FIXTURE_DIR}:")
    for p in paths:
        size = p.stat().st_size
        print(f"  {p.name:28} {size:6} B")
    return 0


if __name__ == "__main__":
    sys.exit(main())