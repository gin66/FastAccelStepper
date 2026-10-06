#!/usr/bin/env python3
"""Known system limitations: a failure that is the target working as designed.

A measurement can come out "failed" for a reason that is a property of the
platform rather than a defect. The one that exists today is the MCPWM/PCNT
generator: it free-runs and is stopped from an interrupt, so a delayed ISR lets
one extra pulse out on the **last** command of a program (documented and now
measured in `extras/doc/platforms/esp32.md`). Regenerating the report must not
report that as a finding, and must not need the report hand-edited -- this file
is the machine-readable form the generator reads.

Each limitation carries the scope it is a property of (**architecture family**
and **driver**) and a predicate over a recorded result. `run_matrix.classify()`
asks :func:`match` before falling through to `DEFECT`, so a known limitation is
a class of its own; `report.py` reads the same registry so its mode tables agree
with the matrix report. The raw result is still recorded `failed` -- the report
reframes it, and the measurement stays the evidence.

Adding a limitation here is the whole change; no report text is touched. A test
(`test_report.py`) checks every entry's doc anchor exists, so its reference
cannot rot.
"""

from pathlib import Path

import harness

# Repo root: <root>/extras/tests/saleae_based/scripts/known_limitations.py
ROOT = Path(__file__).resolve().parents[4]


class KnownLimitation:
    """One documented, platform-scoped non-defect.

    `doc` is repo-relative and carries the `#anchor` of the section that
    explains it. `arch_family` is what `harness.arch_family()` returns (the
    driver may exist on several chips); `driver` is the firmware driver name.
    `matches` is the defect *shape*, and it must be narrow: it distinguishes
    this limitation from a real defect that looks similar.
    """

    __slots__ = ("id", "title", "doc", "arch_family", "driver", "_matches")

    def __init__(self, id, title, doc, arch_family, driver, matches):
        self.id = id
        self.title = title
        self.doc = doc
        self.arch_family = arch_family
        self.driver = driver
        self._matches = matches

    def in_scope(self, family, drivers):
        return self.arch_family == family and self.driver in (drivers or ())

    def matches(self, res):
        return self.in_scope(harness.arch_family(res.get("arch")),
                             res.get("drivers")) and self._matches(res)

    # The doc split into (path, anchor), for a renderer that links to it.
    @property
    def doc_path(self):
        return self.doc.split("#", 1)[0]

    @property
    def doc_anchor(self):
        parts = self.doc.split("#", 1)
        return parts[1] if len(parts) > 1 else ""


def _overrun_on_driver(res, driver):
    """The overrun's exact shape: +1 extra step, on `driver`'s stepper, nothing else.

    The MCPWM/PCNT overrun is one pulse, only the last one, and the period stays
    on spec. Two extra steps, a missing step, a wrong period, or an extra step
    on some *other* stepper is a different failure and must not be excused by
    this entry.
    """
    drivers = list(res.get("drivers") or [])
    per = res.get("per_stepper") or {}
    if not per or len(per) != len(drivers):
        return False
    hit = 0
    for i, letter in enumerate(sorted(per)):
        sub = per[letter] or {}
        steps = sub.get("steps") or {}
        period = sub.get("period") or {}
        broken = steps.get("ok") is False or period.get("ok") is False
        if not broken:
            continue
        if (drivers[i] == driver
                and steps.get("extra_steps") == 1
                and not steps.get("missing_steps")
                and period.get("ok", True)):
            hit += 1
        else:
            return False
    return hit == 1


LIMITATIONS = (
    KnownLimitation(
        id="mcpwm_pcnt_overrun_last_command",
        title="MCPWM/PCNT free-running overrun: one extra pulse on the last "
              "command",
        doc="extras/doc/platforms/esp32.md#mcpwm-pcnt-overrun",
        arch_family="esp",
        driver="mcpwm_pcnt",
        matches=lambda r: _overrun_on_driver(r, "mcpwm_pcnt"),
    ),
)


def match(res):
    """The known limitation a result is an instance of, or None.

    No match means "not a known limitation" -- which is not the same as "a
    defect": the caller decides that, and a result that matches nothing stays
    whatever it was.
    """
    if not isinstance(res, dict):
        return None
    for lim in LIMITATIONS:
        if lim.matches(res):
            return lim
    return None
