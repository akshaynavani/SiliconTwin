"""Baseline snapshots: catch performance drift that still passes expectations.

A baseline stores selected numeric metrics per case. Comparing a new run
against it flags any metric that moved by more than the allowed tolerance in
the "worse" direction (e.g. start-up got slower or overshoot grew).
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .executor import CaseOutcome
from .expectations import PathError, resolve_path

# path -> (relative tolerance, absolute tolerance, direction that counts as worse)
TRACKED: dict[str, tuple[float, float, str]] = {
    "regulation.startup_ms": (0.10, 0.5, "up"),
    "regulation.overshoot_pct": (0.20, 0.25, "up"),
    "regulation.steady_state_error": (0.50, 0.005, "up"),
    "regulation.ripple_pp": (0.50, 0.01, "up"),
    "regulation.vout_min": (0.02, 0.05, "down"),
    "regulation.vout_max": (0.02, 0.05, "up"),
    "regulation.iout_peak": (0.10, 0.2, "up"),
    "watchdog.max_kick_gap_ms": (0.25, 1.0, "up"),
}


@dataclass
class Drift:
    case: str
    path: str
    baseline: float
    current: float
    allowed: float

    def describe(self) -> str:
        return (f"{self.case}: {self.path} {self.baseline:.4g} -> {self.current:.4g} "
                f"(allowed drift {self.allowed:.4g})")


def snapshot(metrics: dict[str, Any]) -> dict[str, float]:
    snap: dict[str, float] = {}
    for path in TRACKED:
        try:
            value = resolve_path(metrics, path)
        except PathError:
            continue
        if isinstance(value, (int, float)) and not isinstance(value, bool) and value >= -1e300:
            snap[path] = float(value)
    return snap


def load(path: Path) -> dict[str, dict[str, float]]:
    if not path.exists():
        return {}
    return json.loads(path.read_text())


def save(path: Path, outcomes: list[CaseOutcome],
         existing: dict[str, dict[str, float]] | None = None) -> dict[str, dict[str, float]]:
    data = dict(existing or {})
    for o in outcomes:
        if o.passed and o.result is not None:
            data[o.case.name] = snapshot(o.result.metrics)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True))
    return data


def compare(outcomes: list[CaseOutcome], baseline: dict[str, dict[str, float]]) -> list[Drift]:
    drifts: list[Drift] = []
    for o in outcomes:
        ref = baseline.get(o.case.name)
        if not ref or o.result is None or not o.result.metrics:
            continue
        current = snapshot(o.result.metrics)
        for path, base_value in ref.items():
            if path not in current or path not in TRACKED:
                continue
            rel, abs_tol, worse = TRACKED[path]
            allowed = max(abs_tol, rel * abs(base_value))
            delta = current[path] - base_value
            if (worse == "up" and delta > allowed) or (worse == "down" and -delta > allowed):
                drifts.append(Drift(o.case.name, path, base_value, current[path], allowed))
    return drifts
