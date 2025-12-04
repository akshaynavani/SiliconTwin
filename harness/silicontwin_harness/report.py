"""JUnit XML, Markdown and JSON reports."""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from pathlib import Path

from .executor import CaseOutcome

_SUMMARY_METRICS = (
    ("startup ms", "regulation.startup_ms"),
    ("overshoot %", "regulation.overshoot_pct"),
    ("vout final", "regulation.vout_final"),
    ("state", "firmware.final_state"),
)


def _metric(outcome: CaseOutcome, path: str) -> str:
    from .expectations import PathError, resolve_path

    if outcome.result is None or not outcome.result.metrics:
        return "-"
    try:
        value = resolve_path(outcome.result.metrics, path)
    except PathError:
        return "-"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def write_junit(outcomes: list[CaseOutcome], path: Path, suite_name: str) -> None:
    failures = sum(1 for o in outcomes if not o.passed and not o.error)
    errors = sum(1 for o in outcomes if o.error)
    total_time = sum(o.result.wall_time_s for o in outcomes if o.result)
    root = ET.Element("testsuite", name=suite_name, tests=str(len(outcomes)),
                      failures=str(failures), errors=str(errors), time=f"{total_time:.3f}")
    for o in outcomes:
        tc = ET.SubElement(root, "testcase", classname=suite_name, name=o.case.name,
                           time=f"{o.result.wall_time_s:.3f}" if o.result else "0")
        if o.error:
            ET.SubElement(tc, "error", message=o.error.splitlines()[0] if o.error else "")
        elif o.failures:
            text = "\n".join(c.describe() for c in o.failures)
            el = ET.SubElement(tc, "failure", message=f"{len(o.failures)} expectation(s) failed")
            el.text = text
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def markdown_summary(outcomes: list[CaseOutcome], suite_name: str) -> str:
    passed = sum(1 for o in outcomes if o.passed)
    lines = [f"# {suite_name}: {passed}/{len(outcomes)} passed", ""]
    header = ["case", "result"] + [label for label, _ in _SUMMARY_METRICS]
    lines.append("| " + " | ".join(header) + " |")
    lines.append("|" + "---|" * len(header))
    for o in outcomes:
        result = "PASS" if o.passed else ("ERROR" if o.error else "FAIL")
        row = [o.case.name, result] + [_metric(o, p) for _, p in _SUMMARY_METRICS]
        lines.append("| " + " | ".join(row) + " |")
    failed = [o for o in outcomes if not o.passed]
    if failed:
        lines += ["", "## Failures", ""]
        for o in failed:
            lines.append(f"### {o.case.name}")
            if o.error:
                lines.append(f"- error: `{o.error}`")
            for c in o.failures:
                lines.append(f"- {c.describe()}")
            lines.append("")
    return "\n".join(lines) + "\n"


def write_json(outcomes: list[CaseOutcome], path: Path) -> None:
    doc = []
    for o in outcomes:
        doc.append({
            "case": o.case.name,
            "passed": o.passed,
            "error": o.error,
            "wall_time_s": o.result.wall_time_s if o.result else None,
            "checks": [
                {"path": c.path, "op": c.op, "expected": c.expected, "actual": c.actual,
                 "passed": c.passed, "message": c.message}
                for c in o.checks
            ],
            "metrics": o.result.metrics if o.result else {},
        })
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, indent=2, default=str))
