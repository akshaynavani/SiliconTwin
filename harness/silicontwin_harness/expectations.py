"""Expectation language evaluated against simulator metrics.

Keys are dotted paths into the metrics JSON. List elements are addressed as
``faults[0]``, ``faults[-1]`` or ``faults[*]`` (fan-out over every element):

    "firmware.final_state" = "REGULATING"                  # equality
    "regulation.startup_ms" = { max = 40 }
    "regulation.vout_final" = { approx = 12.0, tol = 0.1 }
    "regulation.vout_final" = { approx = "$regulation.vref_final", rel = 0.01 }
    "faults[*].fault" = { contains = "OCP" }
    "resets" = { len = 1 }
    "uart.output" = { regex = "OK state=\\w+" }

Supported operators: eq, ne, min, max, approx (+ tol / rel), contains,
not_contains, in, len, len_min, len_max, regex. A string value starting with
``$`` is resolved as another path in the same metrics document.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Any

_MISSING = object()
_TOKEN = re.compile(r"([^.\[\]]+)|\[(-?\d+|\*)\]")


class PathError(KeyError):
    pass


@dataclass
class CheckResult:
    path: str
    op: str
    expected: Any
    actual: Any
    passed: bool
    message: str = ""

    def describe(self) -> str:
        status = "PASS" if self.passed else "FAIL"
        text = f"{status} {self.path} {self.op} {self.expected!r} (actual {self.actual!r})"
        return f"{text}: {self.message}" if self.message else text


def _tokens(path: str) -> list[str | int]:
    out: list[str | int] = []
    pos = 0
    for m in _TOKEN.finditer(path):
        if m.start() != pos and path[pos:m.start()] != ".":
            raise PathError(f"malformed path {path!r}")
        pos = m.end()
        if m.group(1) is not None:
            out.append(m.group(1))
        else:
            idx = m.group(2)
            out.append("*" if idx == "*" else int(idx))
    if pos != len(path):
        raise PathError(f"malformed path {path!r}")
    return out


def resolve_path(doc: Any, path: str) -> Any:
    """Resolves a path; a ``[*]`` segment returns a list of all matches."""

    def walk(node: Any, toks: list[str | int]) -> Any:
        if not toks:
            return node
        head, rest = toks[0], toks[1:]
        if head == "*":
            if not isinstance(node, list):
                raise PathError(f"{path!r}: [*] applied to non-list")
            return [walk(item, rest) for item in node]
        if isinstance(head, int):
            if not isinstance(node, list) or not -len(node) <= head < len(node):
                raise PathError(f"{path!r}: index {head} out of range")
            return walk(node[head], rest)
        if not isinstance(node, dict) or head not in node:
            raise PathError(f"{path!r}: no key {head!r}")
        return walk(node[head], rest)

    return walk(doc, _tokens(path))


def _deref(doc: Any, value: Any) -> Any:
    if isinstance(value, str) and value.startswith("$"):
        return resolve_path(doc, value[1:])
    return value


def _num(x: Any) -> float:
    if isinstance(x, bool) or not isinstance(x, (int, float)):
        raise TypeError(f"expected a number, got {x!r}")
    return float(x)


def _check(op: str, actual: Any, expected: Any, spec: dict[str, Any]) -> tuple[bool, str]:
    if op == "eq":
        return actual == expected, ""
    if op == "ne":
        return actual != expected, ""
    if op == "min":
        return _num(actual) >= _num(expected), ""
    if op == "max":
        return _num(actual) <= _num(expected), ""
    if op == "approx":
        a, e = _num(actual), _num(expected)
        tol = float(spec.get("tol", 0.0))
        rel = float(spec.get("rel", 0.0))
        if tol == 0.0 and rel == 0.0:
            rel = 1e-6
        allowed = max(tol, rel * abs(e))
        diff = abs(a - e)
        return math.isfinite(a) and diff <= allowed, f"|diff|={diff:.6g} allowed={allowed:.6g}"
    if op == "contains":
        return expected in actual, ""
    if op == "not_contains":
        return expected not in actual, ""
    if op == "in":
        return actual in expected, ""
    if op == "len":
        return len(actual) == int(expected), f"len={len(actual)}"
    if op == "len_min":
        return len(actual) >= int(expected), f"len={len(actual)}"
    if op == "len_max":
        return len(actual) <= int(expected), f"len={len(actual)}"
    if op == "regex":
        return re.search(str(expected), str(actual)) is not None, ""
    raise ValueError(f"unknown operator {op!r}")


_MODIFIERS = {"tol", "rel"}


def evaluate(metrics: dict[str, Any], expect: dict[str, Any]) -> list[CheckResult]:
    """Evaluates every expectation; never raises for bad metrics, only reports."""
    results: list[CheckResult] = []
    for path, spec in expect.items():
        try:
            actual = resolve_path(metrics, path)
        except PathError as exc:
            results.append(CheckResult(path, "exists", True, None, False, str(exc)))
            continue
        ops = spec if isinstance(spec, dict) else {"eq": spec}
        for op, raw_expected in ops.items():
            if op in _MODIFIERS:
                continue
            try:
                expected = _deref(metrics, raw_expected)
                passed, msg = _check(op, actual, expected, ops)
            except (TypeError, ValueError, PathError) as exc:
                results.append(CheckResult(path, op, raw_expected, actual, False, str(exc)))
                continue
            results.append(CheckResult(path, op, expected, actual, passed, msg))
    return results
