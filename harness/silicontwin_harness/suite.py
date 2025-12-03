"""Regression suite definition (TOML) and configuration-matrix expansion.

    [suite]
    name = "regression"
    config_dir = "../configs"        # relative to the suite file
    scenario_dir = "../scenarios"

    [[case]]
    name = "startup"
    config = "nominal_12v.ini"
    scenario = "startup.scn"
    tags = ["smoke"]
    set = { "firmware.kp_v" = 3.0 }  # --set overrides
    matrix = { config = ["nominal_12v.ini", "low_voltage_5v.ini"], seed = [1, 2] }
    expect = { "firmware.final_state" = "REGULATING" }

Matrix keys ``config``, ``scenario``, ``seed`` and ``duration_ms`` replace the
case fields; any other key is treated as a ``section.key`` override. Each
combination becomes its own case named ``<name>[k=v,...]``.
"""

from __future__ import annotations

import itertools
import tomllib
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

from .runner import RunSpec

_SPECIAL = {"config", "scenario", "seed", "duration_ms"}


class SuiteError(ValueError):
    pass


@dataclass
class Case:
    name: str
    config: Path | None = None
    scenario: Path | None = None
    overrides: dict[str, Any] = field(default_factory=dict)
    seed: int | None = None
    duration_ms: int | None = None
    tags: list[str] = field(default_factory=list)
    expect: dict[str, Any] = field(default_factory=dict)
    trace: bool = False

    def to_spec(self) -> RunSpec:
        return RunSpec(
            name=self.name,
            config=self.config,
            scenario=self.scenario,
            overrides=dict(self.overrides),
            seed=self.seed,
            duration_ms=self.duration_ms,
            trace=self.trace,
        )


@dataclass
class Suite:
    name: str
    path: Path
    cases: list[Case]
    binary: str | None = None

    def select(self, tags: list[str] | None = None, pattern: str | None = None) -> list[Case]:
        chosen = self.cases
        if tags:
            wanted = set(tags)
            chosen = [c for c in chosen if wanted & set(c.tags)]
        if pattern:
            chosen = [c for c in chosen if pattern in c.name]
        return chosen


def _resolve(base: Path, value: str | None) -> Path | None:
    if value is None:
        return None
    p = Path(value)
    return p if p.is_absolute() else (base / p).resolve()


def _short(value: Any) -> str:
    if isinstance(value, str) and "." in value and "/" not in value:
        return value.rsplit(".", 1)[0]  # drop file extensions in names
    return str(value)


def expand_matrix(case: Case, matrix: dict[str, list[Any]], config_dir: Path,
                  scenario_dir: Path) -> list[Case]:
    if not matrix:
        return [case]
    keys = sorted(matrix)
    for k in keys:
        if not isinstance(matrix[k], list) or not matrix[k]:
            raise SuiteError(f"case {case.name!r}: matrix entry {k!r} must be a non-empty list")
    out: list[Case] = []
    for combo in itertools.product(*(matrix[k] for k in keys)):
        variant = replace(case, overrides=dict(case.overrides), tags=list(case.tags))
        label = []
        for key, value in zip(keys, combo):
            label.append(f"{key.split('.')[-1]}={_short(value)}")
            if key == "config":
                variant.config = _resolve(config_dir, value)
            elif key == "scenario":
                variant.scenario = _resolve(scenario_dir, value)
            elif key == "seed":
                variant.seed = int(value)
            elif key == "duration_ms":
                variant.duration_ms = int(value)
            else:
                variant.overrides[key] = value
        variant.name = f"{case.name}[{','.join(label)}]"
        out.append(variant)
    return out


def load_suite(path: str | Path) -> Suite:
    path = Path(path).resolve()
    try:
        data = tomllib.loads(path.read_text())
    except tomllib.TOMLDecodeError as exc:
        raise SuiteError(f"{path}: {exc}") from exc
    meta = data.get("suite", {})
    base = path.parent
    config_dir = _resolve(base, meta.get("config_dir", ".")) or base
    scenario_dir = _resolve(base, meta.get("scenario_dir", ".")) or base

    cases: list[Case] = []
    seen: set[str] = set()
    for raw in data.get("case", []):
        if "name" not in raw:
            raise SuiteError(f"{path}: every [[case]] needs a name")
        unknown = set(raw) - {"name", "config", "scenario", "set", "seed", "duration_ms",
                              "tags", "expect", "matrix", "trace"}
        if unknown:
            raise SuiteError(f"case {raw['name']!r}: unknown fields {sorted(unknown)}")
        for key in raw.get("set", {}):
            if "." not in key:
                raise SuiteError(f"case {raw['name']!r}: override {key!r} must be section.key")
        for key in raw.get("matrix", {}):
            if key not in _SPECIAL and "." not in key:
                raise SuiteError(f"case {raw['name']!r}: matrix key {key!r} must be section.key")
        case = Case(
            name=raw["name"],
            config=_resolve(config_dir, raw.get("config")),
            scenario=_resolve(scenario_dir, raw.get("scenario")),
            overrides=dict(raw.get("set", {})),
            seed=raw.get("seed"),
            duration_ms=raw.get("duration_ms"),
            tags=list(raw.get("tags", [])),
            expect=dict(raw.get("expect", {})),
            trace=bool(raw.get("trace", False)),
        )
        for variant in expand_matrix(case, raw.get("matrix", {}), config_dir, scenario_dir):
            if variant.name in seen:
                raise SuiteError(f"duplicate case name {variant.name!r}")
            seen.add(variant.name)
            cases.append(variant)

    return Suite(name=meta.get("name", path.stem), path=path, cases=cases,
                 binary=meta.get("binary"))
