"""Thin wrapper around the ``silicontwin`` CLI."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

BINARY_ENV = "SILICONTWIN_BIN"
_DEFAULT_LOCATIONS = ("build/silicontwin", "build/Release/silicontwin", "build/Debug/silicontwin")


class SimulatorError(RuntimeError):
    """The simulator could not be started or rejected its inputs."""


def find_binary(explicit: str | os.PathLike[str] | None = None, root: Path | None = None) -> Path:
    """Locates the simulator: explicit path, $SILICONTWIN_BIN, the build tree, then $PATH."""
    candidates: list[Path] = []
    if explicit:
        candidates.append(Path(explicit))
    if os.environ.get(BINARY_ENV):
        candidates.append(Path(os.environ[BINARY_ENV]))
    base = root or Path(__file__).resolve().parents[2]
    candidates.extend(base / loc for loc in _DEFAULT_LOCATIONS)
    for c in candidates:
        if c.is_file() and os.access(c, os.X_OK):
            return c.resolve()
    on_path = shutil.which("silicontwin")
    if on_path:
        return Path(on_path)
    raise SimulatorError(
        "silicontwin binary not found; build it with CMake or set "
        f"{BINARY_ENV}=/path/to/silicontwin"
    )


@dataclass
class RunSpec:
    """One simulator invocation."""

    name: str
    config: Path | None = None
    scenario: Path | None = None
    overrides: dict[str, Any] = field(default_factory=dict)
    seed: int | None = None
    duration_ms: int | None = None
    trace: bool = False

    def argv(self, binary: Path, json_out: Path, trace_out: Path | None) -> list[str]:
        args = [str(binary), "--quiet", "--json", str(json_out)]
        if self.config:
            args += ["--config", str(self.config)]
        if self.scenario:
            args += ["--scenario", str(self.scenario)]
        for key, value in sorted(self.overrides.items()):
            args += ["--set", f"{key}={format_value(value)}"]
        if self.seed is not None:
            args += ["--seed", str(self.seed)]
        if self.duration_ms is not None:
            args += ["--duration-ms", str(self.duration_ms)]
        if trace_out is not None:
            args += ["--trace", str(trace_out)]
        return args


def format_value(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


@dataclass
class RunResult:
    spec: RunSpec
    returncode: int
    metrics: dict[str, Any]
    stderr: str
    wall_time_s: float
    trace_path: Path | None = None

    @property
    def ok(self) -> bool:
        return self.returncode == 0 and bool(self.metrics)


class SimRunner:
    """Runs specs and returns parsed metrics."""

    def __init__(self, binary: Path, workdir: Path, timeout_s: float = 300.0) -> None:
        self.binary = Path(binary)
        self.workdir = Path(workdir)
        self.timeout_s = timeout_s
        self.workdir.mkdir(parents=True, exist_ok=True)

    def run(self, spec: RunSpec) -> RunResult:
        safe = "".join(ch if ch.isalnum() or ch in "-_." else "_" for ch in spec.name)
        json_out = self.workdir / f"{safe}.json"
        trace_out = self.workdir / f"{safe}.csv" if spec.trace else None
        argv = spec.argv(self.binary, json_out, trace_out)
        start = time.perf_counter()
        try:
            proc = subprocess.run(
                argv, capture_output=True, text=True, timeout=self.timeout_s, check=False
            )
        except FileNotFoundError as exc:
            raise SimulatorError(f"cannot execute {self.binary}: {exc}") from exc
        except subprocess.TimeoutExpired:
            return RunResult(spec, -1, {}, f"timed out after {self.timeout_s}s",
                             time.perf_counter() - start)
        elapsed = time.perf_counter() - start

        metrics: dict[str, Any] = {}
        if proc.returncode == 0 and json_out.exists():
            try:
                metrics = json.loads(json_out.read_text())
            except json.JSONDecodeError as exc:
                return RunResult(spec, proc.returncode, {}, f"invalid JSON from simulator: {exc}",
                                 elapsed)
        return RunResult(spec, proc.returncode, metrics, proc.stderr, elapsed, trace_out)
