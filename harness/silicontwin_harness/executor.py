"""Runs suite cases in parallel and evaluates their expectations."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field
from typing import Callable

from .expectations import CheckResult, evaluate
from .runner import RunResult, SimRunner
from .suite import Case


@dataclass
class CaseOutcome:
    case: Case
    result: RunResult | None
    checks: list[CheckResult] = field(default_factory=list)
    error: str = ""

    @property
    def passed(self) -> bool:
        return not self.error and self.result is not None and self.result.ok and all(
            c.passed for c in self.checks)

    @property
    def failures(self) -> list[CheckResult]:
        return [c for c in self.checks if not c.passed]


def run_case(runner: SimRunner, case: Case) -> CaseOutcome:
    try:
        result = runner.run(case.to_spec())
    except Exception as exc:  # surfaced in the report rather than aborting the suite
        return CaseOutcome(case, None, error=str(exc))
    if not result.ok:
        msg = result.stderr.strip() or f"simulator exited with {result.returncode}"
        return CaseOutcome(case, result, error=msg)
    return CaseOutcome(case, result, evaluate(result.metrics, case.expect))


def run_cases(runner: SimRunner, cases: list[Case], jobs: int = 1,
              on_done: Callable[[CaseOutcome], None] | None = None) -> list[CaseOutcome]:
    """Each case is a separate simulator process, so threads give real parallelism."""
    outcomes: dict[str, CaseOutcome] = {}
    with ThreadPoolExecutor(max_workers=max(1, jobs)) as pool:
        futures = {pool.submit(run_case, runner, c): c for c in cases}
        for fut in as_completed(futures):
            outcome = fut.result()
            outcomes[outcome.case.name] = outcome
            if on_done:
                on_done(outcome)
    return [outcomes[c.name] for c in cases]  # stable, suite order
