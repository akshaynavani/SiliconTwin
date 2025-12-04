"""Command line entry point: ``silicontwin-regress``.

    silicontwin-regress list  regression/suite.toml [--tags smoke]
    silicontwin-regress run   regression/suite.toml [--jobs 8] [--tags smoke]
                              [--filter short] [--junit out/junit.xml]
                              [--report out/summary.md] [--json out/results.json]
                              [--baseline regression/baseline.json [--update-baseline]]
                              [--binary build/silicontwin] [--workdir out/runs]
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from . import baseline as baseline_mod
from .executor import CaseOutcome, run_cases
from .report import markdown_summary, write_json, write_junit
from .runner import SimRunner, SimulatorError, find_binary
from .suite import SuiteError, load_suite


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="silicontwin-regress",
                                description="Automated regression runs for SiliconTwin")
    sub = p.add_subparsers(dest="command", required=True)

    ls = sub.add_parser("list", help="list the cases a suite expands to")
    ls.add_argument("suite")
    ls.add_argument("--tags", nargs="*")
    ls.add_argument("--filter")

    run = sub.add_parser("run", help="run a suite")
    run.add_argument("suite")
    run.add_argument("--tags", nargs="*", help="only cases carrying any of these tags")
    run.add_argument("--filter", help="only cases whose name contains this text")
    run.add_argument("--jobs", "-j", type=int, default=os.cpu_count() or 1)
    run.add_argument("--binary", help="path to the silicontwin executable")
    run.add_argument("--workdir", default="out/runs", help="where per-case JSON/CSV land")
    run.add_argument("--junit", help="write a JUnit XML report")
    run.add_argument("--report", help="write a Markdown summary")
    run.add_argument("--json", help="write full results as JSON")
    run.add_argument("--baseline", help="compare against (or update) a metrics baseline")
    run.add_argument("--update-baseline", action="store_true")
    run.add_argument("--timeout", type=float, default=300.0, help="per-case timeout (s)")
    run.add_argument("--quiet", "-q", action="store_true")
    return p


def _print_outcome(o: CaseOutcome) -> None:
    status = "PASS" if o.passed else ("ERROR" if o.error else "FAIL")
    wall = f"{o.result.wall_time_s:6.2f}s" if o.result else "   -   "
    print(f"[{status:5}] {wall}  {o.case.name}", flush=True)
    if o.error:
        print(f"         {o.error}")
    for c in o.failures:
        print(f"         {c.describe()}")


def cmd_list(args: argparse.Namespace) -> int:
    suite = load_suite(args.suite)
    for case in suite.select(args.tags, args.filter):
        tags = ",".join(case.tags)
        print(f"{case.name:50} [{tags}]")
    return 0


def cmd_run(args: argparse.Namespace) -> int:
    suite = load_suite(args.suite)
    cases = suite.select(args.tags, args.filter)
    if not cases:
        print("no cases selected", file=sys.stderr)
        return 2
    binary_hint = args.binary or (
        str((suite.path.parent / suite.binary).resolve()) if suite.binary else None)
    binary = find_binary(binary_hint)
    runner = SimRunner(binary, Path(args.workdir), timeout_s=args.timeout)
    if not args.quiet:
        print(f"suite {suite.name}: {len(cases)} case(s), {args.jobs} job(s), binary {binary}")

    outcomes = run_cases(runner, cases, jobs=args.jobs,
                         on_done=None if args.quiet else _print_outcome)

    if args.junit:
        write_junit(outcomes, Path(args.junit), suite.name)
    if args.json:
        write_json(outcomes, Path(args.json))
    summary = markdown_summary(outcomes, suite.name)
    if args.report:
        Path(args.report).parent.mkdir(parents=True, exist_ok=True)
        Path(args.report).write_text(summary)

    exit_code = 0 if all(o.passed for o in outcomes) else 1

    if args.baseline:
        path = Path(args.baseline)
        existing = baseline_mod.load(path)
        if args.update_baseline:
            baseline_mod.save(path, outcomes, existing)
            print(f"baseline updated: {path}")
        else:
            drifts = baseline_mod.compare(outcomes, existing)
            for d in drifts:
                print(f"[DRIFT] {d.describe()}")
            if drifts:
                exit_code = 1

    passed = sum(1 for o in outcomes if o.passed)
    print(f"{passed}/{len(outcomes)} passed")
    return exit_code


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "list":
            return cmd_list(args)
        return cmd_run(args)
    except (SuiteError, SimulatorError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
