import json
import xml.etree.ElementTree as ET

import pytest

from silicontwin_harness import baseline
from silicontwin_harness.cli import main
from silicontwin_harness.executor import run_cases
from silicontwin_harness.runner import RunSpec, SimRunner, SimulatorError, find_binary
from silicontwin_harness.suite import Case

from conftest import write_suite


def test_spec_builds_expected_argv(tmp_path):
    spec = RunSpec("n", config=tmp_path / "c.ini", scenario=tmp_path / "s.scn",
                   overrides={"b.x": True, "a.y": 1.5}, seed=3, duration_ms=50)
    argv = spec.argv(tmp_path / "bin", tmp_path / "o.json", None)
    assert argv[:4] == [str(tmp_path / "bin"), "--quiet", "--json", str(tmp_path / "o.json")]
    assert argv[argv.index("--seed") + 1] == "3"
    sets = [argv[i + 1] for i, a in enumerate(argv) if a == "--set"]
    assert sets == ["a.y=1.5", "b.x=true"]  # sorted, booleans lower-cased


def test_runner_parses_metrics_and_trace(fake_binary, tmp_path):
    runner = SimRunner(fake_binary, tmp_path / "work")
    result = runner.run(RunSpec("case/with spaces", overrides={"fake.vout": 5}, trace=True))
    assert result.ok
    assert result.metrics["regulation"]["vout_final"] == 5.0
    assert result.trace_path is not None and result.trace_path.exists()
    assert "/" not in result.trace_path.name


def test_runner_reports_simulator_errors(fake_binary, tmp_path):
    result = SimRunner(fake_binary, tmp_path).run(RunSpec("bad", overrides={"fake.fail": True}))
    assert not result.ok
    assert "boom" in result.stderr


def test_find_binary_prefers_env(fake_binary, monkeypatch):
    monkeypatch.setenv("SILICONTWIN_BIN", str(fake_binary))
    assert find_binary(root=fake_binary.parent / "nowhere") == fake_binary.resolve()


def test_find_binary_missing(tmp_path, monkeypatch):
    monkeypatch.setenv("PATH", str(tmp_path))
    with pytest.raises(SimulatorError):
        find_binary(root=tmp_path)


def test_parallel_execution_keeps_suite_order(fake_binary, tmp_path):
    cases = [Case(name=f"c{i}", overrides={"fake.vout": i},
                  expect={"regulation.vout_final": {"approx": float(i), "tol": 1e-9}})
             for i in range(8)]
    outcomes = run_cases(SimRunner(fake_binary, tmp_path), cases, jobs=4)
    assert [o.case.name for o in outcomes] == [c.name for c in cases]
    assert all(o.passed for o in outcomes)


def test_cli_end_to_end_with_reports(fake_binary, suite_dir, tmp_path, capsys):
    suite = write_suite(suite_dir, """
        [[case]]
        name = "good"
        config = "a.ini"
        tags = ["smoke"]
        expect = { "firmware.final_state" = "REGULATING" }

        [[case]]
        name = "bad"
        config = "b.ini"
        set = { "fake.vout" = 3.0 }
        expect = { "regulation.vout_final" = { approx = 12.0, tol = 0.1 } }

        [[case]]
        name = "broken"
        set = { "fake.fail" = true }
        expect = { "faults" = { len = 0 } }
    """)
    junit = tmp_path / "out" / "junit.xml"
    report = tmp_path / "out" / "summary.md"
    results = tmp_path / "out" / "results.json"
    code = main(["run", str(suite), "--binary", str(fake_binary), "--workdir",
                 str(tmp_path / "runs"), "--junit", str(junit), "--report", str(report),
                 "--json", str(results), "-j", "2"])
    assert code == 1
    out = capsys.readouterr().out
    assert "1/3 passed" in out

    root = ET.parse(junit).getroot()
    assert root.get("tests") == "3"
    assert root.get("failures") == "1"
    assert root.get("errors") == "1"
    assert "FAIL" in report.read_text() and "ERROR" in report.read_text()
    doc = json.loads(results.read_text())
    assert {d["case"] for d in doc} == {"good", "bad", "broken"}

    assert main(["run", str(suite), "--binary", str(fake_binary), "--tags", "smoke",
                 "--workdir", str(tmp_path / "runs"), "-q"]) == 0


def test_cli_list_and_errors(suite_dir, capsys):
    suite = write_suite(suite_dir, '[[case]]\nname = "only"\ntags = ["x"]\n')
    assert main(["list", str(suite)]) == 0
    assert "only" in capsys.readouterr().out
    assert main(["list", str(suite_dir / "missing.toml")]) == 2
    bad = write_suite(suite_dir, "[[case]]\nnope = 1\n")
    assert main(["list", str(bad)]) == 2


def test_baseline_roundtrip_and_drift(fake_binary, suite_dir, tmp_path, capsys):
    suite = write_suite(suite_dir, '[[case]]\nname = "b"\nexpect = { "faults" = { len = 2 } }\n')
    base = tmp_path / "baseline.json"
    args = ["run", str(suite), "--binary", str(fake_binary), "--workdir", str(tmp_path / "r"),
            "--baseline", str(base), "-q"]
    assert main(args + ["--update-baseline"]) == 0
    snap = json.loads(base.read_text())
    assert snap["b"]["regulation.startup_ms"] == 25.0

    # Same metrics: no drift.
    assert main(args) == 0

    # Make the stored baseline look much faster than the current run.
    snap["b"]["regulation.startup_ms"] = 10.0
    base.write_text(json.dumps(snap))
    assert main(args) == 1
    assert "[DRIFT]" in capsys.readouterr().out


def test_baseline_ignores_improvements():
    from silicontwin_harness.executor import CaseOutcome
    from silicontwin_harness.runner import RunResult

    metrics = {"regulation": {"startup_ms": 20.0, "vout_min": 11.5}}
    outcome = CaseOutcome(Case("x"), RunResult(RunSpec("x"), 0, metrics, "", 0.1))
    ref = {"x": {"regulation.startup_ms": 25.0, "regulation.vout_min": 11.0}}
    assert baseline.compare([outcome], ref) == []
    ref = {"x": {"regulation.vout_min": 11.9}}
    (drift,) = baseline.compare([outcome], ref)
    assert drift.path == "regulation.vout_min"
