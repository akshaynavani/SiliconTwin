import pytest

from silicontwin_harness.expectations import PathError, evaluate, resolve_path


def test_resolve_dotted_index_and_fanout(sample_metrics):
    assert resolve_path(sample_metrics, "regulation.vout_final") == 11.99
    assert resolve_path(sample_metrics, "faults[0].fault") == "OCP"
    assert resolve_path(sample_metrics, "faults[-1].fault") == "UVP"
    assert resolve_path(sample_metrics, "transitions[*].to") == ["SOFT_START", "REGULATING"]


@pytest.mark.parametrize("path", ["regulation.nope", "faults[5]", "faults..fault",
                                  "regulation[0]", "uart.output[*]"])
def test_bad_paths_raise(sample_metrics, path):
    with pytest.raises(PathError):
        resolve_path(sample_metrics, path)


def test_all_operators_pass(sample_metrics):
    expect = {
        "firmware.final_state": "REGULATING",
        "firmware.final_fault": {"ne": "OCP", "in": ["NONE", "UVP"]},
        "regulation.startup_ms": {"min": 20, "max": 30},
        "regulation.vout_final": {"approx": "$regulation.vref_final", "rel": 0.01},
        "firmware.fault_history": {"contains": "OCP", "not_contains": "SENSOR", "len": 2},
        "faults": {"len_min": 1, "len_max": 3},
        "uart.output": {"regex": r"OK state=\w+"},
        "regulation.pgood_final": True,
    }
    results = evaluate(sample_metrics, expect)
    assert results and all(r.passed for r in results), [r.describe() for r in results]


def test_failures_are_reported_not_raised(sample_metrics):
    results = evaluate(sample_metrics, {
        "regulation.overshoot_pct": {"max": 0.05},
        "regulation.vout_final": {"approx": 12.5, "tol": 0.1},
        "missing.key": 1,
        "firmware.final_state": {"min": 3},       # type error on a string
        "faults": {"frobnicate": 1},              # unknown operator
    })
    assert [r.passed for r in results] == [False] * 5
    assert "allowed" in results[1].message
    assert results[2].op == "exists"


def test_approx_defaults_to_tight_relative_tolerance(sample_metrics):
    ok = evaluate(sample_metrics, {"regulation.vref_final": {"approx": 12.0}})
    bad = evaluate(sample_metrics, {"regulation.vout_final": {"approx": 12.0}})
    assert ok[0].passed and not bad[0].passed


def test_booleans_are_not_numbers(sample_metrics):
    res = evaluate(sample_metrics, {"regulation.pgood_final": {"min": 0}})
    assert not res[0].passed
