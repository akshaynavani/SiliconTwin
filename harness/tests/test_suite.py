import pytest

from silicontwin_harness.suite import SuiteError, load_suite

from conftest import write_suite


def test_basic_case_paths_are_resolved(suite_dir):
    path = write_suite(suite_dir, """
        [[case]]
        name = "one"
        config = "a.ini"
        scenario = "s.scn"
        tags = ["smoke"]
        set = { "firmware.kp_v" = 2.5 }
        expect = { "firmware.final_state" = "REGULATING" }
    """)
    suite = load_suite(path)
    assert suite.name == "t"
    (case,) = suite.cases
    assert case.config == (suite_dir / "configs" / "a.ini").resolve()
    assert case.scenario == (suite_dir / "scenarios" / "s.scn").resolve()
    spec = case.to_spec()
    assert spec.overrides == {"firmware.kp_v": 2.5}


def test_matrix_expands_cartesian_product(suite_dir):
    path = write_suite(suite_dir, """
        [[case]]
        name = "m"
        scenario = "s.scn"
        matrix = { config = ["a.ini", "b.ini"], "firmware.kp_v" = [1.0, 2.0, 3.0], seed = [7] }
    """)
    cases = load_suite(path).cases
    assert len(cases) == 6
    names = {c.name for c in cases}
    assert "m[config=a,kp_v=1.0,seed=7]" in names
    c = next(c for c in cases if c.name == "m[config=b,kp_v=3.0,seed=7]")
    assert c.config.name == "b.ini"
    assert c.overrides == {"firmware.kp_v": 3.0}
    assert c.seed == 7


def test_selection_by_tag_and_filter(suite_dir):
    path = write_suite(suite_dir, """
        [[case]]
        name = "fast_one"
        tags = ["smoke"]
        [[case]]
        name = "slow_one"
        tags = ["slow"]
    """)
    suite = load_suite(path)
    assert [c.name for c in suite.select(["smoke"])] == ["fast_one"]
    assert [c.name for c in suite.select(pattern="slow")] == ["slow_one"]
    assert len(suite.select()) == 2


@pytest.mark.parametrize("body, message", [
    ('[[case]]\nconfig = "a.ini"\n', "needs a name"),
    ('[[case]]\nname = "x"\nbogus = 1\n', "unknown fields"),
    ('[[case]]\nname = "x"\nset = { kp = 1 }\n', "section.key"),
    ('[[case]]\nname = "x"\nmatrix = { kp = [1] }\n', "section.key"),
    ('[[case]]\nname = "x"\nmatrix = { seed = [] }\n', "non-empty"),
    ('[[case]]\nname = "x"\n[[case]]\nname = "x"\n', "duplicate"),
    ('[[case]\n', None),
])
def test_invalid_suites_are_rejected(suite_dir, body, message):
    path = write_suite(suite_dir, body)
    with pytest.raises(SuiteError, match=message):
        load_suite(path)


def test_shipped_regression_suite_loads():
    from pathlib import Path

    suite_path = Path(__file__).resolve().parents[2] / "regression" / "suite.toml"
    suite = load_suite(suite_path)
    assert len(suite.cases) >= 20
    for case in suite.cases:
        if case.config:
            assert case.config.exists(), case.config
        if case.scenario:
            assert case.scenario.exists(), case.scenario
        assert case.expect, f"{case.name} has no expectations"
