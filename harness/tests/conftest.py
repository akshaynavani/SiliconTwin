import json
import os
import stat
import sys
import textwrap
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

SAMPLE_METRICS = {
    "name": "sample",
    "sim_time_ms": 100,
    "regulation": {"vout_final": 11.99, "vref_final": 12.0, "startup_ms": 25.0,
                   "overshoot_pct": 0.1, "steady_state_error": 0.003, "ripple_pp": 0.02,
                   "vout_min": 0.0, "vout_max": 12.02, "iout_peak": 5.1, "pgood_final": True},
    "firmware": {"final_state": "REGULATING", "final_fault": "NONE",
                 "fault_history": ["OCP", "UVP"], "retries": 0, "params_from_flash": True},
    "faults": [{"t_ms": 50.1, "fault": "OCP"}, {"t_ms": 190, "fault": "UVP"}],
    "transitions": [{"t_ms": 1, "from": "STANDBY", "to": "SOFT_START"},
                    {"t_ms": 21, "from": "SOFT_START", "to": "REGULATING"}],
    "windows": [{"label": "startup", "settle_ms": 21.0}],
    "resets": [],
    "watchdog": {"expirations": 0, "kicks": 99, "max_kick_gap_ms": 1.0},
    "can": {"rx": {"telemetry": 9, "boot": 1}, "acks": []},
    "uart": {"output": "OK state=REGULATING vout=11.990\r\n", "framing_errors": 0},
}


@pytest.fixture
def sample_metrics():
    return json.loads(json.dumps(SAMPLE_METRICS))


@pytest.fixture
def fake_binary(tmp_path):
    """A stand-in for the simulator that echoes its arguments into the metrics.

    Behaviour is steered through --set overrides:
      fake.vout=<v>  sets regulation.vout_final
      fake.fail=true exits with status 2 and an error on stderr
    """
    script = tmp_path / "fake_silicontwin"
    script.write_text(textwrap.dedent(f"""\
        #!{sys.executable}
        import json, sys
        args = sys.argv[1:]
        out = args[args.index("--json") + 1]
        sets = dict(a.split("=", 1) for a in
                    (args[i + 1] for i, x in enumerate(args) if x == "--set"))
        if sets.get("fake.fail") == "true":
            sys.stderr.write("config error: boom\\n")
            sys.exit(2)
        metrics = json.loads({json.dumps(json.dumps(SAMPLE_METRICS))})
        metrics["argv"] = args
        if "fake.vout" in sets:
            metrics["regulation"]["vout_final"] = float(sets["fake.vout"])
        if "--trace" in args:
            open(args[args.index("--trace") + 1], "w").write("t_ms,vout\\n0,0\\n")
        with open(out, "w") as f:
            json.dump(metrics, f)
        """))
    script.chmod(script.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    return script


@pytest.fixture
def suite_dir(tmp_path):
    (tmp_path / "configs").mkdir()
    (tmp_path / "scenarios").mkdir()
    (tmp_path / "configs" / "a.ini").write_text("[sim]\nname = a\n")
    (tmp_path / "configs" / "b.ini").write_text("[sim]\nname = b\n")
    (tmp_path / "scenarios" / "s.scn").write_text("0 mark x\n10 end\n")
    return tmp_path


def write_suite(directory: Path, body: str) -> Path:
    path = directory / "suite.toml"
    path.write_text('[suite]\nname = "t"\nconfig_dir = "configs"\nscenario_dir = "scenarios"\n\n'
                    + textwrap.dedent(body))
    return path


@pytest.fixture(autouse=True)
def _no_env_binary(monkeypatch):
    monkeypatch.delenv("SILICONTWIN_BIN", raising=False)
    yield
    os.environ.pop("SILICONTWIN_BIN", None)
