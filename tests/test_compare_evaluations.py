import copy
import json
import subprocess
import sys

import pytest

from scripts.compare_evaluations import compare_reports


def report(events):
    return {"episodes": len(events), "split": "test", "shield": True,
            "records": [{"seed": 20000 + i, "event": event, "steps": 1800,
                         "intervention_reasons": {"turning_fast": i}} for i, event in enumerate(events)]}


def test_paired_outcomes_identify_regressions_even_with_equal_success_rates():
    old = report(["success", "timeout", "timeout", "success"])
    new = report(["timeout", "success", "timeout", "success"])
    original = copy.deepcopy(old)
    comparison = compare_reports(old, new)
    assert comparison["new_timeouts"] == [20000]
    assert comparison["regressed_successes"] == [20000]
    assert comparison["recovered_successes"] == [20001]
    assert comparison["persistent_failures"] == [20002]
    assert comparison["baseline_outcomes"] == comparison["candidate_outcomes"]
    assert comparison["changed_or_failed_records"][0]["candidate_reasons"] == {"turning_fast": 0}
    assert old == original


def test_new_safety_event_is_reported():
    assert compare_reports(report(["success"]), report(["collision"]))["new_safety_events"] == [20000]


@pytest.mark.parametrize("change", ["summary", "duplicate", "seeds", "settings", "incomplete"])
def test_incomparable_reports_are_rejected(change):
    old = report(["success", "timeout"])
    new = copy.deepcopy(old)
    if change == "summary":
        new.pop("records")
    elif change == "duplicate":
        new["records"][1]["seed"] = 20000
    elif change == "seeds":
        new["records"][1]["seed"] = 20002
    elif change == "settings":
        new["shield"] = False
    else:
        new["episodes"] = 100
    with pytest.raises(ValueError):
        compare_reports(old, new)


def test_cli_does_not_overwrite_baseline(tmp_path):
    baseline, candidate = tmp_path/"baseline.json", tmp_path/"candidate.json"
    baseline.write_text(json.dumps(report(["success"])))
    candidate.write_text(json.dumps(report(["timeout"])))
    original = baseline.read_bytes()
    result = subprocess.run([sys.executable, "scripts/compare_evaluations.py", "--baseline", str(baseline),
                             "--candidate", str(candidate), "--output", str(baseline)],
                            text=True, capture_output=True)
    assert result.returncode != 0
    assert baseline.read_bytes() == original
