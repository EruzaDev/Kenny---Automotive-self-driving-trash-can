"""Read-only paired evaluation comparison; keep baseline and candidate reports separate."""
import argparse
from collections import Counter
import json
from pathlib import Path


def episode_index(report):
    records = report.get("records")
    if not isinstance(records, list) or not records:
        raise ValueError("Full evaluation reports with nonempty records are required, not summaries")
    indexed = {}
    for record in records:
        seed, event = record.get("seed"), record.get("event")
        if type(seed) is not int or seed in indexed:
            raise ValueError(f"Missing, invalid, or duplicate episode seed: {seed}")
        if event not in ("success", "timeout", "collision", "cliff"):
            raise ValueError(f"Unknown outcome for seed {seed}: {event}")
        indexed[seed] = record
    if report.get("episodes", len(records)) != len(records):
        raise ValueError("Episode count does not match records; evaluation may be incomplete")
    return indexed


def compare_reports(baseline, candidate):
    for key in ("split", "stage", "map_mode", "shield", "grid_resolution", "route_clearance_weight"):
        if baseline.get(key) != candidate.get(key):
            raise ValueError(f"Evaluation setting differs: {key}")
    before, after = episode_index(baseline), episode_index(candidate)
    if before.keys() != after.keys():
        raise ValueError("Episode seed sets differ; compare the same held-out episodes")
    transitions = Counter()
    rows = []
    groups = {key: [] for key in ("new_timeouts", "regressed_successes", "recovered_successes",
                                  "persistent_failures", "new_safety_events")}
    for seed in sorted(before):
        old, new = before[seed], after[seed]
        previous, current = old["event"], new["event"]
        transitions[f"{previous} -> {current}"] += 1
        if current == "timeout" and previous != "timeout":
            groups["new_timeouts"].append(seed)
        if previous == "success" and current != "success":
            groups["regressed_successes"].append(seed)
        if previous != "success" and current == "success":
            groups["recovered_successes"].append(seed)
        if previous != "success" and current != "success":
            groups["persistent_failures"].append(seed)
        if current in ("collision", "cliff") and current != previous:
            groups["new_safety_events"].append(seed)
        if previous != current or current != "success":
            rows.append({"seed": seed, "baseline_event": previous, "candidate_event": current,
                         "baseline_steps": old.get("steps"), "candidate_steps": new.get("steps"),
                         "baseline_reasons": old.get("intervention_reasons", {}),
                         "candidate_reasons": new.get("intervention_reasons", {}),
                         "baseline_no_route_steps": old.get("no_route_steps"),
                         "candidate_no_route_steps": new.get("no_route_steps")})
    return {"episodes": len(before), "baseline_outcomes": dict(Counter(r["event"] for r in before.values())),
            "candidate_outcomes": dict(Counter(r["event"] for r in after.values())),
            "transitions": dict(sorted(transitions.items())), **groups, "changed_or_failed_records": rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--candidate", required=True, type=Path)
    parser.add_argument("--output", type=Path, help="New comparison file; existing files are never overwritten")
    args = parser.parse_args()
    try:
        result = compare_reports(json.loads(args.baseline.read_text()), json.loads(args.candidate.read_text()))
        result.update(baseline=str(args.baseline), candidate=str(args.candidate))
        if args.output:
            with args.output.open("x") as file:
                json.dump(result, file, indent=2)
    except (OSError, ValueError, TypeError, AttributeError) as exc:
        parser.error(str(exc))
    for key, value in result.items():
        if key != "changed_or_failed_records":
            print(f"{key}: {value}")


if __name__ == "__main__":
    main()
