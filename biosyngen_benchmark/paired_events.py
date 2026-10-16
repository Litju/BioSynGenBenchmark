"""Reconstruct sampled event brackets independently from declared model states.

These are model-reference checks, never FORCE/LPT anatomical phase detection.
Missing phenomena stay missing; they are not filled from controller labels.
"""


def verify_events(reference, declared):
    times, contacts = reference["physics_times_s"], reference["contacts"]
    resolution = max(b - a for a, b in zip(times, times[1:]))
    foot_ids = [c["contact_id"] for c in reference["contact_specs"] if c["plate_frame"] is not None]
    floor_ids = [c["contact_id"] for c in reference["contact_specs"] if c["kind"] == "BAR_FLOOR"]
    findings = []

    def check(name, value, index):
        if index is None:
            outcome = "ABSENT" if value is None else "REJECTED"
            bracket = None
        else:
            bracket = [times[max(0, index - 1)], times[index]]
            outcome = "PASS" if value is not None and bracket[0] - resolution <= value <= bracket[1] + resolution else "REJECTED"
        findings.append({"event": name, "outcome": outcome, "declared_time_s": value, "sampled_bracket_s": bracket})

    if "events" in declared:
        open_feet = [all(c[f] == "OPEN" for f in foot_ids) for c in contacts]
        takeoff = next((i for i in range(1, len(contacts) - 2) if all(open_feet[i:i + 3])), None)
        touchdown = next((i for i in range((takeoff or 0) + 1, len(contacts)) if not open_feet[i]), None) if takeoff else None
        check("takeoff", declared["events"]["takeoff"]["time_s"], takeoff)
        check("touchdown", declared["events"]["touchdown"]["time_s"], touchdown)
        # Reversal is independently bracketed by total vertical momentum.
        momentum = reference["vertical_momentum_kg_m_s"]
        if reference.get("protocol") != "sj":
            reversal = next((i for i in range(1, takeoff or len(times)) if momentum[i - 1] < -1e-6 * reference["mass_kg"] and momentum[i] >= 0), None)
            check("com_velocity_reversal", declared["events"]["com_velocity_reversal"]["time_s"], reversal)
    if floor_ids and "lift_off_s" in declared:
        floor = floor_ids[0]
        lift = next((i for i, c in enumerate(contacts) if c[floor] == "OPEN"), None)
        check("bar_floor_lift_off", declared["lift_off_s"], lift)
        ret = next((i for i in range((lift or 0) + 1, len(contacts)) if contacts[i][floor] != "OPEN"), None) if lift else None
        check("bar_floor_return", declared.get("bar_floor_return_s", declared.get("bar_return_s")), ret)
    if "reps" in declared:
        body = reference["reference_attachment"]["body_id"]
        velocities = [s["bodies"][body]["velocity_m_s"][1] for s in reference["states"]]
        heights = [s["bodies"][body]["position_m"][1] for s in reference["states"]]
        start = 1
        for rep in declared["reps"]:
            onset_value = rep.get("descent_onset_s", rep.get("lowering_onset_s"))
            onset = next((i for i in range(start, len(times)) if velocities[i] < -0.02), None)
            check(f"rep{rep['index']}_descent", onset_value, onset)
            bottom = next((i for i in range((onset or 0) + 1, len(times)) if velocities[i - 1] < 0 <= velocities[i]), None) if onset else None
            check(f"rep{rep['index']}_bottom", rep["bottom_reversal_s"], bottom)
            end_value = rep.get("rep_end_s", rep.get("lockout_s"))
            # These are published model thresholds, not anatomical lockout.
            end = None
            if bottom is not None:
                if "rep_end_s" in rep:
                    end = next((i for i in range(bottom + 1, len(times)) if
                        abs(velocities[i]) <= 0.02 and heights[i] >= 0.98 * heights[0]), None)
                else:
                    end = next((i for i in range(bottom + 1, len(times)) if velocities[i] <= 0.01 and
                        heights[i] > heights[bottom] + 0.5 * (heights[onset] - heights[bottom])), None)
            check(f"rep{rep['index']}_model_threshold_completion", end_value, end)
            if end_value is not None:
                start = next((i for i, t in enumerate(times) if t > end_value), len(times))
    return {"findings": findings, "passed": bool(findings) and all(f["outcome"] == "PASS" for f in findings),
        "scope": "SAMPLED_PHYSICAL_REFERENCE_ONLY", "time_resolution_s": resolution,
        "missing_events": sum(f["outcome"] == "ABSENT" for f in findings),
        "anatomical_and_product_event_qualification": "NOT_ESTABLISHED"}
