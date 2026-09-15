#!/usr/bin/env python3
"""Read a CASE balance sweep CSV and report what it actually says.

Usage::

    ./analyze.py results.csv
    ./analyze.py --compare kp60.csv kp15.csv
    ./analyze.py --self-test

Three questions, in the order they matter:

1. **What fraction of plausible CASE robots stayed upright?** Reported with a
   Wilson score interval, because the honest answer to "60 of 80 passed" at
   this sample size is a range, not a point. The denominator is
   ``pass + fail`` only: ``timeout``, ``error`` and ``skipped`` rows are
   harness failures, not robot failures, and are counted separately so they
   cannot masquerade as either.

2. **How much do the passing ones hunt?** Gains that technically balance while
   oscillating at several degrees are not usable gains, so the distribution of
   ``rms_pitch_error_deg`` among passers is reported alongside the fraction.

3. **Which parameters separate pass from fail?** A per-parameter comparison of
   pass-group and fail-group means, standardised by the pooled spread so the
   ranking is readable across parameters with different units. This is a
   ranking, not a model -- it says where to look, not how much each parameter
   contributes.

``--compare`` exists for the validation experiment behind the comment at
``case_voice/balance_control.cpp:39``, which claims Kp=60 stabilises more
plausible robots than Kp=15. When both CSVs carry the same ``seed`` and the
same ``run_index`` values they hold the SAME robots, so the comparison is
paired and the 2x2 table (which robots each gain set won and lost) is printed.
Otherwise it falls back to comparing two independent proportions.
"""

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

PASS_OUTCOMES = ("pass", "fail")
NON_PARAMETER_COLUMNS = frozenset(
    {
        "run_index",
        "seed",
        "wall_s",
        "passed",
        "outcome",
        "detail",
        "samples",
        "escaped_gate_at_s",
        "rms_pitch_error_deg",
        "peak_pitch_deg",
        "recovery_time_s",
        "mean_abs_pwm",
    }
)


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"{path} has no data rows")
    if "outcome" not in rows[0]:
        raise ValueError(f"{path} has no 'outcome' column -- is it a sweep CSV?")
    return rows


def numeric(rows: list[dict[str, str]], column: str) -> np.ndarray:
    """Column as floats, with booleans read back from their lowercase spelling."""
    values = []
    for row in rows:
        cell = (row.get(column) or "").strip()
        if cell == "true":
            values.append(1.0)
        elif cell == "false":
            values.append(0.0)
        elif cell == "":
            values.append(math.nan)
        else:
            try:
                values.append(float(cell))
            except ValueError:
                values.append(math.nan)
    return np.asarray(values, dtype=float)


def parameter_columns(rows: list[dict[str, str]]) -> list[str]:
    """Swept parameters and --set overrides: every column that is not a result."""
    columns = []
    for name in rows[0]:
        if name in NON_PARAMETER_COLUMNS:
            continue
        values = numeric(rows, name)
        if np.all(np.isnan(values)):
            continue  # a free-text override column carries no separation signal
        columns.append(name)
    return columns


def wilson_interval(successes: int, trials: int, z: float = 1.96) -> tuple[float, float]:
    """95% Wilson score interval.

    Wilson rather than the normal approximation because a sweep that comes back
    0/40 or 40/40 is exactly the case the normal approximation reports as a
    zero-width interval, and "100%, no uncertainty" is never true at n=40.
    """
    if trials <= 0:
        return (math.nan, math.nan)
    phat = successes / trials
    denominator = 1.0 + z * z / trials
    centre = (phat + z * z / (2 * trials)) / denominator
    spread = (
        z * math.sqrt(phat * (1.0 - phat) / trials + z * z / (4 * trials * trials))
    ) / denominator
    return (max(0.0, centre - spread), min(1.0, centre + spread))


def outcome_counts(rows: list[dict[str, str]]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for row in rows:
        outcome = (row.get("outcome") or "").strip() or "(blank)"
        counts[outcome] = counts.get(outcome, 0) + 1
    return counts


def completed(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    return [row for row in rows if (row.get("outcome") or "").strip() in PASS_OUTCOMES]


def quantiles(values: np.ndarray) -> dict[str, float]:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return {}
    keys = ("min", "p25", "median", "p75", "p90", "max")
    points = np.quantile(finite, [0.0, 0.25, 0.5, 0.75, 0.90, 1.0])
    stats = dict(zip(keys, (float(p) for p in points)))
    stats["mean"] = float(np.mean(finite))
    stats["n"] = float(finite.size)
    return stats


def separation(
    rows: list[dict[str, str]], columns: list[str]
) -> list[tuple[str, float, float, float]]:
    """(name, pass mean, fail mean, standardised difference), strongest first."""
    passed = [row for row in rows if (row.get("outcome") or "").strip() == "pass"]
    failed = [row for row in rows if (row.get("outcome") or "").strip() == "fail"]
    if not passed or not failed:
        return []
    ranked = []
    for column in columns:
        pass_values = numeric(passed, column)
        fail_values = numeric(failed, column)
        pass_values = pass_values[np.isfinite(pass_values)]
        fail_values = fail_values[np.isfinite(fail_values)]
        if pass_values.size < 2 or fail_values.size < 2:
            continue
        pass_mean = float(np.mean(pass_values))
        fail_mean = float(np.mean(fail_values))
        pooled = math.sqrt((float(np.var(pass_values)) + float(np.var(fail_values))) / 2.0)
        standardised = (pass_mean - fail_mean) / pooled if pooled > 0 else 0.0
        ranked.append((column, pass_mean, fail_mean, standardised))
    ranked.sort(key=lambda item: abs(item[3]), reverse=True)
    return ranked


# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------


def print_fraction(label: str, rows: list[dict[str, str]]) -> tuple[int, int]:
    counts = outcome_counts(rows)
    done = completed(rows)
    passes = sum(1 for row in done if row["outcome"].strip() == "pass")
    trials = len(done)

    print(f"{label}: {len(rows)} rows")
    for outcome in sorted(counts):
        print(f"  {outcome:8s} {counts[outcome]:5d}")
    excluded = len(rows) - trials
    if excluded:
        print(f"  ({excluded} row(s) excluded from the fraction: harness failures, not falls)")
    if trials == 0:
        print("  stabilised fraction: no completed runs")
        return passes, trials
    low, high = wilson_interval(passes, trials)
    print(
        f"  stabilised: {passes}/{trials} = {100.0 * passes / trials:.1f}%"
        f"  (95% Wilson {100.0 * low:.1f}-{100.0 * high:.1f}%)"
    )
    return passes, trials


def print_hunt(rows: list[dict[str, str]]) -> None:
    passed = [row for row in rows if (row.get("outcome") or "").strip() == "pass"]
    stats = quantiles(numeric(passed, "rms_pitch_error_deg"))
    print("\nhunt amplitude among passing runs (rms pitch error, deg)")
    if not stats:
        print("  no passing runs with a recorded hunt amplitude")
        return
    print(
        f"  n={int(stats['n'])}  min {stats['min']:.3f}  p25 {stats['p25']:.3f}  "
        f"median {stats['median']:.3f}  p75 {stats['p75']:.3f}  p90 {stats['p90']:.3f}  "
        f"max {stats['max']:.3f}  mean {stats['mean']:.3f}"
    )
    for column, unit in (("peak_pitch_deg", "deg"), ("recovery_time_s", "s"), ("mean_abs_pwm", "")):
        extra = quantiles(numeric(passed, column))
        if extra:
            suffix = f" {unit}" if unit else ""
            print(
                f"  {column:20s} median {extra['median']:.3f}{suffix}  "
                f"p90 {extra['p90']:.3f}{suffix}  (n={int(extra['n'])})"
            )


def print_separation(rows: list[dict[str, str]]) -> None:
    ranked = separation(rows, parameter_columns(rows))
    print("\nparameters separating pass from fail (standardised mean difference)")
    if not ranked:
        print("  needs at least two passes and two fails to compare")
        return
    print(f"  {'parameter':32s} {'pass mean':>12s} {'fail mean':>12s} {'sep':>7s}")
    for column, pass_mean, fail_mean, standardised in ranked:
        print(f"  {column:32s} {pass_mean:12.5g} {fail_mean:12.5g} {standardised:+7.2f}")
    print("  sep > 0: passing runs had the higher value. Ranking only -- not a model.")


def print_compare(path_a: Path, path_b: Path) -> None:
    rows_a = read_rows(path_a)
    rows_b = read_rows(path_b)
    passes_a, trials_a = print_fraction(f"A  {path_a.name}", rows_a)
    print()
    passes_b, trials_b = print_fraction(f"B  {path_b.name}", rows_b)

    if trials_a and trials_b:
        print(
            f"\ndifference (A - B): "
            f"{100.0 * (passes_a / trials_a - passes_b / trials_b):+.1f} percentage points"
        )

    keyed_a = {(row.get("seed"), row.get("run_index")): row for row in completed(rows_a)}
    keyed_b = {(row.get("seed"), row.get("run_index")): row for row in completed(rows_b)}
    shared = sorted(keyed_a.keys() & keyed_b.keys())
    if not shared:
        print(
            "\nrows do not pair on (seed, run_index): comparing independent proportions only. "
            "Rerun both sweeps with the same --seed and --runs to pair them."
        )
    else:
        # Same seed and run index means the same 16-dimensional robot in both
        # sweeps, so the comparison can name the robots each gain set won and
        # lost rather than only the totals.
        both = a_only = b_only = neither = 0
        for key in shared:
            in_a = keyed_a[key]["outcome"].strip() == "pass"
            in_b = keyed_b[key]["outcome"].strip() == "pass"
            both += in_a and in_b
            a_only += in_a and not in_b
            b_only += in_b and not in_a
            neither += not in_a and not in_b
        print(f"\npaired on (seed, run_index): {len(shared)} identical robots in both sweeps")
        print(f"  both upright     {both:5d}")
        print(f"  A only           {a_only:5d}   (A rescued these)")
        print(f"  B only           {b_only:5d}   (A lost these)")
        print(f"  neither          {neither:5d}")
        verdict = (
            "A stabilises more of the same robots"
            if a_only > b_only
            else "B stabilises more of the same robots"
            if b_only > a_only
            else "the two are tied on these robots"
        )
        print(f"  verdict: {verdict}")

    for label, rows in ((path_a.name, rows_a), (path_b.name, rows_b)):
        stats = quantiles(
            numeric(
                [row for row in rows if (row.get("outcome") or "").strip() == "pass"],
                "rms_pitch_error_deg",
            )
        )
        if stats:
            print(
                f"\nhunt among passers, {label}: median {stats['median']:.3f} deg, "
                f"p90 {stats['p90']:.3f} deg (n={int(stats['n'])})"
            )


def print_report(path: Path) -> None:
    rows = read_rows(path)
    print_fraction(str(path), rows)
    print_hunt(rows)
    print_separation(rows)


# --------------------------------------------------------------------------
# Offline self-test
# --------------------------------------------------------------------------


def self_test() -> int:
    # Wilson against hand-checkable anchors.
    low, high = wilson_interval(50, 100)
    assert abs((low + high) / 2 - 0.5) < 1e-9, (low, high)
    assert 0.39 < low < 0.41 and 0.59 < high < 0.61, (low, high)
    zero_low, zero_high = wilson_interval(0, 40)
    assert zero_low == 0.0 and 0.0 < zero_high < 0.12, (zero_low, zero_high)
    full_low, full_high = wilson_interval(40, 40)
    assert full_high == 1.0 and 0.88 < full_low < 1.0, (full_low, full_high)
    assert all(math.isnan(bound) for bound in wilson_interval(0, 0))

    header = (
        "run_index,seed,com_height_above_axle_m,rotation_artifact_enabled,kp,"
        "passed,rms_pitch_error_deg,peak_pitch_deg,recovery_time_s,mean_abs_pwm,"
        "outcome,escaped_gate_at_s,samples,detail"
    )
    body = [
        "0,1,0.090,true,60,true,0.40,2.0,0.5,35,pass,,1001,",
        "1,1,0.085,false,60,true,0.60,2.5,0.7,36,pass,,1001,",
        "2,1,0.050,true,60,false,,18.0,,40,fail,4.2,1001,",
        "3,1,0.046,false,60,false,,19.0,,41,fail,3.9,1001,",
        "4,1,0.070,true,60,false,,,,,timeout,,,killed after 120.0s",
        "5,1,0.070,true,60,false,,,,,error,,,no result file",
    ]
    rows = list(csv.DictReader([header] + body))

    done = completed(rows)
    assert len(done) == 4, done  # timeout and error are excluded, not failures
    passes = sum(1 for row in done if row["outcome"] == "pass")
    assert passes == 2
    assert outcome_counts(rows) == {"pass": 2, "fail": 2, "timeout": 1, "error": 1}

    # Booleans survive the round trip through their lowercase CSV spelling.
    flags = numeric(rows, "rotation_artifact_enabled")
    assert flags.tolist() == [1.0, 0.0, 1.0, 0.0, 1.0, 1.0], flags

    columns = parameter_columns(rows)
    assert "com_height_above_axle_m" in columns and "kp" in columns, columns
    assert not (set(columns) & NON_PARAMETER_COLUMNS), columns

    ranked = separation(rows, columns)
    assert ranked[0][0] == "com_height_above_axle_m", ranked
    assert ranked[0][1] > ranked[0][2] and ranked[0][3] > 0, ranked  # passers sat higher

    hunt = quantiles(numeric([row for row in rows if row["outcome"] == "pass"], "rms_pitch_error_deg"))
    assert hunt["n"] == 2 and abs(hunt["median"] - 0.5) < 1e-9, hunt

    # A dry-run CSV has no completed runs at all and must not divide by zero.
    dry = list(
        csv.DictReader([header, "0,1,0.070,true,60,false,,,,,skipped,,,dry run: nothing launched"])
    )
    assert completed(dry) == []
    assert separation(dry, parameter_columns(dry)) == []
    print_fraction("dry-run csv", dry)

    print("analyze.py self-test: all checks passed")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Report stabilised fraction, hunt distribution, and parameter separation."
    )
    parser.add_argument("csv", nargs="?", help="sweep results CSV")
    parser.add_argument(
        "--compare",
        nargs=2,
        metavar=("A.csv", "B.csv"),
        help="compare two sweeps, paired by (seed, run_index) when possible",
    )
    parser.add_argument("--self-test", action="store_true", help="run offline checks and exit")
    args = parser.parse_args(argv)

    if args.self_test:
        return self_test()
    try:
        if args.compare:
            print_compare(Path(args.compare[0]), Path(args.compare[1]))
        elif args.csv:
            print_report(Path(args.csv))
        else:
            parser.error("give a CSV, --compare A.csv B.csv, or --self-test")
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
