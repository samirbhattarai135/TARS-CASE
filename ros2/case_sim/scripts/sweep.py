#!/usr/bin/env python3
"""Latin-hypercube robustness sweep for the CASE balance simulation.

No physical parameter of this robot has been measured. A single run with
estimated parameters answers "do these gains balance one guessed robot". This
sweep answers the question that matters: what fraction of the plausible CASE
robots these gains keep upright, and how much hunt they produce doing it.

FIRST STEP ON A NEW CONTAINER -- benchmark before sizing::

    ./sweep.py --benchmark

That runs ONE headless simulation at nominal parameters, reports its wall-clock
duration and the achieved real-time factor, parses its result file, and exits.
The sweep size is decided from that number; do not guess it. Then::

    ./sweep.py --runs 200 --out results.csv
    ./analyze.py results.csv

Validation experiment (the claim at case_voice/balance_control.cpp:39)::

    ./sweep.py --runs 200 --set kp=60 --out kp60.csv
    ./sweep.py --runs 200 --set kp=15 --out kp15.csv
    ./analyze.py --compare kp60.csv kp15.csv

Both use the same seed, so the two CSVs hold the SAME 200 robots and
analyze.py pairs them run for run.

LAUNCH CONTRACT
---------------
Every run is a fresh process::

    ros2 launch case_sim case_sim.launch.py \\
        gui:=false result_file:=<path> seed:=<int> <name>:=<value> ...

The arguments emitted are exactly:

* ``gui:=false`` -- headless, on every sweep and benchmark run
* ``result_file`` -- absolute path the run must write its metrics JSON to
* ``seed`` -- integer; every stochastic element of the run derives from it.
  It is ``--seed + run_index``, so the CSV records the master ``--seed`` and a
  run's own seed is recoverable as ``seed + run_index``.
* one argument per parameter carrying a ``range:`` in
  ``config/sweep_ranges.yaml``, named with that leaf key verbatim -- flat, not
  dotted, not prefixed by its block: ``total_mass_kg``,
  ``com_height_above_axle_m``, ``wheel_radius_m``, ``wheel_separation_m``,
  ``body_depth_m``, ``wheel_friction``, ``torque_constant_kt``,
  ``back_emf_constant_kv``, ``resistance_ohm``, ``supply_voltage_v``,
  ``delay_samples``, ``angle_noise_sigma_deg``, ``rotation_artifact_enabled``,
  ``rotation_artifact_gain_deg_s2``, ``lean_deg``, ``impulse_magnitude_ns``.
  That list is what the YAML holds today; the YAML is authoritative and this
  driver rereads it, so adding or renaming a range changes the arguments
  emitted without any edit here.
* anything added with ``--set k=v``, passed through verbatim

Nothing else. Every other quantity (gains, gate band, body width, IMU offset,
run protocol, physics step) is read from ``config/sweep_ranges.yaml`` by the
launch file itself, so the two entry points cannot drift apart. The launch file
must declare a launch argument for each name above, and for any scalar
``--set`` is expected to override (``kp``, ``ki``, ``kd``, ``deadzone_pwm``)
defaulting to its YAML value.

Booleans are emitted lowercase (``rotation_artifact_enabled:=true``), which is
what ROS launch substitutions expect. Every value is formatted exactly once, so
the string in argv and the string in the CSV cannot disagree.

Each run is launched with its own ``ROS_DOMAIN_ID`` and ``GZ_PARTITION`` so
that ``--jobs`` above 1 cannot let one run's supervisor talk to another run's
controller manager.

RESULT CONTRACT
---------------
Each run writes ``result_file`` as JSON::

    {"outcome": "pass|fail|error", "passed": true, "detail": "",
     "rms_pitch_error_deg": 0.0, "peak_pitch_deg": 0.0, "recovery_time_s": 0.0,
     "mean_abs_pwm": 0.0, "escaped_gate_at_s": null, "samples": 0}

The simulation owns these metrics because it owns the pitch trace: pass is
``gate_lower_deg < pitch < gate_upper_deg`` for the full run (the firmware's own
give-up band, read from the YAML by the sim, not invented here), and
``rms_pitch_error_deg`` is the hunt amplitude measured after
``settling_window_s`` only. This driver does not recompute them -- two reducers
for one metric is how a sweep and a simulation come to disagree about what
"pass" means -- but it does not trust the file either. It validates the schema,
cross-checks ``passed`` against ``outcome`` and ``escaped_gate_at_s``, and
records anything inconsistent as ``error``.

``outcome`` in the CSV is ``pass``, ``fail``, ``error``, ``timeout`` (wall-clock
limit hit with no usable result file -- never a pass) or ``skipped``
(``--dry-run``). Result files are kept at ``<out-stem>_results/run_NNNN.json``;
they are the first thing wanted when a row says ``error``.

Offline checks (no ROS, no Gazebo needed)::

    ./sweep.py --self-test
    ./sweep.py --dry-run --runs 8 --out /tmp/params.csv
"""

import argparse
import csv
import json
import logging
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

LOG = logging.getLogger("case_sim.sweep")

DEFAULT_SEED = 20260915
LAUNCH_PACKAGE = "case_sim"
LAUNCH_FILE = "case_sim.launch.py"

# Launch arguments this driver sets itself; --set may not shadow them.
RESERVED_LAUNCH_ARGS = ("gui", "result_file", "seed")

NUMERIC_METRICS = (
    "rms_pitch_error_deg",
    "peak_pitch_deg",
    "recovery_time_s",
    "mean_abs_pwm",
)
RESULT_COLUMNS = (
    "passed",
    *NUMERIC_METRICS,
    "outcome",
    "escaped_gate_at_s",
    "samples",
    "detail",
)


# --------------------------------------------------------------------------
# Parameter ranges
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class ParamSpec:
    """One swept parameter: where it lives in the YAML and how to sample it."""

    name: str
    path: str
    low: float
    high: float
    kind: str  # "float" | "int" | "bool"
    nominal: object

    def from_unit(self, u: float):
        """Map a sample from [0, 1) onto this parameter's declared range."""
        if self.kind == "bool":
            return bool(u >= 0.5)
        if self.kind == "int":
            # Inclusive integer range: each of the (high - low + 1) values takes
            # an equal share of the unit interval.
            span = int(self.high) - int(self.low) + 1
            return int(min(int(self.low) + int(u * span), int(self.high)))
        return float(self.low + u * (self.high - self.low))


def find_ranges_file(explicit: str | None) -> Path:
    """Locate config/sweep_ranges.yaml from the source tree or an install tree."""
    if explicit:
        path = Path(explicit).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"ranges file not found: {path}")
        return path

    here = Path(__file__).resolve().parent
    candidates = (
        here.parent / "config" / "sweep_ranges.yaml",  # source tree: scripts/..
        here.parent.parent / "share" / LAUNCH_PACKAGE / "config" / "sweep_ranges.yaml",
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        "sweep_ranges.yaml not found; looked in "
        + ", ".join(str(c) for c in candidates)
        + " -- pass --ranges explicitly"
    )


def load_ranges(path: Path) -> dict:
    # Imported here rather than at module scope so --self-test runs on a bare
    # interpreter; PyYAML ships with ROS 2 wherever the sweep actually runs.
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError(
            "PyYAML is required to read sweep_ranges.yaml (it ships with ROS 2); "
            "--self-test runs without it"
        ) from exc
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def discover_specs(ranges: dict) -> list[ParamSpec]:
    """Every leaf carrying a `range:` is swept. Nothing is listed twice here."""
    specs: list[ParamSpec] = []
    seen: dict[str, str] = {}

    def walk(node, path: str) -> None:
        if not isinstance(node, dict):
            return
        if "range" in node:
            name = path.rsplit(".", 1)[-1]
            if name in seen:
                raise ValueError(
                    f"duplicate swept parameter name {name!r} at {path} and {seen[name]}; "
                    "launch argument names are the leaf keys and must be unique"
                )
            seen[name] = path
            specs.append(_make_spec(name, path, node))
            return
        for key, value in node.items():
            walk(value, f"{path}.{key}" if path else key)

    walk(ranges, "")
    if not specs:
        raise ValueError("no parameters with a 'range' found in the ranges file")
    collisions = sorted(seen.keys() & set(RESERVED_LAUNCH_ARGS))
    if collisions:
        raise ValueError(
            f"swept parameter(s) {collisions} collide with launch arguments this "
            "driver sets itself"
        )
    return specs


def _make_spec(name: str, path: str, node: dict) -> ParamSpec:
    bounds = node["range"]
    if not isinstance(bounds, (list, tuple)) or len(bounds) != 2:
        raise ValueError(f"{path}.range must be a two-element list, got {bounds!r}")
    low, high = bounds
    # bool before int: in Python a bool IS an int, and a boolean parameter that
    # slips through as an integer range comes out of the sweep as 0/1.
    if isinstance(low, bool) or isinstance(high, bool):
        kind = "bool"
    elif isinstance(low, int) and isinstance(high, int):
        kind = "int"
    elif isinstance(low, (int, float)) and isinstance(high, (int, float)):
        kind = "float"
    else:
        raise ValueError(f"{path}.range holds a non-numeric bound: {bounds!r}")
    if kind != "bool" and not high > low:
        raise ValueError(f"{path}.range is not increasing: {bounds!r}")
    return ParamSpec(
        name=name,
        path=path,
        low=float(low),
        high=float(high),
        kind=kind,
        nominal=node.get("nominal"),
    )


def run_duration_s(ranges: dict) -> float:
    return float(ranges["run_protocol"]["duration_s"])


# --------------------------------------------------------------------------
# Latin hypercube sampling
# --------------------------------------------------------------------------


def latin_hypercube(n: int, dimensions: int, rng: np.random.Generator) -> np.ndarray:
    """N x D matrix in [0, 1): one sample per stratum, dimensions permuted.

    Independent uniform draws leave holes at the run counts this sweep can
    afford. Stratifying guarantees every dimension is covered evenly however
    few runs the container's real-time factor turns out to permit.
    """
    if n < 1 or dimensions < 1:
        raise ValueError(f"latin_hypercube needs n >= 1 and d >= 1, got {n}, {dimensions}")
    strata = (np.arange(n)[:, None] + rng.random((n, dimensions))) / n
    for column in range(dimensions):
        strata[:, column] = rng.permutation(strata[:, column])
    return strata


def sample_parameters(
    specs: list[ParamSpec], runs: int, seed: int
) -> list[dict[str, object]]:
    rng = np.random.default_rng(seed)
    unit = latin_hypercube(runs, len(specs), rng)
    return [
        {spec.name: spec.from_unit(float(unit[row, col])) for col, spec in enumerate(specs)}
        for row in range(runs)
    ]


def nominal_parameters(specs: list[ParamSpec]) -> dict[str, object]:
    """Nominal values, typed exactly as a sampled set would be."""
    values: dict[str, object] = {}
    for spec in specs:
        if spec.nominal is None:
            raise ValueError(f"{spec.path} has a range but no nominal")
        if spec.kind == "bool":
            values[spec.name] = bool(spec.nominal)
        elif spec.kind == "int":
            values[spec.name] = int(spec.nominal)
        else:
            values[spec.name] = float(spec.nominal)
    return values


def format_value(value: object) -> str:
    """One formatting path, so argv and the CSV can never disagree."""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return "" if math.isnan(value) else repr(value)
    if value is None:
        return ""
    return str(value)


# --------------------------------------------------------------------------
# Result parsing -- a trust boundary, not a convenience
# --------------------------------------------------------------------------


class ResultError(ValueError):
    """The run's result file is missing, malformed, or self-contradictory."""


def _numeric(value, field: str) -> float:
    if value is None:
        return math.nan
    # bool is an int in Python; a boolean where a metric belongs is a schema bug.
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ResultError(f"{field} is not numeric: {value!r}")
    return float(value)


def parse_result(path: Path) -> dict[str, object]:
    """Validate one run's JSON and return the metric half of its CSV row."""
    try:
        with path.open("r", encoding="utf-8") as handle:
            payload = json.load(handle)
    except FileNotFoundError as exc:
        raise ResultError(f"no result file at {path}") from exc
    except (OSError, json.JSONDecodeError) as exc:
        raise ResultError(f"unreadable result file {path}: {exc}") from exc

    if not isinstance(payload, dict):
        raise ResultError(f"result file {path} is not a JSON object")
    for key in ("outcome", "passed"):
        if key not in payload:
            raise ResultError(f"result file {path} has no {key!r}")

    outcome = payload["outcome"]
    passed = payload["passed"]
    if outcome not in ("pass", "fail", "error"):
        raise ResultError(f"unknown outcome {outcome!r}")
    if not isinstance(passed, bool):
        raise ResultError(f"passed is not a boolean: {passed!r}")
    if passed != (outcome == "pass"):
        raise ResultError(f"passed={passed} contradicts outcome={outcome!r}")

    escaped = payload.get("escaped_gate_at_s")
    escaped_at = _numeric(escaped, "escaped_gate_at_s")
    # Leaving the gate means applyMotorControl() coasted the motors; a run that
    # reports both an escape and a pass has a broken reducer, and a broken
    # reducer reporting success is the one failure this sweep must never absorb.
    if passed and escaped is not None:
        raise ResultError(f"passed=true but the gate was left at t={escaped_at}")

    samples = payload.get("samples")
    sample_count = _numeric(samples, "samples")
    if passed and sample_count == 0:
        raise ResultError("passed=true on a run that recorded no samples")

    row: dict[str, object] = {
        "passed": passed,
        "outcome": outcome,
        "escaped_gate_at_s": escaped_at,
        "samples": int(sample_count) if not math.isnan(sample_count) else "",
        "detail": str(payload.get("detail", "")),
    }
    for field in NUMERIC_METRICS:
        if outcome in ("pass", "fail") and field not in payload:
            raise ResultError(f"result file {path} has no {field!r}")
        row[field] = _numeric(payload.get(field), field)
    return row


def failed_row(outcome: str, detail: str) -> dict[str, object]:
    row: dict[str, object] = {column: "" for column in RESULT_COLUMNS}
    row.update({"passed": False, "outcome": outcome, "detail": detail})
    return row


# --------------------------------------------------------------------------
# Run execution
# --------------------------------------------------------------------------


def launch_argv(
    params: dict[str, object], result_file: Path, seed: int, overrides: dict[str, str]
) -> list[str]:
    argv = [
        "ros2",
        "launch",
        LAUNCH_PACKAGE,
        LAUNCH_FILE,
        "gui:=false",
        f"result_file:={result_file}",
        f"seed:={seed}",
    ]
    argv += [f"{name}:={format_value(value)}" for name, value in params.items()]
    argv += [f"{name}:={value}" for name, value in overrides.items()]
    return argv


def _terminate_group(process: subprocess.Popen) -> None:
    """Escalate on the whole process group.

    ros2 launch is a supervisor: killing it alone strands gzserver and the
    controller manager, and stranded Gazebo processes on a shared container
    poison every run that follows. SIGINT first, which ros2 launch propagates
    for a clean Gazebo shutdown.
    """
    for sig, grace in ((signal.SIGINT, 10.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(os.getpgid(process.pid), sig)
        except (ProcessLookupError, PermissionError):
            return
        try:
            process.wait(timeout=grace)
            return
        except subprocess.TimeoutExpired:
            continue


def run_environment(index: int) -> dict[str, str]:
    """Fence each run onto its own ROS domain and Gazebo transport partition.

    Concurrent runs otherwise share a controller_manager name, a /case/pitch
    topic and a world name, so --jobs would let one run's supervisor read
    another run's controller -- the silent cross-contamination this harness uses
    a fresh process to avoid in the first place. Applied unconditionally: it
    also fences a run off from any gzserver left over from a killed predecessor.
    """
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(1 + index % 100)  # 0-101 is portable; 0 left free
    env["GZ_PARTITION"] = f"case_sim_{index}"
    return env


def execute_run(argv: list[str], timeout_s: float, index: int) -> tuple[bool, float, str]:
    """Run one simulation to completion. Returns (timed_out, wall_seconds, log)."""
    started = time.monotonic()
    # ponytail: a fresh process per run costs ~3-5 s of Gazebo startup. In-process
    # world reset is faster but leaks controller and physics state between runs,
    # corrupting results silently rather than loudly; upgrade to in-process reset
    # only if the benchmark shows that startup cost dominating.
    process = subprocess.Popen(
        argv,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=run_environment(index),
    )
    try:
        output = process.communicate(timeout=timeout_s)[0]
    except subprocess.TimeoutExpired:
        _terminate_group(process)
        try:
            output = process.communicate(timeout=10.0)[0] or ""
        except subprocess.TimeoutExpired:
            output = ""
        return True, time.monotonic() - started, output
    return False, time.monotonic() - started, output or ""


def run_single(
    index: int,
    params: dict[str, object],
    result_dir: Path,
    overrides: dict[str, str],
    seed: int,
    timeout_s: float,
) -> dict[str, object]:
    result_file = result_dir / f"run_{index:04d}.json"
    # A rerun into the same --out must not read the previous sweep's result and
    # report it as fresh.
    result_file.unlink(missing_ok=True)

    argv = launch_argv(params, result_file, seed + index, overrides)
    LOG.debug("run %d: %s", index, " ".join(argv))
    timed_out, wall, output = execute_run(argv, timeout_s, index)

    row: dict[str, object] = {"run_index": index, "seed": seed, "wall_s": round(wall, 3)}
    row.update(params)
    row.update(overrides)

    try:
        metrics = parse_result(result_file)
    except ResultError as exc:
        if timed_out:
            LOG.warning("run %d timed out after %.1f s (%s)", index, wall, exc)
            row.update(failed_row("timeout", f"killed after {wall:.1f}s"))
        else:
            LOG.error("run %d: %s\n%s", index, exc, output[-2000:])
            row.update(failed_row("error", str(exc)))
        return row

    if timed_out:
        # A usable result plus a hung process is a Gazebo shutdown bug, not a
        # lost run: keep the metrics and make the hang visible.
        LOG.warning("run %d wrote its result but had to be killed at %.1f s", index, wall)
        metrics["detail"] = (str(metrics["detail"]) + " [launch hung; killed]").strip()
    LOG.info("run %d: %s (%.1f s)", index, metrics["outcome"], wall)
    row.update(metrics)
    return row


# --------------------------------------------------------------------------
# CSV output
# --------------------------------------------------------------------------


class ResultWriter:
    """Append-and-flush per row, so a killed sweep keeps what it finished."""

    def __init__(self, path: Path, fieldnames: list[str]) -> None:
        self._lock = threading.Lock()
        self._handle = path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._handle, fieldnames=fieldnames)
        self._writer.writeheader()
        self._handle.flush()

    def write(self, row: dict[str, object]) -> None:
        formatted = {key: format_value(value) for key, value in row.items()}
        with self._lock:
            self._writer.writerow(formatted)
            self._handle.flush()

    def close(self) -> None:
        self._handle.close()


def result_fieldnames(specs: list[ParamSpec], overrides: dict[str, str]) -> list[str]:
    return (
        ["run_index", "seed"]
        + [spec.name for spec in specs]
        + list(overrides)
        + list(RESULT_COLUMNS)
        + ["wall_s"]
    )


# --------------------------------------------------------------------------
# Entry points
# --------------------------------------------------------------------------


def require_ros() -> None:
    if shutil.which("ros2") is None:
        raise RuntimeError(
            "ros2 is not on PATH -- source the workspace before sweeping "
            "(--dry-run and --self-test do not need it)"
        )


def do_benchmark(args, specs: list[ParamSpec], duration_s: float, overrides) -> int:
    require_ros()
    params = nominal_parameters(specs)
    with tempfile.TemporaryDirectory(prefix="case_sim_benchmark_") as tmp:
        row = run_single(
            index=0,
            params=params,
            result_dir=Path(tmp),
            overrides=overrides,
            seed=args.seed,
            timeout_s=args.timeout,
        )
    wall = float(row["wall_s"])
    print("benchmark: one headless run at nominal parameters")
    print(f"  outcome            : {row['outcome']}")
    print(f"  wall clock         : {wall:.1f} s  (includes Gazebo startup)")
    print(f"  simulated duration : {duration_s:.1f} s")
    if wall > 0:
        print(f"  effective RTF      : {duration_s / wall:.2f} x  (startup included)")
        for runs in (100, 200, 500):
            print(f"  {runs:4d} runs serial  : {runs * wall / 60.0:.1f} min")
    if row["outcome"] in ("pass", "fail"):
        print(f"  rms hunt (deg)     : {row['rms_pitch_error_deg']}")
        print(f"  peak pitch (deg)   : {row['peak_pitch_deg']}")
        print(f"  mean |pwm|         : {row['mean_abs_pwm']}")
        print("  result contract verified end to end; size the sweep from the RTF above.")
        return 0
    print(f"  no usable result ({row['detail']}) -- fix the launch before sweeping.")
    return 1


def do_sweep(args, specs: list[ParamSpec], overrides) -> int:
    if not args.dry_run:
        require_ros()
    samples = sample_parameters(specs, args.runs, args.seed)
    out_path = Path(args.out).expanduser().resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    writer = ResultWriter(out_path, result_fieldnames(specs, overrides))

    try:
        if args.dry_run:
            for index, params in enumerate(samples):
                row: dict[str, object] = {"run_index": index, "seed": args.seed, "wall_s": ""}
                row.update(params)
                row.update(overrides)
                row.update(failed_row("skipped", "dry run: nothing launched"))
                writer.write(row)
            print(f"{len(samples)} parameter sets written to {out_path} (nothing launched)")
            return 0

        result_dir = out_path.with_name(out_path.stem + "_results")
        result_dir.mkdir(parents=True, exist_ok=True)
        LOG.info("per-run result files kept in %s", result_dir)

        def work(item):
            index, params = item
            return run_single(
                index=index,
                params=params,
                result_dir=result_dir,
                overrides=overrides,
                seed=args.seed,
                timeout_s=args.timeout,
            )

        counts: dict[str, int] = {}
        if args.jobs > 1:
            from concurrent.futures import ThreadPoolExecutor

            with ThreadPoolExecutor(max_workers=args.jobs) as pool:
                rows = pool.map(work, enumerate(samples))
                for row in rows:
                    writer.write(row)
                    counts[row["outcome"]] = counts.get(row["outcome"], 0) + 1
        else:
            for item in enumerate(samples):
                row = work(item)
                writer.write(row)
                counts[row["outcome"]] = counts.get(row["outcome"], 0) + 1

        print(f"{len(samples)} runs written to {out_path}")
        for outcome in sorted(counts):
            print(f"  {outcome:8s} {counts[outcome]}")
        return 0
    finally:
        writer.close()


def parse_overrides(pairs: list[str], specs: list[ParamSpec]) -> dict[str, str]:
    swept = {spec.name for spec in specs}
    overrides: dict[str, str] = {}
    for pair in pairs:
        if "=" not in pair:
            raise ValueError(f"--set expects name=value, got {pair!r}")
        name, value = pair.split("=", 1)
        name = name.strip()
        if name in swept:
            raise ValueError(
                f"--set {name}= collides with a swept parameter; remove its range "
                "from sweep_ranges.yaml if it should be fixed instead"
            )
        if name in RESERVED_LAUNCH_ARGS:
            raise ValueError(f"--set {name}= collides with an argument the driver sets")
        if name in overrides:
            raise ValueError(f"--set {name}= given twice")
        overrides[name] = value.strip()
    return overrides


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Latin-hypercube robustness sweep for the CASE balance simulation.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--runs", type=int, default=100, help="number of sweep runs")
    parser.add_argument("--jobs", type=int, default=1, help="concurrent runs (default serial)")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED, help="LHS seed")
    parser.add_argument("--out", default="results.csv", help="results CSV path")
    parser.add_argument("--ranges", default=None, help="override sweep_ranges.yaml path")
    parser.add_argument(
        "--timeout",
        type=float,
        default=120.0,
        help="wall-clock seconds per run before it is killed and recorded as a timeout",
    )
    parser.add_argument(
        "--set",
        dest="overrides",
        action="append",
        default=[],
        metavar="NAME=VALUE",
        help="extra launch argument, e.g. --set kp=15 (recorded as a CSV column)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="generate and write the parameter sets without launching anything",
    )
    parser.add_argument(
        "--benchmark",
        action="store_true",
        help="run ONE headless simulation, report its real-time factor, and exit",
    )
    parser.add_argument("--self-test", action="store_true", help="run offline checks and exit")
    parser.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    if args.self_test:
        return self_test()

    if args.runs < 1:
        LOG.error("--runs must be at least 1")
        return 2
    if args.jobs < 1:
        LOG.error("--jobs must be at least 1")
        return 2

    try:
        ranges_path = find_ranges_file(args.ranges)
        ranges = load_ranges(ranges_path)
        specs = discover_specs(ranges)
        duration_s = run_duration_s(ranges)
        overrides = parse_overrides(args.overrides, specs)
    except (OSError, ValueError, KeyError, RuntimeError) as exc:
        LOG.error("%s", exc)
        return 2
    LOG.info("ranges: %s (%d swept parameters)", ranges_path, len(specs))

    try:
        if args.benchmark:
            return do_benchmark(args, specs, duration_s, overrides)
        return do_sweep(args, specs, overrides)
    except RuntimeError as exc:
        LOG.error("%s", exc)
        return 2


# --------------------------------------------------------------------------
# Offline self-test
# --------------------------------------------------------------------------


def _write_json(path: Path, payload: dict) -> Path:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle)
    return path


def self_test() -> int:
    specs = [
        ParamSpec("total_mass_kg", "plant.total_mass_kg", 0.60, 1.20, "float", 0.85),
        ParamSpec("delay_samples", "sensor.delay_samples", 1, 3, "int", 2),
        ParamSpec(
            "rotation_artifact_enabled",
            "sensor.rotation_artifact_enabled",
            0,
            1,
            "bool",
            True,
        ),
        ParamSpec("lean_deg", "controller.lean_deg", -2.0, 2.0, "float", 0.0),
    ]
    runs = 12

    # -- LHS shape, bounds, and stratum occupancy --------------------------
    rng = np.random.default_rng(DEFAULT_SEED)
    unit = latin_hypercube(runs, len(specs), rng)
    assert unit.shape == (runs, len(specs)), unit.shape
    assert np.all(unit >= 0.0) and np.all(unit < 1.0)
    for column in range(unit.shape[1]):
        occupied = set(np.floor(unit[:, column] * runs).astype(int).tolist())
        assert occupied == set(range(runs)), f"dimension {column} missed strata: {occupied}"

    samples = sample_parameters(specs, runs, DEFAULT_SEED)
    assert len(samples) == runs
    for row in samples:
        assert set(row) == {spec.name for spec in specs}
        assert 0.60 <= row["total_mass_kg"] <= 1.20, row
        assert 1 <= row["delay_samples"] <= 3, row
        assert -2.0 <= row["lean_deg"] <= 2.0, row

    # -- types: not numpy scalars, not floats ------------------------------
    for row in samples:
        assert type(row["delay_samples"]) is int, type(row["delay_samples"])
        assert type(row["rotation_artifact_enabled"]) is bool, type(
            row["rotation_artifact_enabled"]
        )
        assert type(row["total_mass_kg"]) is float, type(row["total_mass_kg"])
    assert {row["delay_samples"] for row in samples} == {1, 2, 3}
    assert {row["rotation_artifact_enabled"] for row in samples} == {False, True}
    assert format_value(True) == "true" and format_value(False) == "false"
    assert format_value(2) == "2" and format_value(0.5) == "0.5"
    assert format_value(math.nan) == "" and format_value(None) == ""

    # -- determinism -------------------------------------------------------
    assert sample_parameters(specs, runs, DEFAULT_SEED) == samples, "seed is not reproducible"
    assert sample_parameters(specs, runs, DEFAULT_SEED + 1) != samples, "seed has no effect"

    # -- spec discovery types the ranges as written in the YAML ------------
    discovered = discover_specs(
        {
            "plant": {"total_mass_kg": {"nominal": 0.85, "range": [0.60, 1.20]}},
            "sensor": {
                "delay_samples": {"nominal": 2, "range": [1, 3]},
                "rotation_artifact_enabled": {"nominal": True, "range": [False, True]},
            },
            "controller": {"kp": 60.0},
        }
    )
    kinds = {spec.name: spec.kind for spec in discovered}
    assert kinds == {
        "total_mass_kg": "float",
        "delay_samples": "int",
        "rotation_artifact_enabled": "bool",
    }, kinds
    nominals = nominal_parameters(discovered)
    assert type(nominals["delay_samples"]) is int
    assert type(nominals["rotation_artifact_enabled"]) is bool

    # -- launch argv is the documented contract ----------------------------
    argv = launch_argv(
        {"delay_samples": 2, "rotation_artifact_enabled": True, "lean_deg": -1.5},
        Path("/tmp/run_0000.json"),
        seed=7,
        overrides={"kp": "15"},
    )
    assert argv[:4] == ["ros2", "launch", LAUNCH_PACKAGE, LAUNCH_FILE], argv
    assert "gui:=false" in argv and "result_file:=/tmp/run_0000.json" in argv, argv
    assert "seed:=7" in argv and "kp:=15" in argv, argv
    assert "rotation_artifact_enabled:=true" in argv, argv
    assert "delay_samples:=2" in argv and "lean_deg:=-1.5" in argv, argv
    assert not any(arg.startswith("trace_csv:=") for arg in argv), argv

    # -- overrides may not shadow a swept or driver-owned argument ---------
    assert parse_overrides(["kp=15"], specs) == {"kp": "15"}
    for bad in ("lean_deg=2", "result_file=/tmp/x", "gui=true"):
        try:
            parse_overrides([bad], specs)
        except ValueError:
            continue
        raise AssertionError(f"--set {bad} must be refused")

    # -- the result file is a trust boundary -------------------------------
    good = {
        "outcome": "pass",
        "passed": True,
        "detail": "",
        "rms_pitch_error_deg": 0.42,
        "peak_pitch_deg": 3.1,
        "recovery_time_s": 1.25,
        "mean_abs_pwm": 37.5,
        "escaped_gate_at_s": None,
        "samples": 1001,
    }
    with tempfile.TemporaryDirectory(prefix="case_sim_selftest_") as tmp:
        tmp_path = Path(tmp)
        parsed = parse_result(_write_json(tmp_path / "pass.json", good))
        assert parsed["passed"] is True and parsed["outcome"] == "pass"
        assert parsed["rms_pitch_error_deg"] == 0.42 and parsed["samples"] == 1001
        assert math.isnan(parsed["escaped_gate_at_s"])

        fell = dict(good, outcome="fail", passed=False, escaped_gate_at_s=6.2)
        parsed = parse_result(_write_json(tmp_path / "fail.json", fell))
        assert parsed["passed"] is False and parsed["outcome"] == "fail"
        assert parsed["escaped_gate_at_s"] == 6.2

        # A run that left the gate is gone; no reducer may call that a pass.
        contradictory = dict(good, escaped_gate_at_s=6.2)
        rejected = [
            _write_json(tmp_path / "contradictory.json", contradictory),
            _write_json(tmp_path / "mismatch.json", dict(good, outcome="fail")),
            _write_json(tmp_path / "unknown.json", dict(good, outcome="crashed")),
            _write_json(tmp_path / "nonbool.json", dict(good, passed="yes")),
            _write_json(tmp_path / "nosamples.json", dict(good, samples=0)),
            _write_json(tmp_path / "nometric.json", {k: v for k, v in good.items() if k != "peak_pitch_deg"}),
            _write_json(tmp_path / "textmetric.json", dict(good, mean_abs_pwm="lots")),
            tmp_path / "absent.json",
        ]
        garbage = tmp_path / "garbage.json"
        garbage.write_text("{not json at all", encoding="utf-8")
        rejected.append(garbage)
        for path in rejected:
            try:
                parse_result(path)
            except ResultError:
                continue
            raise AssertionError(f"{path.name} must not parse as a usable result")

    # -- concurrent runs are fenced off from each other ---------------------
    envs = [run_environment(index) for index in (0, 1, 2)]
    assert len({env["ROS_DOMAIN_ID"] for env in envs}) == 3, envs
    assert len({env["GZ_PARTITION"] for env in envs}) == 3, envs
    assert all(0 < int(env["ROS_DOMAIN_ID"]) <= 101 for env in envs), envs

    # -- a killed run is a distinct outcome, never a pass -------------------
    timed_out = failed_row("timeout", "killed after 120.0s")
    assert timed_out["outcome"] == "timeout" and timed_out["passed"] is False
    assert failed_row("error", "boom")["passed"] is False

    print("sweep.py self-test: all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
