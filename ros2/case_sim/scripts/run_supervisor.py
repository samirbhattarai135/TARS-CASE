#!/usr/bin/env python3
"""Drives one simulation run and emits its metrics.

One run is: settle, take a push, keep standing or fall over. This node owns that
timeline. It waits for the controller to go active, starts the clock, applies
the disturbance impulse at the scheduled simulation time, and writes a single
JSON result before shutting the run down.

The pass criterion is the firmware's own give-up band, not a threshold invented
here: outside (gate_lower, gate_upper) degrees, applyMotorControl() coasts the
motors and the robot is gone. A pass in simulation therefore means what a pass
means on the bench.

Startup order matters and is the reason this node exists rather than a timer in
the launch file. Nothing is measured until the control loop is live, and getting
there takes three steps that cannot be reordered:

1. The robot spawns UPRIGHT, with physics running at real time. Upright is an
   equilibrium, so it keeps standing through the second or two the spawner needs.
   The world cannot start paused: gz_ros2_control gates controller_manager
   update() on `sim_period >= control_period`, and sim_period is 0 while paused,
   so a paused world can never activate a controller at all.
2. Once the controller reports active, the robot is tilted into its real initial
   condition and the real-time factor is released to the run's setting.
3. Only then does the band check arm. Before that, the upright readings are
   inside the gate and would otherwise start the run early.
"""

import json
import logging
import math
import subprocess
import time
from pathlib import Path

logger = logging.getLogger(__name__)

UPRIGHT_DEG = 180.0

# The reduction below turns a pitch trace into the run's verdict, and it is the
# only place a run is judged. It stays importable without ROS so --self-test can
# exercise it anywhere; the node itself needs a sourced workspace.
try:
    import rclpy
    from controller_manager_msgs.srv import ListControllers
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float64

    ROS_AVAILABLE = True
except ImportError:  # --self-test on a machine without ROS
    ROS_AVAILABLE = False


def in_band(pitch_deg: float, gate_lower_deg: float, gate_upper_deg: float) -> bool:
    """The firmware's give-up band. One definition, used to arm and to judge."""
    return gate_lower_deg < pitch_deg < gate_upper_deg


def summarize(
    trace: list[tuple[float, float]],
    pwm: list[float],
    settling_window_s: float,
    setpoint_deg: float,
    impulse_time_s: float,
    gate_lower_deg: float,
    gate_upper_deg: float,
) -> dict:
    """Reduce one run's pitch trace to its metrics.

    `trace` is (seconds since arming, pitch in degrees), already rebased so that
    t=0 is the first in-band reading.

    Deviation is measured from `setpoint_deg`, not from upright. The controller
    is commanded to hold 180 + lean_deg, and lean is swept: measuring against 180
    would score a robot that perfectly holds its commanded lean as having 2
    degrees of hunt, and the hunt column would then correlate with |lean_deg|
    rather than with anything about the gains.
    """
    escaped_at = next(
        (t for t, d in trace if not in_band(d, gate_lower_deg, gate_upper_deg)), None
    )

    # Hunt is measured only after settling. Reproducing the firmware's wound-up
    # startup integrator is deliberate, but letting that transient into this
    # number would swamp the oscillation being measured.
    settled = [d for t, d in trace if t >= settling_window_s]
    if settled:
        rms = (sum((d - setpoint_deg) ** 2 for d in settled) / len(settled)) ** 0.5
        peak = max(abs(d - setpoint_deg) for d in settled)
    else:
        rms = float("nan")
        peak = float("nan")

    after = [(t, d) for t, d in trace if t >= impulse_time_s]
    recovery = float("nan")
    for i, (t, _) in enumerate(after):
        if all(abs(d - setpoint_deg) < 1.0 for _, d in after[i:]):
            recovery = t - impulse_time_s
            break

    return {
        "outcome": "pass" if escaped_at is None else "fail",
        "passed": escaped_at is None,
        "rms_pitch_error_deg": rms,
        "peak_pitch_deg": peak,
        "recovery_time_s": recovery,
        "mean_abs_pwm": (sum(pwm) / len(pwm)) if pwm else float("nan"),
        "escaped_gate_at_s": escaped_at,
        "samples": len(trace),
    }


class RunSupervisor(Node if ROS_AVAILABLE else object):
    def __init__(self) -> None:
        super().__init__("run_supervisor")

        defaults = {
            "world_name": "case_world",
            "model_name": "case",
            "controller_name": "balance_controller",
            "settling_window_s": 3.0,
            "impulse_time_s": 3.0,
            "impulse_magnitude_ns": 0.12,
            "physics_step_s": 0.001,
            "duration_s": 10.0,
            "initial_tilt_deg": 2.0,
            "spawn_z_m": 0.039,
            "gate_lower_deg": 150.0,
            "gate_upper_deg": 200.0,
            # Needed here, not only by the controller: metrics are measured as
            # deviation from the commanded setpoint, which is 180 + lean.
            "lean_deg": 0.0,
            "real_time_factor": 0.0,
            "activation_timeout_s": 60.0,
            "result_file": "/tmp/case_run.json",
            # Non-empty: write the full per-sample trace here as CSV. Off by
            # default because a sweep wants verdicts, not traces; indispensable
            # when a run fails for a reason no summary statistic can show.
            "trace_file": "",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._p = {name: self.get_parameter(name).value for name in defaults}

        self._pitch: list[tuple[float, float]] = []
        self._pwm: list[float] = []
        self._start_sim_s: float | None = None
        self._impulse_applied = False
        self._finished = False

        # The controller's sensor ring buffer starts zero-filled, so the first
        # `delay_samples` readings arrive as ~0 degrees. That is deliberate --
        # it reproduces the firmware's pre-packet `input = 0` and is what winds
        # the startup integrator. Those samples are far outside the gate, so
        # judging them would fail every run at t=0.01 s and look exactly like
        # "the gains do not work". The band check arms on the first in-band
        # reading, and the run clock starts there too.
        self._armed = False

        # Upright is inside the gate, so without this the run would arm during
        # controller activation -- before the robot has been tilted into its
        # initial condition, and with several seconds of standing counted as
        # part of the settling window.
        self._arming_enabled = False
        self._trace: list[tuple[float, float, float, float]] = []
        self._latest_pid_output = 0.0
        self._latest_pwm = 0.0
        self._next_progress_s = 0.0
        self._initial_pitch_deg = float("nan")

        # The controller publishes through a real-time publisher. Best-effort
        # here costs nothing and keeps the subscription from imposing delivery
        # guarantees on a real-time path.
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Float64, "/case/pitch", self._on_pitch, qos)
        self.create_subscription(Float64, "/case/pwm", self._on_pwm, qos)
        self.create_subscription(Float64, "/case/pid_output", self._on_pid_output, qos)

    # ---- startup ---------------------------------------------------------

    def wait_for_controller(self) -> bool:
        """Block until the balance controller reports active.

        Wall clock, deliberately: simulation time does not advance while the
        world is paused, so a simulation-time wait here would never return.
        """
        client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        deadline = time.monotonic() + self._p["activation_timeout_s"]
        wanted = self._p["controller_name"]

        while time.monotonic() < deadline:
            if not client.wait_for_service(timeout_sec=1.0):
                continue
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            response = future.result()
            if response is not None:
                for controller in response.controller:
                    if controller.name == wanted and controller.state == "active":
                        self.get_logger().info(f"{wanted} active")
                        return True
            time.sleep(0.25)

        self.get_logger().error(f"{wanted} did not become active")
        return False

    def start_run(self) -> None:
        """Tilt the robot, release the rate limit, then allow the run to arm.

        The tilt is applied here rather than at spawn because the robot has to
        survive controller activation, and a 2-degree lean does not: it falls in
        well under a second, while activation takes one or two. Spawning upright
        and tilting afterwards gives the run a clean, identical initial condition
        no matter how long activation took.
        """
        # Pause first. Each gz command below is a subprocess taking a good
        # fraction of a wall second, and physics is running at real time -- so
        # without this the robot is tilted and actively driven for a second or
        # more BEFORE the run arms, and the measurement starts mid-fall. A slow
        # fall survives that window and a fast one does not, which silently
        # makes the two incomparable. Pausing costs no simulation time, and is
        # safe now only because the controller is already active: a world paused
        # before activation can never activate one at all.
        self._set_paused(True)

        tilt_rad = math.radians(self._p["initial_tilt_deg"])
        tilt_ok = self._gz_request(
            f"/world/{self._p['world_name']}/set_pose",
            "gz.msgs.Pose",
            f'name: "{self._p["model_name"]}", '
            f"position: {{x: 0, y: 0, z: {self._p['spawn_z_m']}}}, "
            f"orientation: {{x: 0, y: {math.sin(tilt_rad / 2.0)}, z: 0, "
            f"w: {math.cos(tilt_rad / 2.0)}}}",
        )

        if not tilt_ok:
            # Without the tilt the run starts from a different initial condition
            # than the one recorded, so its metrics describe an experiment that
            # was never performed.
            self._set_paused(False)
            self._finish(outcome="error", detail="set_pose failed; initial tilt not applied")
            return

        # 0 means unlocked, which is what the sweep wants; the demo asks for 1.
        rtf = self._p["real_time_factor"]
        self._gz_request(
            f"/world/{self._p['world_name']}/set_physics",
            "gz.msgs.Physics",
            f"max_step_size: {self._p['physics_step_s']}, real_time_factor: {rtf}",
        )

        self._arming_enabled = True
        self._set_paused(False)
        self.create_timer(0.01, self._tick)

    # ---- run loop --------------------------------------------------------

    def _sim_now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_pitch(self, msg: Float64) -> None:
        if not self._armed:
            if not self._arming_enabled:
                return
            if not in_band(msg.data, self._p["gate_lower_deg"], self._p["gate_upper_deg"]):
                return
            self._armed = True
            self._start_sim_s = self._sim_now_s()
            # Recorded so a run that began from the wrong attitude is visible in
            # the data rather than silently averaged into the sweep.
            self._initial_pitch_deg = msg.data

        elapsed = self._sim_now_s() - self._start_sim_s
        self._pitch.append((elapsed, msg.data))
        if self._p["trace_file"]:
            self._trace.append((elapsed, msg.data, self._latest_pid_output, self._latest_pwm))

    def _on_pwm(self, msg: Float64) -> None:
        self._pwm.append(abs(msg.data))
        self._latest_pwm = msg.data

    def _on_pid_output(self, msg: Float64) -> None:
        self._latest_pid_output = msg.data

    def _tick(self) -> None:
        if self._finished or self._start_sim_s is None:
            return
        elapsed = self._sim_now_s() - self._start_sim_s

        # A headless run is several seconds of total silence between activation
        # and the verdict, which reads as a hang and invites an interrupt that
        # discards the run and its trace.
        if elapsed >= self._next_progress_s:
            self._next_progress_s = elapsed + 2.0
            self.get_logger().info(
                f"t={elapsed:5.1f}s  pitch={self._pitch[-1][1]:7.2f}  "
                f"pwm={self._latest_pwm:6.1f}  (180 = upright)"
            )

        if not self._impulse_applied and elapsed >= self._p["impulse_time_s"]:
            self._impulse_applied = True
            self._apply_impulse()

        if elapsed >= self._p["duration_s"]:
            self._finish()

    def _set_paused(self, paused: bool) -> bool:
        return self._gz_request(
            f"/world/{self._p['world_name']}/control",
            "gz.msgs.WorldControl",
            f"pause: {'true' if paused else 'false'}",
        )

    def _gz_request(self, service: str, reqtype: str, req: str) -> bool:
        """Call a gz SERVICE.

        World control, set_pose and set_physics are services, not topics.
        Publishing to them with `gz topic -p` is accepted and does nothing at
        all -- which is how every run up to 2026-09-16 started from an untilted
        robot while the CSV recorded the tilt it believed it had applied. A
        silent no-op is the worst possible failure for a harness, so this
        reports success and callers act on it.
        """
        try:
            done = subprocess.run(
                ["gz", "service", "-s", service,
                 "--reqtype", reqtype, "--reptype", "gz.msgs.Boolean",
                 "--timeout", "5000", "--req", req],
                check=False, capture_output=True, timeout=15.0, text=True,
            )
        except (subprocess.TimeoutExpired, OSError) as exc:
            logger.error("gz service %s failed: %s", service, exc)
            return False
        if done.returncode != 0 or "true" not in done.stdout.lower():
            logger.error(
                "gz service %s returned %s: %s %s",
                service, done.returncode, done.stdout.strip(), done.stderr.strip(),
            )
            return False
        return True

    def _apply_impulse(self) -> None:
        """Deliver the disturbance as a single-step wrench.

        /world/<w>/wrench applies for exactly one physics iteration, so a given
        impulse J needs force J / physics_step_s. That makes the delivered
        impulse exact and independent of how fast the simulation happens to be
        running -- which a force held for some wall-clock window would not be,
        and the sweep would then be comparing runs that got different pushes.

        Fore-aft, in the pitch plane: a two-wheel balancer is neutrally stable
        sideways, so a lateral push tests nothing the controller is responsible
        for.

        ponytail: shells out to `gz topic` rather than bridging EntityWrench
        through ros_gz. One subprocess call per run is not worth a bridge and
        its QoS matching. Upgrade if per-run overhead shows up in the benchmark.
        """
        force_n = self._p["impulse_magnitude_ns"] / self._p["physics_step_s"]
        self._gz_publish(
            f"/world/{self._p['world_name']}/wrench",
            "gz.msgs.EntityWrench",
            f'entity: {{name: "{self._p["model_name"]}", type: MODEL}}, '
            f"wrench: {{force: {{x: {force_n}, y: 0, z: 0}}, "
            f"torque: {{x: 0, y: 0, z: 0}}}}",
        )

    def _gz_publish(self, topic: str, msg_type: str, payload: str) -> None:
        try:
            subprocess.run(
                ["gz", "topic", "-t", topic, "-m", msg_type, "-p", payload],
                check=True, capture_output=True, timeout=10.0,
            )
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired, OSError) as exc:
            # A failed impulse makes the run meaningless rather than merely
            # noisy, so it is recorded as an error instead of being allowed to
            # masquerade as a pass.
            logger.error("gz command failed on %s: %s", topic, exc)
            self._finish(outcome="error", detail=f"gz topic {topic} failed: {exc}")

    def write_trace(self) -> None:
        """Flush the per-sample trace, if one was requested.

        Called on normal completion and again on interrupt, so a run stopped
        early still yields the data that explains what it was doing.
        """
        if not self._p["trace_file"] or not self._trace:
            return
        with open(self._p["trace_file"], "w") as handle:
            handle.write("t_s,pitch_deg,pid_output,pwm\n")
            for row in self._trace:
                handle.write("%.4f,%.6f,%.6f,%.6f\n" % row)
        self.get_logger().info(
            f"trace written to {self._p['trace_file']} ({len(self._trace)} samples)"
        )

    # ---- result ----------------------------------------------------------

    def _finish(self, outcome: str | None = None, detail: str = "") -> None:
        if self._finished:
            return
        self._finished = True

        if not self._armed:
            # No in-band reading ever arrived, so the robot was never under
            # control and there is nothing to judge. Recording this as a fall
            # would blame the gains for a startup fault.
            result = {"outcome": "error", "passed": False, "samples": 0,
                      "detail": detail or "no in-band pitch reading arrived"}
        else:
            result = summarize(
                self._pitch, self._pwm,
                settling_window_s=self._p["settling_window_s"],
                setpoint_deg=UPRIGHT_DEG + self._p["lean_deg"],
                impulse_time_s=self._p["impulse_time_s"],
                gate_lower_deg=self._p["gate_lower_deg"],
                gate_upper_deg=self._p["gate_upper_deg"],
            )
            result["detail"] = detail
            result["initial_pitch_deg"] = self._initial_pitch_deg
            if outcome is not None:
                result["outcome"] = outcome
                result["passed"] = outcome == "pass"

        self.write_trace()

        Path(self._p["result_file"]).write_text(json.dumps(result, indent=2))
        self.get_logger().info(f"run finished: {result['outcome']}")
        raise SystemExit(0)


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    rclpy.init()
    node = RunSupervisor()
    try:
        if not node.wait_for_controller():
            Path(node.get_parameter("result_file").value).write_text(
                json.dumps({"outcome": "error", "passed": False,
                            "detail": "controller never became active"}, indent=2)
            )
            return
        node.start_run()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        node.write_trace()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def self_test() -> None:
    """Check the reduction that decides every run's verdict.

    This is the only place a run is judged, so a defect here is invisible: it
    produces a plausible CSV full of wrong answers rather than an error.
    """
    gate = {"gate_lower_deg": 150.0, "gate_upper_deg": 200.0}

    # A trace that stays in band passes; one that leaves it fails at the moment
    # it left, not at the end of the run.
    steady = [(i * 0.01, 180.0) for i in range(1000)]
    result = summarize(steady, [10.0], 3.0, 180.0, 3.0, **gate)
    assert result["passed"] and result["outcome"] == "pass", result
    assert result["escaped_gate_at_s"] is None

    falling = steady[:500] + [(5.0 + i * 0.01, 180.0 + i) for i in range(50)]
    result = summarize(falling, [10.0], 3.0, 180.0, 3.0, **gate)
    assert not result["passed"] and result["outcome"] == "fail", result
    assert abs(result["escaped_gate_at_s"] - 5.20) < 1e-6, result["escaped_gate_at_s"]

    # The settling window must keep the startup transient out of the hunt
    # number. A huge early excursion with a quiet tail must read as quiet.
    transient = [(i * 0.01, 180.0 + (25.0 if i * 0.01 < 3.0 else 0.0)) for i in range(1000)]
    result = summarize(transient, [10.0], 3.0, 180.0, 3.0, **gate)
    assert result["rms_pitch_error_deg"] < 1e-9, result["rms_pitch_error_deg"]
    assert result["peak_pitch_deg"] < 1e-9, result["peak_pitch_deg"]

    # Deviation is measured from the commanded setpoint, not from upright.
    # A robot perfectly holding a 2-degree lean has zero hunt, not 2 degrees.
    leaning = [(i * 0.01, 182.0) for i in range(1000)]
    result = summarize(leaning, [10.0], 3.0, 182.0, 3.0, **gate)
    assert result["rms_pitch_error_deg"] < 1e-9, "lean must not be scored as hunt"
    against_upright = summarize(leaning, [10.0], 3.0, 180.0, 3.0, **gate)
    assert abs(against_upright["rms_pitch_error_deg"] - 2.0) < 1e-9, "guard is inert"

    # Recovery is the first time after the impulse from which the trace stays
    # settled -- not merely the first moment it touches the threshold.
    wobble = (
        [(i * 0.01, 180.0) for i in range(300)]
        + [(3.0 + i * 0.01, 185.0) for i in range(100)]
        + [(4.0 + i * 0.01, 180.0) for i in range(100)]
    )
    result = summarize(wobble, [10.0], 3.0, 180.0, 3.0, **gate)
    assert abs(result["recovery_time_s"] - 1.0) < 1e-6, result["recovery_time_s"]

    never = [(i * 0.01, 180.0 + 5.0 * (i % 2)) for i in range(1000)]
    assert summarize(never, [10.0], 3.0, 180.0, 3.0, **gate)["recovery_time_s"] != \
        summarize(never, [10.0], 3.0, 180.0, 3.0, **gate)["recovery_time_s"], \
        "never settling must be NaN, which is the only value unequal to itself"

    assert in_band(180.0, 150.0, 200.0)
    assert not in_band(150.0, 150.0, 200.0), "band is exclusive, matching the firmware"
    assert not in_band(0.0, 150.0, 200.0), "pre-arming zero readings are out of band"

    print("run_supervisor.py self-test: all checks passed")


if __name__ == "__main__":
    import sys

    if "--self-test" in sys.argv:
        self_test()
    else:
        main()
