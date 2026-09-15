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
the launch file. The world starts PAUSED. Spawning the robot and activating the
controller takes a second or two of wall clock, and the sweep runs with an
unlocked real-time factor -- so if physics were already running, that second
would be tens of simulation seconds and every robot would be flat on the floor
before its controller ever ran. Nothing is measured until the loop is live.
"""

import json
import logging
import subprocess
import time
from pathlib import Path

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64

logger = logging.getLogger(__name__)

UPRIGHT_DEG = 180.0


class RunSupervisor(Node):
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
            "gate_lower_deg": 150.0,
            "gate_upper_deg": 200.0,
            "real_time_factor": 0.0,
            "activation_timeout_s": 60.0,
            "result_file": "/tmp/case_run.json",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._p = {name: self.get_parameter(name).value for name in defaults}

        self._pitch: list[tuple[float, float]] = []
        self._pwm: list[float] = []
        self._start_sim_s: float | None = None
        self._impulse_applied = False
        self._escaped_gate_at: float | None = None
        self._finished = False

        # The controller publishes through a real-time publisher. Best-effort
        # here costs nothing and keeps the subscription from imposing delivery
        # guarantees on a real-time path.
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Float64, "/case/pitch", self._on_pitch, qos)
        self.create_subscription(Float64, "/case/pwm", self._on_pwm, qos)

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

    def start_clock(self) -> None:
        """Unpause physics, and pin the rate when a demo asked for real time."""
        rtf = self._p["real_time_factor"]
        if rtf > 0:
            self._gz_publish(
                f"/world/{self._p['world_name']}/set_physics",
                "gz.msgs.Physics",
                f"max_step_size: {self._p['physics_step_s']}, real_time_factor: {rtf}",
            )
        self._gz_publish(
            f"/world/{self._p['world_name']}/control",
            "gz.msgs.WorldControl",
            "pause: false",
        )
        self.create_timer(0.01, self._tick)

    # ---- run loop --------------------------------------------------------

    def _sim_now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_pitch(self, msg: Float64) -> None:
        now = self._sim_now_s()
        if self._start_sim_s is None:
            self._start_sim_s = now
        elapsed = now - self._start_sim_s
        self._pitch.append((elapsed, msg.data))

        if self._escaped_gate_at is None and not (
            self._p["gate_lower_deg"] < msg.data < self._p["gate_upper_deg"]
        ):
            self._escaped_gate_at = elapsed

    def _on_pwm(self, msg: Float64) -> None:
        self._pwm.append(abs(msg.data))

    def _tick(self) -> None:
        if self._finished or self._start_sim_s is None:
            return
        elapsed = self._sim_now_s() - self._start_sim_s

        if not self._impulse_applied and elapsed >= self._p["impulse_time_s"]:
            self._impulse_applied = True
            self._apply_impulse()

        if elapsed >= self._p["duration_s"]:
            self._finish()

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

    # ---- result ----------------------------------------------------------

    def _finish(self, outcome: str | None = None, detail: str = "") -> None:
        if self._finished:
            return
        self._finished = True

        if outcome is None:
            outcome = "pass" if self._escaped_gate_at is None else "fail"

        # Hunt amplitude is measured only after settling. Reproducing the
        # firmware's wound-up startup integrator is deliberate, but letting that
        # transient into this number would swamp the oscillation being measured.
        settled = [d for t, d in self._pitch if t >= self._p["settling_window_s"]]
        if settled:
            rms = (sum((d - UPRIGHT_DEG) ** 2 for d in settled) / len(settled)) ** 0.5
            peak = max(abs(d - UPRIGHT_DEG) for d in settled)
        else:
            rms = float("nan")
            peak = float("nan")

        result = {
            "outcome": outcome,
            "passed": outcome == "pass",
            "detail": detail,
            "rms_pitch_error_deg": rms,
            "peak_pitch_deg": peak,
            "recovery_time_s": self._recovery_time(),
            "mean_abs_pwm": (sum(self._pwm) / len(self._pwm)) if self._pwm else float("nan"),
            "escaped_gate_at_s": self._escaped_gate_at,
            "samples": len(self._pitch),
        }

        Path(self._p["result_file"]).write_text(json.dumps(result, indent=2))
        self.get_logger().info(f"run finished: {outcome}")
        raise SystemExit(0)

    def _recovery_time(self) -> float:
        """Seconds after the impulse until pitch stays within 1 degree.

        NaN when it never settles, which is not the same as a fail: a robot can
        stay inside the gate while still hunting, and keeping those two apart is
        the point of measuring hunt separately from pass/fail.
        """
        impulse_t = self._p["impulse_time_s"]
        after = [(t, d) for t, d in self._pitch if t >= impulse_t]
        for i, (t, _) in enumerate(after):
            if all(abs(d - UPRIGHT_DEG) < 1.0 for _, d in after[i:]):
                return t - impulse_t
        return float("nan")


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
        node.start_clock()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
