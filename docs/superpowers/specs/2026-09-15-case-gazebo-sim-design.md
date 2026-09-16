# CASE Balance Simulation — Design

**Date:** 2026-09-15
**Status:** Approved, pending implementation plan
**Target platform:** The Construct rosject (web IDE), ROS 2 Jazzy + Gazebo Harmonic

## Purpose

Determine whether the PID gains in `case_voice/balance_control.cpp` (Kp=60, Ki=140,
Kd=0.9) keep CASE upright, and how much hunt they produce, before the robot is
built and without access to physical measurements.

The result must transfer to hardware. A simulation that reports stability the
bench does not reproduce is worse than no simulation, because it converts an open
question into a false answer.

## Why a sweep rather than a single run

No physical parameter of the robot has been measured. Mass, centre-of-mass height,
wheel radius, and motor constants are all estimates derived from the CAD shell
(`figs/bodyShell v5.png`) and the bill of materials.

A single simulation run with estimated parameters answers "do these gains balance
one guessed robot." The question that matters is "do these gains balance the robot
we are actually going to build," and the honest form of that question is a
robustness sweep: randomise every uncertain parameter across its plausible range,
run many trials, and report the fraction that stay upright.

This also matches the claim already recorded in the codebase. The comment at
`case_voice/balance_control.cpp:39` states that "simulation shows the low value
stabilises far fewer plausible robots" — the language of a sweep. That simulation
was never committed and is lost. This design reconstructs it and puts it in the
repository.

## Platform decision

ROS 2 Jazzy with Gazebo Harmonic.

- Jazzy and Harmonic are released together from `packages.ros.org`, so no
  third-party package sources are needed.
- The Construct offers Jazzy rosjects and publishes a Gazebo Sim course against it.
- `gz_ros2_control` exposes `<command_interface name="effort">`, applying requested
  torque directly to a joint. Torque is the only interface that permits an honest
  motor model; a velocity or position interface would hide the actuator behind
  Gazebo's own internal controller.

Rejected: Humble + Harmonic (requires unofficial packages), Humble + Fortress
(older physics, and `gazebo_ros2_control` rather than `gz_ros2_control`).

## 1. Plant model

### Structure

Three links, two joints:

- `base_link` — chassis, modelled as a single box, centre of mass above the wheel axle
- `left_wheel`, `right_wheel` — cylinders on continuous joints sharing a Y axis
- `imu_link` — fixed frame at the MPU6050 mounting position on `base_link`

No caster and no third contact point. The robot falls when control fails, which is
the behaviour under test.

The IMU frame is deliberately not coincident with the centre of mass. The physical
sensor sits where the board is mounted, and that offset contributes centripetal
acceleration to its readings during fast rotation.

### Single-box chassis

The chassis is one box rather than a decomposition into shell, battery, and boards.
Splitting it would require guessing three masses and three positions instead of one,
which adds parameters without adding information. The centre-of-mass height sweep
already expresses the uncertainty that a split would represent. When the built robot
is measured, the box takes the measured values and the sweep narrows.

### Parameterisation

Every physical quantity is a xacro argument. No dimension or mass is written as a
literal in the URDF. Inertia tensors are computed by xacro macros from the box and
cylinder dimensions, so that changing a mass cannot leave a stale inertia behind.

| Parameter | Nominal | Sweep range | Source of uncertainty |
|---|---|---|---|
| Total mass | 0.85 kg | 0.6 – 1.2 | Pi 3B+, ESP32, 2S LiPo, two gearmotors, PLA shell; shell infill and battery size dominate |
| COM height above axle | 70 mm | 45 – 95 | Battery placement in the shell; the single most influential parameter |
| Wheel radius | 34 mm | 30 – 40 | Estimated from the render's wheel-to-body proportion |
| Wheel separation | 105 mm | 90 – 125 | Set by shell width |
| Body depth | 55 mm | 45 – 70 | Contributes to pitch inertia; second-order relative to COM height |
| Wheel/ground friction | 1.0 | 0.7 – 1.3 | Rubber tyre on an unspecified demo surface |

Note the direction of difficulty: a **higher** centre of mass is easier to balance,
because the robot falls more slowly and the controller has more time to react. The
45 mm end of that range is the hard case.

### Recommended measurements

Two measurements, available as soon as the chassis is assembled, collapse the two
widest ranges: total mass on a kitchen scale, and centre-of-mass height by balancing
the chassis on a pencil. Neither blocks this design. Both would turn a broad sweep
into a narrow one.

## 2. Controller and actuator

### Placement

A custom `controller_interface::ControllerInterface` plugin, loaded by
`gz_ros2_control`, claiming an effort command interface on both wheel joints. It
executes synchronously within the simulation update loop at a fixed 100 Hz on
simulation time.

Rejected alternatives:

- A ROS node subscribing to an IMU topic. The asynchronous round trip introduces
  timestep jitter, and Ki=140 is strongly timestep-sensitive, so jitter alone would
  change what the gains mean.
- Gazebo's built-in `JointController` in force mode, or `ros2_controllers`'
  `pid_controller`. Neither reproduces PID_v1's semantics, and neither models the
  PWM deadzone.

### PID_v1 ported verbatim

The Arduino PID_v1 library pre-scales gains by the sample time. With
`SetSampleTime(10)` and the tunings in `balance_control.cpp`:

```
kp = 60                    // used as written
ki = 140 x 0.010 = 1.4     // Ki x sample_time_seconds
kd = 0.9  / 0.010 = 90     // Kd / sample_time_seconds
```

Three further semantics are reproduced exactly:

- **Derivative on measurement.** The derivative term is `-kd * (input - lastInput)`,
  with no further division by dt — the division is already folded into `kd` above.
- **Integral clamped before the proportional term is added.** The accumulator is
  bounded to the output limits at the point of accumulation, not after summing. This
  is the library's anti-windup, and it governs recovery behaviour after a large tilt.
- **Proportional on error** (`P_ON_E`, the library default).
- **The rate gate is integer milliseconds, not floating-point seconds.** Upstream
  gates on `millis() - lastTime >= SampleTime` in `unsigned long` milliseconds.
  Transcribing that as `now_s - last_time_s >= 0.010` on doubles silently skips
  roughly 44% of cycles: `0.03 - 0.02` evaluates to `0.009999999999999998`, a few
  units in the last place short of the threshold. Measured: 2187 of 5000 cycles
  skipped, and 2189 when the time source is `rclcpp::Time::seconds()`, which is
  exactly what the controller passes. Since Ki=140 is strongly timestep-sensitive,
  this would have invalidated every sweep result while appearing to work. The port
  keeps the period in integer milliseconds and derives the gain scaling from it,
  so there is still one source of truth for the period.

A textbook PID implementation would make Kp=60 mean something different from what it
means in the firmware, which would invalidate every result.

### Initialization is part of the port

The firmware's startup sequence produces a transient that the simulation must
reproduce, because "verbatim" has to include the first second.

`BalanceControl::begin()` calls `SetMode(AUTOMATIC)` before any DMP packet has
arrived, so `Initialize()` captures `lastInput = input = 0`. The PID_v1 constructor
sets `lastTime = millis() - SampleTime`, which means the first `Compute()` executes
immediately rather than after one sample period. With `input` still 0 and
`setpoint = 182`, the error is 182 and the integrator accumulates
`ki x 182 = 254.8` per call, pinning `outputSum` at its +255 clamp within two calls
— all before the first real orientation reading.

When the first DMP packet does arrive at roughly 180, `dInput` is about 180 and the
derivative term is roughly `-90 x 180 = -16200`, clamping the output to −255 for a
single cycle.

The `150 < pitch < 200` gate coasts the motors throughout this window, so the kick
never reaches the wheels. The wound integrator does survive it: balancing begins with
`outputSum` pinned at +255, which at a 2-degree error unwinds at 2.8 per 10 ms —
roughly 0.9 seconds of bias working itself out while the robot is trying to stand up.

The simulation reproduces this exactly: `Initialize()` semantics, the immediate first
`Compute()`, and the pre-packet accumulation. The run protocol's settling period
exists partly to let this transient pass before the measurement window opens.

This is a probable firmware defect — initializing the PID after the first DMP packet
would remove it. Fixing it is out of scope here, in the same way the "Brake mode"
comment is.

### Actuator chain

The chain below is what `case_voice/balance_control.cpp` implements in
`BALANCE_ONLY` mode, and what the simulation reproduces:

```
pitch = ypr[1] * 180/pi + 180        // 180 = upright
  |
  v  PID_v1 at 10 ms, output clamped to +/- 255
output
  |
  v  gate: 150 < pitch < 200, otherwise coast
  v  abs(output) / 2
  v  skipDeadzone: 0 -> 0; otherwise 20 + pwm * 235/255
pwm -> TB6612 -> motor
```

Four properties of this chain were found by reading the firmware and materially
affect the result:

1. **The output is halved** (`balance_control.cpp:170`). The PID computes to ±255
   but no more than ±127 reaches the deadzone remap. The effective proportional
   gain at the wheels is 30, not 60. (The remap then lifts that to at most 137 —
   see the next point. 127 is the pre-remap figure.)

2. **The deadzone remap has a ceiling.** `skipDeadzone` maps its input across
   [20, 255], but its input has already been halved, so the maximum commanded PWM is
   137 — roughly 54% duty. The robot cannot request full torque under any
   circumstance. This bounds the recoverable tilt angle.

3. **The step from 0 to 20 is a discontinuity, and integer arithmetic places it.**
   `int basePWM = output` truncates the double toward zero, and `abs(basePWM) / 2` is
   integer division. So an output magnitude below 2 produces PWM 0, and an output
   magnitude of 2 or more produces PWM 20 immediately — there is nothing between.
   `skipDeadzone` is integer arithmetic too: `20 + (pwm * 235) / 255`.

   This step is a limit-cycle generator and is the mechanism behind the hunt
   described in the comment at `balance_control.cpp:18-26`. The simulation must use
   the firmware's integer operations verbatim; a floating-point port of these lines
   moves the discontinuity and would report a stability the hardware will not show.

4. **`stop()` coasts; it does not brake — but the in-gate path does brake.** On
   the TB6612, IN1=LOW with IN2=LOW is stop/high-impedance. Short brake is
   IN1=HIGH with IN2=HIGH. The comment at `self_balance/self_balance.ino:262`
   describes `stop()` as "Brake mode," which is incorrect. The simulation models
   coast there, matching the code; the misleading comment is a separate fix and
   out of scope.

   There is a second regime the original design missed. Inside the gate with
   `|output| < 2`, the firmware still calls `forward(0)` or `reverse(0)`: the
   direction pins are driven and PWM is 0, which on the TB6612 is a short brake,
   not a coast. So the robot brakes while hunting near upright and coasts only
   once it has given up. The motor model gets this right without a special case —
   `torque_from_pwm(0, omega, p)` evaluates to `-Kt*Kv*omega/R`, the braking
   torque — and `is_coasting()` covers only the true `stop()` path.

### Motor model

A `MotorModel` struct at the controller's output stage, kept separate from the PID:

```
tau = Kt * (V_batt * pwm/255 - Kv * omega) / R
```

The back-EMF term is not optional. A falling robot spins its wheels quickly, and a
model without `Kv * omega` promises torque the motor cannot deliver at speed —
precisely the regime in which balance is won or lost.

Motor constants are unknown and therefore swept across the range typical of small
hobby gearmotors. A `ponytail:` comment marks the linear back-EMF model as a known
simplification, with a measured torque curve as the upgrade path.

### Feedback polarity

`motor_direction_sign` (+1 or −1) multiplies the final commanded effort.

The sign of the loop depends on two decisions made independently: how the IMU
quaternion is converted to pitch, and which way the URDF points the wheel joint
axes. If they disagree, the controller drives the robot over faster than gravity
alone would — and the failure looks exactly like "these gains do not work" rather
than like a sign error, which would misattribute a wiring mistake to the quantity
under test.

This is a calibration knob, not a workaround. Physical hardware has the same
ambiguity, resolved on the bench by swapping motor leads. First run on the
rosject: if the robot falls faster with the controller active than with it
coasting, flip this.

### Firmware variant

The simulation models the `case_voice/balance_control.cpp` path: Kp=60, deadzone
compensation enabled, motor modes present.

This carries a risk worth stating plainly. The only hardware-validated data point
available is that Kp=15 balanced the physical robot — and that came from
`self_balance/self_balance.ino`, which has **no** deadzone compensation. In the
chosen configuration, nothing anchors the simulation to observed reality.

Mitigation: `deadzone_pwm` is already a parameter. Setting it to 0 reproduces the
`self_balance.ino` actuator path exactly. The anchor remains available as a
configuration value, not a separate code path.

## 3. Sensor model

### Ground truth, degraded

Gazebo's IMU sensor provides raw accelerometer and gyroscope data, which would then
require fusion. Any filter written here is not the InvenSense DMP, whose fusion is
proprietary and whose behaviour is what the gains were tuned against. Simulating raw
data plus a substitute filter would introduce unquantifiable error while appearing
more rigorous than the alternative.

Instead the controller reads ground-truth pitch and degrades it the way the DMP path
degrades it. What the firmware actually receives is a fused angle, delayed and
slightly noisy, at 100 Hz.

### How the controller obtains pitch

Chassis pitch is the free orientation of `base_link` in the world. It is not any
joint's position, so the joint state interfaces `gz_ros2_control` exports cannot
supply it. The mechanism is an IMU sensor declared inside the `<ros2_control>` tag
on `imu_link`, which `GazeboSimSystem` exposes to the controller as state interfaces
named `orientation.x/y/z/w`, `angular_velocity.*`, and `linear_acceleration.*`.

**This is not in the `gz_ros2_control` documentation.** The Jazzy documentation page
describes only force-torque sensors. IMU support is present in the implementation —
`gz_system.cpp` on the `jazzy` branch matches on `sim::components::Imu` and registers
exactly those interface names in `registerSensors()`. Verified against source on
2026-09-15. This is recorded here so that implementation is not abandoned on the
strength of a documentation page that appears to say the feature is absent.

Gazebo derives the IMU sensor's `orientation` from the link's world pose rather than
by fusing noisy accelerometer and gyroscope data, so these interfaces carry ground
truth — which is exactly what this design wants, since the degradation is applied
deliberately and with known parameters rather than inherited from a substitute
filter.

The IMU sensor's `update_rate` in the URDF must be at least 100 Hz, or the sensor
becomes an additional and unintended source of latency on top of the modelled delay.

| Effect | Model | Sweep range | Justification |
|---|---|---|---|
| Transport delay | N-sample ring buffer | 1 – 3 samples (10 – 30 ms) | DMP fusion latency, I2C FIFO read, and the drain loop in `update()` that keeps only the newest packet |
| Angle noise | Gaussian, sigma | 0.05 – 0.3 degrees | DMP output is already filtered; noise is small but non-zero |
| Rotation artifact | noise scaled by abs(angular acceleration) | on/off | The IMU sits off the COM, so fast rotation contaminates the gravity vector the DMP fuses against |
| Bias drift | not modelled | — | Sub-degree over a 10 s run; irrelevant at this timescale |

**Transport delay is the most important parameter in this table.** It is what Kd=0.9
is fighting: derivative gain applied to a delayed signal is destabilising, and the
amount of delay determines how much derivative gain is affordable. Fixing it at a
guessed value would bias every result, which is why it is swept.

The rotation artifact carries a `ponytail:` comment naming its ceiling — it is a
crude stand-in for a real accelerometer-contamination model, upgradeable if the
sweep proves sensitive to it.

### Angle convention

The controller consumes pitch in the firmware's units: degrees plus 180, with
upright at 180.

`setpoint = 182` in the firmware encodes a 2-degree forward lean compensating
centre-of-mass misalignment on the physical robot. A symmetric URDF has no such
misalignment, so in simulation 182 would mean "deliberately lean forward 2 degrees."

The lean is therefore a separate parameter defaulting to 0, with
`setpoint = 180 + lean`. This lets the sweep determine whether the 2 degrees
compensated something physical or was a tuning artifact.

## 4. Sweep harness

### Benchmark before sizing

The first implementation step is a single headless run with the real-time factor
measured on the actual rosject container. The sweep size is derived from that
measurement. A two-link robot at a 1 ms physics step should run well above real
time, but Construct containers are shared and the run count is not promised in
advance.

If the measured real-time factor makes a meaningful sweep impossible, the fallback
is a Python ODE model for the statistics with Gazebo retained for fidelity
confirmation. That fallback is not designed here and would be a separate decision.

### Pass criterion

`applyMotorControl` coasts the motors whenever pitch leaves the interval
(150, 200) — that is, −30 to +20 degrees from upright. Once outside that band the
robot is freewheeling and cannot recover.

> **Pass** = pitch remains within (150, 200) for the full 10-second run.

The criterion is the controller's own give-up band rather than an invented
threshold, so a pass in simulation means what a pass means on the bench.

### Startup order

Nothing is measured until the control loop is live, and reaching that state takes
three steps that cannot be reordered:

1. The robot spawns **upright**, with physics running at real time. Upright is an
   equilibrium, so it keeps standing through the second or two the spawner needs
   to activate the controller.
2. Once `/controller_manager/list_controllers` reports `balance_controller`
   active, the supervisor tilts the robot into its real initial condition via
   `set_pose` and releases the real-time factor to the run's setting.
3. Only then does the band check arm. Upright readings are inside the gate, so
   arming earlier would start the run during activation and count several
   seconds of standing as part of the settling window.

**The world cannot start paused**, which was this design's first answer.
`gz_ros2_control` gates `controller_manager->update()` on
`sim_period >= control_period`, and `sim_period` is zero while paused — so a
paused world can never activate a controller at all, and every run would have
failed at the activation timeout. Verified against `gz_ros2_control_plugin.cpp`
on the `jazzy` branch.

**The tilt cannot be applied at spawn** either, which was the second answer. A
2-degree lean falls in well under a second while activation takes one or two.
Spawning upright and tilting afterwards gives every run an identical initial
condition regardless of how long activation took.

The activation wait runs on wall clock rather than simulation time, since the
amount of simulation time that elapses during activation is not fixed.

Gazebo does not publish `/clock` on its own. A `ros_gz_bridge parameter_bridge`
supplies it; without that bridge every node with `use_sim_time` stays frozen at
t=0 and the supervisor's timer never fires.

**Open question for the first benchmark run:** the supervisor's timeline (impulse
at t=3 s, finish at t=10 s) rides on a simulation-time timer, which fires only
when a `/clock` message arrives. If Gazebo throttles `/clock` below the physics
rate, the impulse lands at the first tick at or after 3 s, which at an unlocked
real-time factor could be measurably late and differently late per run. Check
`ros2 topic hz /clock` during the benchmark; if it is throttled, drive the
timeline from the pitch callback, which arrives at a deterministic 100 samples
per simulated second.

Gazebo does not publish `/clock` on its own. A `ros_gz_bridge parameter_bridge`
supplies it; without that bridge every node with `use_sim_time` stays frozen at
t=0 and the supervisor's timer never fires.

### Run protocol

Spawn at 2 degrees of forward tilt. Allow 3 seconds of settling — long enough for
the startup integrator transient described in section 2 to unwind. Apply a fore-aft
disturbance impulse at t = 3 s, then continue to t = 10 s.

The disturbance is **fore-aft, in the pitch plane**, not lateral. A two-wheel
balancer is neutrally stable sideways; a lateral push tests nothing the controller
is responsible for.

It is delivered as a **single-step wrench**: Gazebo's `/world/<w>/wrench` topic
applies a force for exactly one physics iteration, so an impulse of J needs a
force of `J / physics_step_s`. Holding a force over a wall-clock window instead
would make the delivered impulse depend on how fast the simulation happened to be
running, and the sweep would be comparing runs that got different pushes while
recording them as identical. This is why `physics_step_s` appears in
`sweep_ranges.yaml` as part of the experiment's definition rather than only as a
solver setting in the world file.

| Quantity | Value |
|---|---|
| Initial tilt | 2 degrees forward |
| Settling window | 0 – 3 s (excluded from hunt measurement) |
| Impulse time | t = 3 s |
| Impulse magnitude | swept, 0.05 – 0.25 N·s, applied at COM height |
| Run duration | 10 s |

The impulse magnitude is swept rather than fixed because "how hard a push survives"
is itself a result worth reporting, and because a single fixed value would either sit
below every configuration's threshold or above it.

Recorded per run:

- pass or fail
- RMS pitch error after settling — the hunt amplitude
- peak pitch
- recovery time after the impulse
- mean absolute PWM

Hunt amplitude matters as much as pass/fail. Gains that technically balance while
oscillating at several degrees are not usable gains.

### Sampling

Latin hypercube sampling over the parameter ranges, rather than independent uniform
draws. With roughly a dozen swept parameters and a run count likely in the low hundreds,
Latin hypercube gives substantially better coverage for the same number of runs —
and if the benchmark comes back slow, coverage at low N is what preserves the
experiment.

### Determinism

Fixed random seed, fixed 1 ms physics step, all timing on simulation time with no
wall-clock dependence. The same seed must reproduce the same results, or the sweep
is not evidence.

### Process model

A fresh Gazebo process per run, accepting roughly 3–5 s of startup overhead.
In-process world reset is faster but leaks controller and physics state between
runs in ways that silently corrupt results. A `ponytail:` comment names the overhead
with in-process reset as the upgrade path if the benchmark shows it is needed.

Serial execution by default, with a `--jobs` flag, since the container's core count
is unknown.

## 5. Package layout and visualisation

The package lives in this repository at `ros2/case_sim/` and is cloned into the
rosject's `~/ros2_ws/src`. Keeping it in the repository is the direct remedy for the
failure this design replaces: the simulation behind the `Kp=60` comment was never
committed and no longer exists.

```
ros2/case_sim/
  urdf/case.urdf.xacro          parameterised; inertias from macros
  worlds/case_world.sdf         1 ms step, Imu system, ApplyLinkWrench
  config/controllers.yaml       ros2_control, update_rate 100
  config/sweep_ranges.yaml      every range from sections 1-3
  config/case.rviz
  src/pid_v1.hpp                verbatim port
  src/motor_model.hpp           PWM -> deadzone -> back-EMF -> torque
  src/balance_controller.cpp    controller_interface plugin
  case_sim_plugins.xml          pluginlib export
  launch/case_sim.launch.py     one launch file; gui:=true/false
  scripts/run_supervisor.py     owns one run's timeline and metrics
  scripts/sweep.py              LHS sampling, run driver, CSV output
  scripts/analyze.py            stabilised fraction, hunt distribution
```

**One launch file, not two.** The design originally called for separate
`demo.launch.py` and `sweep.launch.py`. Thin wrappers delegating to a shared
implementation do not work: `IncludeLaunchDescription` forwards only the
arguments it is explicitly given, so per-run sweep parameters would have fallen
back to their nominal values silently — every run measuring the same robot while
the CSV recorded the parameters it believed it had set. A single
`case_sim.launch.py` with a `gui` argument avoids inventing that failure for the
sake of two file names.

**`run_supervisor.py` was missing from the original design.** Nothing owned one
run's timeline: applying the impulse at the scheduled simulation time, ending the
run at 10 s, and emitting the metrics. It cannot live in `sweep.py`, which is
outside the simulation and cannot act at a precise simulation time, and it does
not belong in the controller, which has one job. It is its own node.

`config/sweep_ranges.yaml` is the single authoritative source for every physical
range. Both the sweep script and the demo launch read it, so nominal values cannot
drift apart between the two entry points.

### What RViz does and does not provide

RViz2 displays the robot model, TF frames, and a torque arrow marker. It does not
plot time series. The pitch, PID output, and PWM traces that actually demonstrate
whether the gains work are viewed in PlotJuggler or read from the CSV. This is
recorded explicitly so that RViz is not mistaken for a plotting tool during the
capstone presentation.

The controller publishes `/case/pitch`, `/case/pid_output`, and `/case/pwm` via
`realtime_tools::RealtimePublisher`. A plain publisher inside a real-time controller
update can block, which would corrupt the timing this design exists to protect.

## 6. Testing

### Licensing

PID_v1 is GPLv3 (Brett Beauregard, v1.1.1), not MIT. This repository is already
GPLv3, so vendoring the library into `test/vendor/` for the equivalence test is
compatible, and the upstream header is kept byte-identical. `package.xml`
declares `GPL-3.0-only` to match.

### Port equivalence — the decisive check

`PID_v1.cpp` is plain C++ whose only platform dependency is `millis()`. The test
compiles the real library against a stubbed clock alongside the port, drives both
with an identical pseudo-random input sequence for several thousand steps, and
asserts the outputs match within floating-point tolerance.

Any drift in the port — the gain pre-scaling, the clamp ordering, derivative on
measurement — fails this test. Every result the sweep produces rests on the port
being exact.

### Harness checks

- A smoke test running three sweep iterations, asserting the CSV shape and column set.
- An assertion that a deliberately absurd gain set fails, so that a harness bug
  cannot report universal success.

### Validation experiment

Rerun the sweep at Kp=15 and at Kp=60. The comment at
`case_voice/balance_control.cpp:39` claims Kp=60 stabilises more plausible robots
than Kp=15. If the sweep reproduces that ordering, the lost result has been
recovered and the simulation earns some confidence. If it contradicts the ordering,
either the simulation or that comment is wrong, and determining which is more
valuable than any single figure the sweep produces.

## Results log

### 2026-09-16: harness brought up, first sweep, NOT yet validated

The stack runs end to end on The Construct: 39 of 40 runs complete, roughly 7 s
each for a failure, 0.58x real-time factor including startup.

**First paired sweep, 19 identical robots under both gain sets:**

| configuration | stabilised |
|---|---|
| Kp=60, deadzone_pwm=20 | 0 / 19 |
| Kp=15, deadzone_pwm=20 | 0 / 20 |

**This does not yet say anything about the gains, because the simulation fails
its own validation test.** The robot balanced on hardware at Kp=15, and a
simulation that returns 0% for a configuration known to work is not a valid
instrument. Note also that both sweeps used `deadzone_pwm: 20`, from
`case_voice`; the hardware-validated firmware is `self_balance.ino`, which has
no deadzone compensation. The anchor configuration this design specified
(`deadzone_pwm: 0`) had not been run.

**Findings that do not depend on the sweep**, established from single-run traces:

- Feedback sign confirmed as +1, by running both signs on identical seeds: at
  +1 pitch oscillates within 1.7 degrees of upright over the first 9 ms; at -1
  it diverges monotonically to 157 degrees.
- Kp=60 saturates the +/-255 clamp at 4.25 degrees of error, so the loop has
  almost no proportional region and behaves as bang-bang torque. Kp=15
  saturates at 17 degrees.
- The deadzone remap makes the smallest non-zero correction PWM 20, which is
  0.54 m/s^2 of base acceleration -- a large minimum kick for a robot 2 degrees
  off vertical.
- The firmware's startup integrator wind-up is fatal to a free-standing robot:
  it commands full torque into a balanced robot before the first sensor
  reading. Hardware survives it only because someone is holding the robot.

**Harness defects found and fixed along the way**, each of which produced
plausible-looking but meaningless results: motor constants estimated for an
ungeared motor (10x low); world commands sent to gz topics instead of services,
so the initial tilt was never applied; measurement armed mid-fall; a rotation
artifact model built on doubly-differentiated pitch that injected hundreds of
degrees of noise; and post-failure tumbling that aborted the ODE collision
solver on 40% of runs.

### 2026-09-16 later: reproducibility achieved, first meaningful failures

Three identical invocations now produce identical verdicts (fail at 4.020 /
4.01 / 4.01 s) from a correctly applied initial condition (initial pitch 178.04
/ 177.89 / 178.09, i.e. the 2 degree tilt plus sensor noise). Before this, the
same parameters gave a pass in one run and a millisecond failure in another.

The cause was a window nobody owned. The controller must be active before
physics can be paused, so the robot was driven while standing upright for a
variable amount of wall time, and `set_pose` resets pose but not velocity --
`gz.msgs.Pose` has no velocity field -- so each run began with different hidden
momentum. The fix is a hold: the controller commands zero effort until pitch
leaves a 1 degree band, so the robot rests at its upright equilibrium until the
teleport moves it. That is harness scaffolding and is documented as such;
`reproduce_startup_windup` opts out, since drive-from-power-on is what that
experiment measures.

**What it revealed:** the robot now settles from its lean, holds for three
seconds, takes the impulse at t=3 s, and falls about a second later. The failure
mode is disturbance rejection, not an inability to stand. Every sweep result
recorded before this point measured a startup artifact and should be discarded.

## Assumptions

All physical parameters in this document are estimates, not measurements. Their
sources are:

- **Mass, dimensions, wheel proportions** — inferred from `figs/bodyShell v5.png`
  and `figs/case_concept_render.png` together with the bill of materials in
  `QUICKSTART.md`.
- **Motor constants** — unknown; swept across the range typical of small hobby
  gearmotors.
- **Sensor latency and noise** — inferred from MPU6050 DMP documented behaviour and
  the FIFO drain logic in `balance_control.cpp:142-151`.

Each should be replaced with a measurement when one becomes available. The design
is structured so that replacing an estimate is a change to
`config/sweep_ranges.yaml`, never a change to code.

## Risks

| Risk | Impact | Mitigation |
|---|---|---|
| Construct real-time factor too low for a useful sweep | The central experiment cannot run | Benchmark first; Python ODE fallback exists but is undesigned |
| No hardware anchor in the chosen firmware configuration | Sim cannot be validated against observed reality | `deadzone_pwm: 0` reproduces the hardware-validated actuator path |
| Motor constants swept over a wide range | Weakens the strength of every individual result | Bench measurement of no-load RPM and stall PWM would narrow it substantially |
| Estimated inertia is wrong in a way the sweep range does not cover | Sweep reports confident but wrong results | Measure mass and COM height once the chassis exists |

## Out of scope

- Voice command and navigation layers. The `MotorMode` variants beyond
  `BALANCE_ONLY` are not simulated.
- The Raspberry Pi hybrid architecture described in `docs/ARCHITECTURE.md`.
- Correcting the "Brake mode" comment at `self_balance/self_balance.ino:262`.
- Any change to firmware. This design consumes the firmware's behaviour; it does not
  modify it.
