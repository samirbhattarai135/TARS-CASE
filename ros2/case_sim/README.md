# case_sim

Gazebo simulation of the CASE two-wheel self-balancing robot, built to answer one
question: **do the firmware's PID gains keep this robot upright?**

No physical parameter of the robot has been measured, so a single run would only
tell you about one guessed robot. This package instead sweeps every uncertain
parameter across its plausible range and reports the fraction of plausible CASE
robots the gains stabilise.

Design and rationale: `docs/superpowers/specs/2026-09-15-case-gazebo-sim-design.md`.

## Running it on The Construct

Create a **ROS 2 Jazzy** rosject, then in its web shell:

```bash
cd ~/ros2_ws/src
# -b matters: this package is not on main.
git clone -b feat/workflow https://github.com/samirbhattarai135/TARS-CASE.git
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src   # only if the build reports missing deps
colcon build --packages-select case_sim
source install/setup.bash
```

colcon recurses from `src/` and finds `TARS-CASE/ros2/case_sim/package.xml`, so
the repository can be cloned whole; no symlink or file moving is needed.

### 1. Benchmark first — always

```bash
ros2 run case_sim sweep.py --benchmark
```

This runs one headless simulation and reports the achieved real-time factor and a
"N runs ≈ X minutes" table. **Size the sweep from that number.** Construct
containers are shared and the achievable run count is not knowable in advance.

### 2. Check the feedback sign — once

```bash
ros2 launch case_sim case_sim.launch.py gui:=true
```

Watch it in Gazebo and RViz. If the robot falls *faster* with the controller
active than it would coasting, the feedback polarity is inverted: set
`motor_direction_sign: -1.0` in `config/sweep_ranges.yaml`. The sign depends on
the quaternion-to-pitch conversion and the URDF joint axis direction, which were
chosen independently, and a wrong sign looks exactly like bad gains.

### 2b. Drive it around

Watching a robot hold still proves it does not fall over. Watching it drive
across the floor and come back proves it balances. Two drives are simulated,
selected by `drive_mode`:

```bash
# The firmware as written: FORWARD_ASSIST / BACKWARD_ASSIST / TURN_LEFT / TURN_RIGHT.
ros2 launch case_sim case_sim.launch.py gui:=true \
    drive_mode:=firmware duration_s:=60 impulse_magnitude_ns:=0.0

# Drive by leaning: the setpoint shifts and the balance loop holds the tilt.
ros2 launch case_sim case_sim.launch.py gui:=true \
    drive_mode:=lean_offset duration_s:=60 impulse_magnitude_ns:=0.0
```

`duration_s` and the zeroed impulse are for watching only: the default 10 s run
ends before you have driven anywhere, and the disturbance at t = 3 s would land
in the middle of it.

Then, in a second shell, with that window focused:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Commands latch: `teleop_twist_keyboard` sends one message per keypress, and the
controller holds the last one it received, so the robot keeps driving until you
press `k`. That mirrors the firmware, whose modes are latched voice commands
rather than a held throttle.

If that package is not installed, anything publishing `geometry_msgs/Twist` on
`/cmd_vel` will do:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
```

**If the robot drives the wrong way, negate `lean_gain_deg`** (or
`steer_gain_pwm` if it steers the wrong way). Both are signed calibration knobs
for the same reason `motor_direction_sign` is: which way a positive setpoint
shift leans depends on the pitch convention and the URDF joint axes, which were
chosen independently.

`drive_mode: "none"` is the default and is what every sweep measures. It is
byte-identical to the controller before the drive existed, so results recorded
earlier remain comparable.

#### What the two drives are for

`firmware` is a port, not a design: it adds a flat 80 PWM on top of the balance
correction instead of leaning the setpoint, so the chassis is driven without
being tilted into the motion and the PID ends up opposing its own throttle. Its
turn branches also bypass the deadzone remap and cannot steer at all while the
robot is near upright, because both wheels take the sign of the balance output
and only their speeds differ.

`lean_offset` does what a balancer is supposed to do. It is a proposal for the
firmware, not a description of it.

Which is better is measurable rather than arguable, on the same robots:

```bash
ros2 run case_sim sweep.py --runs <N> --set drive_mode=firmware    --out fw.csv
ros2 run case_sim sweep.py --runs <N> --set drive_mode=lean_offset --out lean.csv
ros2 run case_sim analyze.py --compare fw.csv lean.csv
```

**That comparison returns nothing today.** No sweep run publishes `/cmd_vel`, so
every mode sees a zero command, and a zero command collapses all three onto the
same arithmetic: `firmware` thresholds to `BALANCE_ONLY`, `lean_offset` shifts
the setpoint by zero and splits the wheels by zero. The two CSVs will be
identical by construction, not merely similar. Making it a real experiment needs
`run_supervisor` to command a velocity profile during the run, which is not
built.

### 3. Sweep

```bash
ros2 run case_sim sweep.py --runs <N> --out results.csv
ros2 run case_sim analyze.py results.csv
```

### 4. The validation experiment

`case_voice/balance_control.cpp:39` claims Kp=60 stabilises more plausible robots
than Kp=15. That claim came from a simulation that was never committed. Reproduce
it:

```bash
ros2 run case_sim sweep.py --runs <N> --set kp=60 --out kp60.csv
ros2 run case_sim sweep.py --runs <N> --set kp=15 --out kp15.csv
ros2 run case_sim analyze.py --compare kp60.csv kp15.csv
```

Both sweeps use the same seed, so the comparison is paired: identical robots,
different gains. If the ordering reproduces, the simulation has recovered a lost
result and earns some trust. If it contradicts, either the simulation or that
comment is wrong — and finding out which is worth more than the number itself.

## Changing parameters

`config/sweep_ranges.yaml` is the single authoritative source for every physical
value. The URDF, the controller, the launch file, and the sweep all read it.
Replacing an estimate with a measurement is a change to that file alone.

The two measurements worth taking as soon as the chassis exists are total mass
(kitchen scale) and centre-of-mass height (balance it on a pencil). They collapse
the two widest ranges in the sweep.

## Tests

```bash
colcon test --packages-select case_sim          # C++ unit tests
ros2 run case_sim sweep.py --self-test
ros2 run case_sim analyze.py --self-test
ros2 run case_sim run_supervisor.py --self-test
```

The C++ suite includes an equivalence test that compiles the real Arduino PID_v1
against a stubbed clock and drives it alongside the port, asserting they match
step for step. Every sweep result rests on that port being exact.

## Verified and unverified

Verified on a machine without ROS: the PID port is bit-identical to upstream over
5000 steps; the motor model reproduces the firmware's integer arithmetic exactly;
the URDF expands and its masses sum correctly at both ends of every range; all
launch arguments reach their consumers; the sweep's samples are in range and its
strata evenly covered.

Verified for the drive layer, on the same machine: routing `BALANCE_ONLY`
through the new per-wheel path reproduces `pwm_from_pid_output` exactly over the
whole output range, so `drive_mode: "none"` cannot have moved any earlier
result; and the ported assist and turn branches match hand-computed firmware
values, including that the turns bypass the deadzone remap.

**Not yet verified, because it requires the rosject:** that the package compiles;
that `gz_ros2_control` registers the IMU interfaces; that the
spawn-upright / activate / tilt / release startup sequence works end to end; the
actual real-time factor; the feedback sign; the sign of `lean_gain_deg` and
`steer_gain_pwm`, which are calibration knobs nothing offline can settle; and
whether Gazebo throttles `/clock`
below the physics rate, which would make the impulse land late and differently
late per run (check `ros2 topic hz /clock` during the benchmark). Nothing here
should be trusted as a result until a benchmark run passes.
