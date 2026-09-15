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
git clone <this repo>          # colcon recurses and finds ros2/case_sim
cd ~/ros2_ws && colcon build --packages-select case_sim && source install/setup.bash
```

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

**Not yet verified, because it requires the rosject:** that the package compiles;
that `gz_ros2_control` registers the IMU interfaces; that the
spawn-upright / activate / tilt / release startup sequence works end to end; the
actual real-time factor; the feedback sign; and whether Gazebo throttles `/clock`
below the physics rate, which would make the impulse land late and differently
late per run (check `ros2 topic hz /clock` during the benchmark). Nothing here
should be trusted as a result until a benchmark run passes.
