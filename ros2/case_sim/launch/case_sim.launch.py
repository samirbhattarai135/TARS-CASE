"""Launches one CASE simulation run.

One launch file for both uses, selected by the `gui` argument. Keeping a single
implementation means the run the presenter watches is the same run the sweep
measures.

Every launch argument is generated from config/sweep_ranges.yaml rather than
listed here. That file is the single authoritative source for physical
parameters, and a hand-maintained argument list beside it would drift.
"""

import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = "case_sim"

# Which block of sweep_ranges.yaml feeds which consumer. The plant shapes the
# robot description; the rest are controller parameters read at activation.
XACRO_BLOCKS = ("plant",)
CONTROLLER_BLOCKS = ("motor", "sensor", "controller")
SUPERVISOR_BLOCKS = ("run_protocol",)


def _load_ranges() -> dict:
    share = get_package_share_directory(PACKAGE)
    with open(Path(share) / "config" / "sweep_ranges.yaml") as handle:
        return yaml.safe_load(handle)


def _leaf_defaults(block: dict) -> dict:
    """Flatten one block to name -> default.

    A parameter with a `range` is swept and defaults to its `nominal`; a bare
    scalar is fixed. Both end up as launch arguments so a sweep run can
    override anything without the launch file enumerating what is sweepable.
    """
    out = {}
    for name, value in block.items():
        if isinstance(value, dict) and "nominal" in value:
            out[name] = value["nominal"]
        elif not isinstance(value, dict):
            out[name] = value
    return out


def _all_defaults(ranges: dict, blocks: tuple[str, ...]) -> dict:
    merged = {}
    for block in blocks:
        merged.update(_leaf_defaults(ranges.get(block, {})))
    return merged


def _resolved(context, names: dict) -> dict:
    """Read launch configurations back as the types the YAML declared.

    Launch hands everything back as a string. Feeding "20" to a controller
    parameter declared as an integer is a runtime failure at activation, and
    feeding "false" to a boolean is worse -- a non-empty string is truthy, so
    a disabled option would silently switch itself on.
    """
    out = {}
    for name, default in names.items():
        raw = LaunchConfiguration(name).perform(context)
        if isinstance(default, bool):
            out[name] = raw.strip().lower() in ("1", "true", "yes")
        elif isinstance(default, int):
            out[name] = int(float(raw))
        elif isinstance(default, float):
            out[name] = float(raw)
        else:
            out[name] = raw
    return out


def _setup(context, *_args, **_kwargs):
    ranges = _load_ranges()
    share = get_package_share_directory(PACKAGE)

    plant = _resolved(context, _all_defaults(ranges, XACRO_BLOCKS))
    control = _resolved(context, _all_defaults(ranges, CONTROLLER_BLOCKS))
    protocol = _resolved(context, _all_defaults(ranges, SUPERVISOR_BLOCKS))

    gui = LaunchConfiguration("gui").perform(context).strip().lower() in ("1", "true", "yes")
    result_file = LaunchConfiguration("result_file").perform(context)
    seed = int(LaunchConfiguration("seed").perform(context))

    xacro_path = Path(share) / "urdf" / "case.urdf.xacro"

    # Merge the controller defaults with this run's overrides into a temp params
    # file. The controller_manager lives inside Gazebo (gz_ros2_control is a
    # world plugin, not a separate node) and loads parameters from the file the
    # URDF names -- so that path must be THIS run's file. Passing overrides only
    # to the spawner would leave two parameter sources with no guarantee about
    # which one the controller sees.
    with open(Path(share) / "config" / "controllers.yaml") as handle:
        params = yaml.safe_load(handle)
    controller_params = params.setdefault("balance_controller", {}).setdefault(
        "ros__parameters", {}
    )
    controller_params.update(control)
    controller_params["noise_seed"] = seed
    controller_params["wheel_radius"] = plant["wheel_radius_m"]

    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix="_case_controllers.yaml", delete=False
    )
    yaml.safe_dump(params, tmp)
    tmp.close()

    # The URDF's gz_ros2_control plugin reads this path, so the robot
    # description must be expanded AFTER the file exists.
    # Passed as a dict: several values contain spaces (imu_offset_xyz_m is
    # "0 0 0.030"), and a string round-trip through split() would truncate them.
    mappings = {k: str(v) for k, v in plant.items()}
    mappings["controller_params_file"] = tmp.name

    # -r: physics must be RUNNING for the controller to activate at all --
    # gz_ros2_control gates controller_manager update() on
    # `sim_period >= control_period`, and sim_period is 0 in a paused world. The
    # world starts at real time and the robot spawns upright, which is an
    # equilibrium it holds through activation; run_supervisor then tilts it and
    # releases the rate limit.
    world = Path(share) / "worlds" / "case_world.sdf"
    gz_args = f"-r {world}" if gui else f"-r -s {world}"

    supervisor = Node(
        package=PACKAGE,
        executable="run_supervisor.py",
        name="run_supervisor",
        parameters=[
            {
                "use_sim_time": True,
                "world_name": "case_world",
                "model_name": "case",
                "controller_name": "balance_controller",
                "result_file": result_file,
                "gate_lower_deg": control["gate_lower_deg"],
                "gate_upper_deg": control["gate_upper_deg"],
                "lean_deg": control["lean_deg"],
                "spawn_z_m": plant["wheel_radius_m"] + 0.005,
                # Demo runs at wall-clock pace so it is watchable; the sweep
                # runs as fast as the machine allows.
                "real_time_factor": 1.0 if gui else 0.0,
                **protocol,
            }
        ],
        output="screen",
    )

    actions = [
        # gz_sim.launch.py does not publish /clock. Without this bridge every
        # node with use_sim_time stays frozen at t=0: the supervisor's timer
        # never fires and the run can only end by timing out.
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="log",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    Path(get_package_share_directory("ros_gz_sim"))
                    / "launch"
                    / "gz_sim.launch.py"
                )
            ),
            launch_arguments={"gz_args": gz_args}.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {
                    "use_sim_time": True,
                    "robot_description": _expand_xacro(xacro_path, mappings),
                }
            ],
            output="screen",
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-topic", "robot_description",
                "-name", "case",
                # Upright, just above the ground. The initial tilt is applied
                # by run_supervisor after the controller activates -- a tilted
                # robot would fall during activation.
                "-z", str(plant["wheel_radius_m"] + 0.005),
            ],
            output="screen",
        ),
        # No --param-file here: the controller_manager runs inside Gazebo and
        # already loaded this run's parameters from the file the URDF names.
        # Passing them again would create a second source with no stated winner.
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["balance_controller", "joint_state_broadcaster"],
            output="screen",
        ),
        supervisor,
        # The supervisor owns the run's lifetime. Without this, it exits and
        # Gazebo keeps running until sweep.py's wall-clock timeout kills the
        # launch -- making every run cost the full timeout.
        RegisterEventHandler(
            OnProcessExit(target_action=supervisor, on_exit=[EmitEvent(event=Shutdown())])
        ),
    ]

    if gui:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(Path(share) / "config" / "case.rviz")],
                parameters=[{"use_sim_time": True}],
                output="log",
            )
        )

    return actions


def _expand_xacro(path: Path, mappings: dict) -> str:
    import xacro

    return xacro.process_file(str(path), mappings=mappings).toxml()


def generate_launch_description() -> LaunchDescription:
    ranges = _load_ranges()
    declared = [
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("result_file", default_value="/tmp/case_run.json"),
    ]
    for block in XACRO_BLOCKS + CONTROLLER_BLOCKS + SUPERVISOR_BLOCKS:
        for name, default in _leaf_defaults(ranges.get(block, {})).items():
            declared.append(DeclareLaunchArgument(name, default_value=str(default)))

    return LaunchDescription(declared + [OpaqueFunction(function=_setup)])
