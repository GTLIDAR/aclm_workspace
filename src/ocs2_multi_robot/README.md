# OCS2 Multi-Robot

OCS2-based model predictive control for cooperative cargo transport with two,
three, or four Unitree B1 quadrupeds equipped with Z1 arms. The package provides
centralized and ADMM-based optimization, terrain-aware references and foothold
selection, and RViz visualization.

The launch files below use a **dummy model-rollout simulation**, not Gazebo
physics or hardware controllers. Perceptive scenarios obtain terrain from PLY
point clouds.

## Requirements

- ROS 1 Noetic on Ubuntu 20.04, a C++17 compiler, CMake 3.14 or newer,
	`catkin_tools`, and Eigen 3.3.7 exactly (as required by CMake).
- The dependencies declared in [package.xml](package.xml), including the
	companion source packages directly under the workspace's `src` directory.
- A graphical session for RViz and `gnome-terminal` for the gait-command window.

Build from the complete workspace; this package alone does not contain all
required controller and mapping dependencies. Whole-body IK and qpOASES are
built inside this package. Robot descriptions live in `assets/robot` and PLY
terrain data and conversion tools live in `assets/terrain`; these are ordinary
assets, not separate catkin packages.

The former quadruped control and ROS packages are now the single
`ocs2_quadruped` package. Its messages, headers, and launch nodes use that name.
The original Unitree repository and its Gazebo-only launch, plugins, controllers,
and messages have been removed. The model-rollout scenarios do not depend on
Gazebo. Description assets retain their original Gazebo tags and controller
configuration as model metadata, but this workspace does not supply a Gazebo
simulation pipeline.

## Build

Run from the workspace root, replacing the example path with your checkout:

```bash
source /opt/ros/noetic/setup.bash
cd ~/aclm_workspace
catkin init
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build ocs2_multi_robot -j 4
source devel/setup.bash
```

Reduce `-j` if compilation exhausts memory. Once dependencies are built, use
`catkin build ocs2_multi_robot --no-deps -j 4` for a package-only rebuild.
Use `--parallel-packages 1` for limited memory. Do not mix `catkin_make` with
this workspace's `catkin build` layout. First builds of the BLASFEO, HPIPM and
CGAL wrappers need network access to download their pinned sources.

Source `devel/setup.bash` in every new terminal and restart running nodes after
rebuilding. While `roslaunch` is running, send motion commands from another
sourced terminal. A correctly built and sourced workspace does not require a manual
`ROS_PACKAGE_PATH` export. These instructions use the development space, not an
installed package distribution. Initial startup can take longer when model
libraries are regenerated in the ignored runtime-output directory
`auto_generated/`.

## Launch Usage

### Quick Start

```bash
# Centralized, two robots, without terrain perception
roslaunch ocs2_multi_robot two_quadruped_dummy.launch

# Centralized, two robots, with terrain perception
roslaunch ocs2_multi_robot two_quadruped_perceptive.launch

# ADMM, three robots, with terrain perception
roslaunch ocs2_multi_robot three_quadruped_perceptive_admm.launch
```

Run these examples separately. Scenarios share global parameters, node names,
and command topics; use one scenario per ROS master. For isolated comparisons,
start with a fresh master. Ordinary dummy launches do not explicitly clear a
previously set `/use_perceptive` parameter.

### Box Scenarios

| Robots and mode | Centralized | ADMM |
| --- | --- | --- |
| 2, non-perceptive | [two_quadruped_dummy.launch](launch/two_quadruped_dummy.launch) | [two_quadruped_dummy_admm.launch](launch/two_quadruped_dummy_admm.launch) |
| 3, non-perceptive | [three_quadruped_dummy.launch](launch/three_quadruped_dummy.launch) | [three_quadruped_dummy_admm.launch](launch/three_quadruped_dummy_admm.launch) |
| 4, non-perceptive | [four_quadruped_dummy.launch](launch/four_quadruped_dummy.launch) | [four_quadruped_dummy_admm.launch](launch/four_quadruped_dummy_admm.launch) |
| 2, perceptive | [two_quadruped_perceptive.launch](launch/two_quadruped_perceptive.launch) | [two_quadruped_perceptive_admm.launch](launch/two_quadruped_perceptive_admm.launch) |
| 3, perceptive | [three_quadruped_perceptive.launch](launch/three_quadruped_perceptive.launch) | [three_quadruped_perceptive_admm.launch](launch/three_quadruped_perceptive_admm.launch) |
| 4, perceptive | [four_quadruped_perceptive.launch](launch/four_quadruped_perceptive.launch) | [four_quadruped_perceptive_admm.launch](launch/four_quadruped_perceptive_admm.launch) |

Additional payload scenarios:

- [three_quadruped_perceptive_admm_table.launch](launch/three_quadruped_perceptive_admm_table.launch)
- [four_quadruped_perceptive_admm_stretcher.launch](launch/four_quadruped_perceptive_admm_stretcher.launch)

Each selects its own task configuration, payload model, and RViz view. Changing
only the cargo URDF is insufficient: handle poses, robot offsets, and cargo
heights must match the payload geometry.

### Arguments

| Argument | Purpose |
| --- | --- |
| `rviz` | Start RViz; defaults to `true`. |
| `taskFile` | Controller configuration, matched to robot count, payload, and solver. |
| `gaitCommandFile` | Gait definitions for the keyboard command node. |
| `urdfFile`, `armUrdfFile`, `cargoUrdfFile` | Robot, arm, and payload descriptions. |
| `enable_obstacle_avoidance` | Enable configured obstacle-avoidance terms. Defaults depend on the launch. |
| `use_perceptive` | Enable perception in perceptive launch files. |
| `world_name` | Select a PLY terrain asset in perceptive launch files. |

```bash
roslaunch ocs2_multi_robot four_quadruped_perceptive_admm.launch \
	world_name:=gap_slope_course rviz:=false
```

`rviz:=false` does not disable the gait terminal. Disabling `use_perceptive`
does not replace terrain-specific initial states with flat-ground ones; use a
dummy launch for the corresponding non-perceptive scenario.

## Terrain and Motion Commands

The shared [elevation_mapping_fixed.launch](launch/elevation_mapping_fixed.launch)
starts the PLY reader, elevation mapping with postprocessing, and convex plane
decomposition. The resulting planar regions feed terrain-aware control and
RViz visualization.

The scenarios use C++ `elevation_mapping`, not the optional GPU
`elevation_mapping_cupy` node. The CuPy and TurtleBot/Gazebo instructions in
third-party READMEs describe separate demos.

`world_name` selects `$(find ocs2_multi_robot)/assets/terrain/point_cloud/<world_name>.ply`;
it does not start a Gazebo world. List available assets with:

```bash
ls "$(rospack find ocs2_multi_robot)/assets/terrain/point_cloud/"
```

The shared launch requires `world_name`; scenario launches pass their own
defaults. To launch only the terrain pipeline:

```bash
roslaunch ocs2_multi_robot elevation_mapping_fixed.launch \
	world_name:=gap_slope_course
```

Standalone mapping also needs the configured TF frames, including the tracked
base frame (default `cargo`). Full perceptive scenarios provide these. Changing
the terrain does not automatically update initial poses or waypoint coordinates.

Wait for the initial MPC policy and robot visualization, then select a gait in
the gait-command terminal using [gait.info](config/info/gait.info). Motion targets
can be supplied through:

- RViz **2D Nav Goal**, on `/move_base_simple/goal`, for cargo position and heading.
- `geometry_msgs/Twist` messages on `/cmd_vel`.
- The `/load_waypoint_trajectory` service for named YAML trajectories.

```bash
rosservice call /load_waypoint_trajectory \
	"yaml_file: '$(rospack find ocs2_multi_robot)/config/yaml/waypoints.yaml'
trajectory_name: 'gap_slope_turn_course'
velocity: 0.3"
```

Inspect [waypoints.yaml](config/yaml/waypoints.yaml) first: the selected trajectory
must suit the scene and current cargo position. A valid new RViz goal switches
back to goal mode; it requires an observation and a TF transform to `odom`.
`/cmd_vel` is ignored while waypoint mode is active. A motion target does not
replace gait selection.

## Configuration

Task files in [config/info/](config/info/) follow these box naming conventions,
where `<count>` is `two`, `three`, or `four`:

| Mode | Pattern |
| --- | --- |
| Centralized | `<count>_quadruped_w_cargo.info` |
| ADMM | `<count>_quadruped_w_cargo_admm.info` |
| Centralized, perceptive | `<count>_quadruped_w_cargo_perceptive.info` |
| ADMM, perceptive | `<count>_quadruped_w_cargo_perceptive_admm.info` |

Payload-specific files use `table` or `stretcher` instead of `cargo`. Check the
launch's `taskFile` argument for the exact mapping.

Key sections include solver settings (`mpc`, `ddp`, `sqp`, `alternating`), robot
and cargo models, handle poses, initial states, gait schedules, costs, and
constraint penalties. `initialStateOffset` stores robot offsets from the cargo;
orientation entries are **yaw, pitch, roll**, in radians. Initialization adds
position offsets directly in world axes, without rotating them by cargo yaw.
Orientation offsets are composed with the cargo rotation.

Initial offsets do not lock orientations during motion. The centralized
interface uses `initialRobotState(2)` for base height, whereas the ADMM optimizer
uses cargo height plus the robot's z offset. ADMM launches still use
`MultiRobotDummyNode` and the target publisher, both of which construct the
centralized interface. Thus the rollout/target initialization and ADMM optimizer
can use different base heights even within one launch; check both definitions
when editing a task file.

The centralized interface derives robot count from cargo handles; the ADMM
optimizer uses `alternating.numConsensusConstraints`. For an ADMM scenario,
these counts must agree because the shared rollout, target publisher, and
visualizer still count handles. Changing only the ROS `num_robots` parameter
is insufficient.

Mapping, segmentation, and waypoint settings are in [config/yaml/](config/yaml/).
Projected foothold markers on `/cen_opt/predicted_footholds` are distinct from
optimized touchdown markers on `/cen_opt/optimizedStateTrajectory`. Check RViz
display and namespace settings when a marker is not visible.

## File Structure

```text
aclm_workspace/
	src/
		ocs2/                         OCS2 submodule
		ocs2_quadruped/                Quadruped control and ROS package
		mapping_third_party/           Mapping dependencies submodule
		ocs2_multi_robot/
			CMakeLists.txt              Build and test targets
			package.xml                 ROS dependencies
			config/info/                Models, controller tasks, and gaits
			config/yaml/                Mapping, segmentation, and waypoints
			launch/                     Scenario and shared pipeline launches
			include/ocs2_multi_robot/   Public headers and utilities
			assets/robot/              B1, B1Z1, Z1 and payload descriptions
			assets/terrain/            PLY point clouds and conversion tools
			third_party/qpOASES/       Vendored QP solver source and license
			doc/whole_body_ik.md       Integrated IK API and usage
			src/
				kinematics/              Whole-body differential IK
				alternating_base/        ADMM interfaces and consensus solver
				constraint/              Robot, payload, and terrain constraints
				cost/                    Tracking costs
				dynamics/                Robot and cargo dynamics
				initialization/          Solver initialization
				precomputation/          Cached optimization quantities
				reference_manager/       Gait and terrain-aware references
				robot_interface/         Robot model interfaces
				terrain/                 Height maps and region selection
				visualization/           Markers, TF, and full-body reconstruction
				common/, utils/          Shared helpers
			rviz/                      Robot-count and payload-specific views
			script/                    Benchmarks, residual plots, ROS utilities
			srv/                       Waypoint and mode services
			auto_generated/            Generated model artifacts
	build/                         Catkin build output
	devel/                         Development environment
```

Main executables are `MultiRobotDummyMpcNode` (centralized MPC),
`MultiRobotDummyConsensusMpcNode` (ADMM), `MultiRobotDummyNode` (rollout and
visualization), and `MultiRobotGeneralizedTargetTrajectoriesPublisher` (targets).

## Troubleshooting

- **Missing packages:** build dependencies and source the correct development
	environment in the current terminal.
- **Missing terrain:** check the PLY path, `/points`, mapping nodes, and TF.
	Successful launch parsing alone does not prove valid map output.
- **Unexpected grasp posture:** check the task configuration, cargo model,
	handle poses, and initial offsets together.
- **No gait terminal:** check `gnome-terminal` and the graphical session. Some
	dummy launches use `title=...` instead of `--title=...` in their launch
	prefix, which can also prevent startup. With the scenario running, use
	`rosrun ocs2_multi_robot GaitCommandNode` in another sourced terminal if
	the launch's gait terminal did not start.
- **RViz errors:** inspect rendering errors and TF status. Hiding map or robot
	displays is not a substitute for fixing a rendering failure.
- **View tracking:** `view_frame_publisher.py` uses Python 3 and is installed
	by catkin. Check `/view_frame_publisher` and the `odom` to `view` transform.
	Fixed `virtual_ee_joint` transforms come from the URDF through `/tf_static`,
	not dynamic joint-state messages. Rebuild Docker images after source fixes;
	existing images do not automatically receive workspace changes.
- **Slow simulation:** use Release builds and inspect timing output. Configured
	MPC/MRT frequencies are targets, not guaranteed rates; simulation time may
	advance more slowly than wall time.

Keep terrain, controller settings, robot count, and visualization workload
unchanged when comparing MPC timings.

## License and Citation

Original contributions use the workspace's [MIT License](../../LICENSE).
Existing upstream-derived code and bundled resources retain their original
terms, including BSD source headers, qpOASES LGPL, and Unitree MPL notices.
The complete package is not MIT-only; CGAL-dependent binaries need a separate
distribution assessment.

See [THIRD_PARTY_NOTICES.md](../../THIRD_PARTY_NOTICES.md) for the OCS2 and
Mapping Third-Party submodules' provenance, license conditions, and upstream
citations, and [CITATION.cff](../../CITATION.cff)
for the ADMM-CLM software citation. These relative links refer to the source
workspace. Installed workspace notices and license copies are placed in
`share/ocs2_multi_robot/licensing/`.