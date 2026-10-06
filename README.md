# ADMM-CLM Workspace

**Project website:** [https://admm-clm.github.io/](https://admm-clm.github.io/)

**Docker image:** [pengyuanshu/aclm on Docker Hub](https://hub.docker.com/repository/docker/pengyuanshu/aclm/)

OCS2-based model predictive control for cooperative cargo transport with two,
three, or four Unitree B1 quadrupeds equipped with Z1 arms. The core package
provides centralized and ADMM-based optimization, terrain-aware references and
foothold selection, and RViz visualization. Examples use **model-rollout
simulation**, not Gazebo physics or hardware controllers; perceptive scenarios
obtain terrain from PLY point clouds.

<div align="center">

https://github.com/user-attachments/assets/69bd0968-ad7f-4b33-b152-e1a8099fd4c9

</div>

**Contents**

- [ADMM-CLM Workspace](#admm-clm-workspace)
  - [1. Workspace Structure](#1-workspace-structure)
  - [2. Installation](#2-installation)
    - [Prebuilt Docker Image](#prebuilt-docker-image)
    - [Source Build](#source-build)
  - [3. Launch Scenarios](#3-launch-scenarios)
    - [Launch with Docker](#launch-with-docker)
    - [Launch without Docker](#launch-without-docker)
    - [Box Scenarios](#box-scenarios)
    - [Launch Arguments](#launch-arguments)
    - [Terrain Pipeline](#terrain-pipeline)
    - [Gait Selection and Motion Targets](#gait-selection-and-motion-targets)
  - [Citation](#citation)
  - [License and Acknowledgements](#license-and-acknowledgements)

## 1. Workspace Structure

`ocs2_multi_robot` and `ocs2_quadruped` are ordinary directories tracked by
the workspace. `ocs2`, `mapping_third_party`, and `ocs2_robotic_assets` are Git submodules.

```text
aclm_workspace/
  Dockerfile, compose.yaml, docker/  Container build and runtime
  src/
    ocs2_multi_robot/                Cooperative transport MPC
      config/, launch/               Configuration and scenarios
      include/, src/                 Controller, IK, and visualization
      assets/robot/                  Robot and payload descriptions
      assets/terrain/                Point clouds and conversion tools
      third_party/qpOASES/           Integrated QP solver
    ocs2/                            OCS2 submodule
    ocs2_robotic_assets/             ROS 1 assets for OCS2 examples and tests
    ocs2_quadruped/                  Quadruped control and ROS package
    mapping_third_party/             Mapping dependencies submodule
```

See the [package README](src/ocs2_multi_robot/README.md) for internal structure.

`ocs2_robotic_assets` is pinned to ROS 1-compatible commit
`b126d00d55f1e67905c3c1516995466df6873c5c`. Its upstream default `ros2`
branch uses ament and is not compatible with this ROS Noetic catkin workspace.
Generated build output and `auto_generated/` model libraries should not be committed.

## 2. Installation

Run these commands in Linux or WSL2 Ubuntu 20.04. Choose the prebuilt image
or build from source.

### Prebuilt Docker Image

Clone the workspace for its Compose configuration and pull the image:

```bash
git clone https://github.com/GTLIDAR/aclm_workspace.git aclm_workspace
cd aclm_workspace
docker pull pengyuanshu/aclm:noetic-v1
docker tag pengyuanshu/aclm:noetic-v1 aclm:noetic
```

The tag matches `compose.yaml`. The published image is an independent snapshot;
its paths and entrypoint may differ. Build from source for the current layout.

### Source Build

For either Docker or native builds, populate the source dependencies first:

```bash
git clone --recurse-submodules \
  https://github.com/GTLIDAR/aclm_workspace.git aclm_workspace
cd aclm_workspace
```

For an existing checkout, run `git submodule update --init --recursive`.
Private repositories require read access. Docker does not fetch submodules.

**Docker:** install Docker Engine with Compose, or enable Docker Desktop WSL
Integration. No host ROS installation is needed.

```bash
export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"
docker compose build ocs2
```

This builds `aclm:noetic`. Sources are copied, not mounted; rebuild after edits.
Compose uses a mirrored base image; set
`ROS_BASE_IMAGE=osrf/ros:noetic-desktop-full` to use the official image.

**Native:** use ROS Noetic, `catkin_tools`, CMake >= 3.14, C++17, and Eigen
**3.3.7 exactly**. Install system dependencies listed in
[package.xml](src/ocs2_multi_robot/package.xml) and the companion packages;
see the [Dockerfile](Dockerfile) for the container's dependency setup.

```bash
source /opt/ros/noetic/setup.bash
catkin init
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build ocs2_multi_robot --jobs 4 --parallel-packages 1
```

Reduce `--jobs` if memory is limited. Use `catkin build`, not `catkin_make`.
Both source-build methods need network access for first-build dependency downloads.

## 3. Launch Scenarios

Choose a launch method below, then select a scenario from the table.
Use **one scenario per ROS master**: scenarios share parameters, node names,
and command topics. Start with a fresh master for isolated comparisons, because
ordinary dummy launches do not explicitly clear a previous `/use_perceptive`
parameter. Docker containers use the shared host network.

### Launch with Docker

The entrypoint and `docker exec` commands below match images built with this
checkout's Dockerfile. The published snapshot may use a different entrypoint
or workspace layout; verify its environment before applying these commands.

Use Docker Engine with Compose or Docker Desktop with Ubuntu WSL Integration.
On Docker Desktop 4.34+, enable host networking under
**Settings > Resources > Network > Enable host networking**.
RViz and the gait-command window need X11/XWayland or WSLg. On WSLg, check that
`echo "$DISPLAY"` is not empty and `/tmp/.X11-unix/` contains a display socket.
On native Linux, authorize the matching user from a graphical terminal:

```bash
xhost +si:localuser:"$(id -un)"
```

From the workspace root, launch the two-robot ADMM example:

```bash
docker compose run --rm --pull never --name aclm-demo ocs2 \
  roslaunch ocs2_multi_robot two_quadruped_perceptive_admm.launch
```

`ocs2` is the Compose service, `aclm-demo` is the container name, and
`ocs2_multi_robot` is the ROS package. The entrypoint loads ROS and the compiled
workspace; `roslaunch` starts a master if needed. Replace the launch filename
to select another scenario.

To start an interactive container instead:

```bash
docker compose run --rm --pull never --name aclm-demo ocs2
```

This opens Bash at `/workspace`. To open another ROS-enabled terminal in the
running container:

```bash
docker exec -it aclm-demo /usr/local/bin/ocs2-entrypoint bash
```

Run the terrain and motion commands below inside these container terminals.
Press **Ctrl+C** to stop a scenario; in interactive Bash mode, also run `exit`.
`--rm` deletes the exited container, not the image. Copy needed container-local
files out before exiting.

### Launch without Docker

A graphical session with RViz and `gnome-terminal` is needed for visualization
and the gait-command window. From the built workspace, source the development
environment in **every new terminal**, then launch:

```bash
source devel/setup.bash
roslaunch ocs2_multi_robot two_quadruped_dummy_admm.launch
```

A correctly built and sourced workspace does not require a manual
`ROS_PACKAGE_PATH` export. While `roslaunch` is running, open another terminal,
source `devel/setup.bash`, and run the terrain and motion commands below there.
Press **Ctrl+C** in the launch terminal to stop the scenario.

### Box Scenarios

| Robots and mode | Centralized | ADMM |
| --- | --- | --- |
| 2, non-perceptive | [two_quadruped_dummy.launch](src/ocs2_multi_robot/launch/two_quadruped_dummy.launch) | [two_quadruped_dummy_admm.launch](src/ocs2_multi_robot/launch/two_quadruped_dummy_admm.launch) |
| 3, non-perceptive | [three_quadruped_dummy.launch](src/ocs2_multi_robot/launch/three_quadruped_dummy.launch) | [three_quadruped_dummy_admm.launch](src/ocs2_multi_robot/launch/three_quadruped_dummy_admm.launch) |
| 4, non-perceptive | [four_quadruped_dummy.launch](src/ocs2_multi_robot/launch/four_quadruped_dummy.launch) | [four_quadruped_dummy_admm.launch](src/ocs2_multi_robot/launch/four_quadruped_dummy_admm.launch) |
| 2, perceptive | [two_quadruped_perceptive.launch](src/ocs2_multi_robot/launch/two_quadruped_perceptive.launch) | [two_quadruped_perceptive_admm.launch](src/ocs2_multi_robot/launch/two_quadruped_perceptive_admm.launch) |
| 3, perceptive | [three_quadruped_perceptive.launch](src/ocs2_multi_robot/launch/three_quadruped_perceptive.launch) | [three_quadruped_perceptive_admm.launch](src/ocs2_multi_robot/launch/three_quadruped_perceptive_admm.launch) |
| 4, perceptive | [four_quadruped_perceptive.launch](src/ocs2_multi_robot/launch/four_quadruped_perceptive.launch) | [four_quadruped_perceptive_admm.launch](src/ocs2_multi_robot/launch/four_quadruped_perceptive_admm.launch) |

Additional payload scenarios are
[three_quadruped_perceptive_admm_table.launch](src/ocs2_multi_robot/launch/three_quadruped_perceptive_admm_table.launch)
and
[four_quadruped_perceptive_admm_stretcher.launch](src/ocs2_multi_robot/launch/four_quadruped_perceptive_admm_stretcher.launch).
Each selects its own task configuration, payload model, and RViz view.

### Launch Arguments

| Argument | Purpose |
| --- | --- |
| `rviz` | Start RViz; defaults to `true`. Does not control the gait terminal. |
| `taskFile` | Controller configuration, matched to robot count, payload, and solver. |
| `gaitCommandFile` | Gait definitions for the keyboard command node. |
| `urdfFile`, `armUrdfFile`, `cargoUrdfFile` | Robot, arm, and payload descriptions. |
| `enable_obstacle_avoidance` | Enable configured obstacle-avoidance terms. Defaults depend on the launch. |
| `use_perceptive` | Enable perception in perceptive launch files. |
| `world_name` | Select a PLY terrain asset in perceptive launch files. |

For example, in a ROS-enabled native or container terminal:

```bash
roslaunch ocs2_multi_robot four_quadruped_perceptive_admm.launch \
  world_name:=gap_slope_course rviz:=false
```

Disabling `use_perceptive` does not replace terrain-specific initial states with
flat-ground ones; use a dummy launch for a non-perceptive scenario.

### Terrain Pipeline

The shared
[elevation_mapping_fixed.launch](src/ocs2_multi_robot/launch/elevation_mapping_fixed.launch)
starts the PLY reader, elevation mapping with postprocessing, and convex plane
decomposition. The resulting planar regions feed terrain-aware control and
RViz visualization.

This pipeline uses the C++ `elevation_mapping` node, not
`elevation_mapping_cupy`. CUDA/CuPy and TurtleBot/Gazebo demos in third-party
READMEs are optional upstream examples, not requirements for these scenarios.

`world_name` selects `$(find ocs2_multi_robot)/assets/terrain/point_cloud/<world_name>.ply`;
it does not start a Gazebo world. List available assets with:

```bash
ls "$(rospack find ocs2_multi_robot)/assets/terrain/point_cloud/"
```

Scenario launches supply a default `world_name`. To launch only the terrain
pipeline, provide it explicitly:

```bash
roslaunch ocs2_multi_robot elevation_mapping_fixed.launch \
  world_name:=gap_slope_course
```

Standalone mapping also needs the configured TF frames, including the tracked
base frame (default `cargo`). Full perceptive scenarios provide these. Changing
terrain does not automatically update initial poses or waypoint coordinates.

### Gait Selection and Motion Targets

Wait for the initial MPC policy and robot visualization, then select a gait
such as `trot` in the gait-command terminal. Definitions are in
[gait.info](src/ocs2_multi_robot/config/info/gait.info). Motion targets can be
supplied through:

- RViz **2D Nav Goal**, on `/move_base_simple/goal`, for cargo position and heading.
- `geometry_msgs/Twist` messages on `/cmd_vel`.
- The `/load_waypoint_trajectory` service for named YAML trajectories.

```bash
rosservice call /load_waypoint_trajectory \
  "yaml_file: '$(rospack find ocs2_multi_robot)/config/yaml/waypoints.yaml'
trajectory_name: 'gap_slope_turn_course'
velocity: 0.3"
```

Inspect [waypoints.yaml](src/ocs2_multi_robot/config/yaml/waypoints.yaml) first:
the trajectory must suit the scene and current cargo position. A valid new
RViz goal switches back to goal mode; it requires an observation and a TF
transform to `odom`. `/cmd_vel` is ignored while waypoint mode is active.
A motion target does not replace gait selection.

## Citation

If you use this work, please cite:

> Ziyi Zhou*, Pengyuan Shu*, Ruize Cao*, Yuntian Zhao, and Ye Zhao.
> **ACLM: ADMM-Based Distributed Model Predictive Control for Collaborative
> Loco-Manipulation.** arXiv:2603.07095, March 7, 2026.
> DOI: [10.48550/arXiv.2603.07095](https://doi.org/10.48550/arXiv.2603.07095).
> *The first three authors contributed equally.*

```bibtex
@misc{zhou2026aclm,
  title         = {{ACLM}: {ADMM}-Based Distributed Model Predictive Control for Collaborative Loco-Manipulation},
  author        = {Zhou, Ziyi and Shu, Pengyuan and Cao, Ruize and Zhao, Yuntian and Zhao, Ye},
  year          = {2026},
  eprint        = {2603.07095},
  archivePrefix = {arXiv},
  primaryClass  = {cs.RO},
  doi           = {10.48550/arXiv.2603.07095},
  url           = {https://arxiv.org/abs/2603.07095}
}
```

Machine-readable metadata is in [CITATION.cff](CITATION.cff). This citation
identifies the arXiv preprint, not an IEEE proceedings version. Also cite upstream
libraries and methods actually used; see the
[third-party citation instructions](THIRD_PARTY_NOTICES.md#academic-attribution).

## License and Acknowledgements

Original ADMM-CLM contributions are available under the [MIT License](LICENSE).
Upstream-derived code, dependencies, and robot/terrain assets retain their
own licenses; the workspace and its binaries are **not MIT-only**.

We thank the OCS2, ANYbotics mapping, elevation_mapping_cupy, quadruped-control,
qpOASES, and Unitree contributors. The mapping collection was copied from
[DRCL-USC/Quadruped_Wrapper](https://github.com/DRCL-USC/Quadruped_Wrapper/tree/837f51807e192241437c642bc5dc9c93a46224f0/third_party),
whose component upstreams and pinned versions are recorded in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

Those notices cover the OCS2, OCS2 Robotic Assets, and Mapping Third-Party submodules, including
the CGAL GPL component. Before distributing binaries or Docker images, also
review the separately retained [qpOASES LGPL terms](src/ocs2_multi_robot/third_party/qpOASES/LICENSE.txt)
and [Unitree resource license](src/ocs2_multi_robot/assets/LICENSE.unitree);
asset-specific provenance still needs confirmation.
