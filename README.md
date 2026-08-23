# tiago_wbc_ros2 🤖

<p align="center">
  <img src="assets/logo.png" alt="tiago_wbc_ros2 logo" width="220">
</p>

A minimal ROS 2 whole-body control demo for the TIAGo robot.

## Getting Started

### 1. Clone the repository

Clone the repository and initialize its submodules:

```bash
git clone https://github.com/hucebot/tiago_wbc_ros2.git
cd tiago_wbc_ros2
git submodule update --init --recursive
```

### 2. Configure the environment

Create a `.env` file from the provided template:

```bash
cp .env.template .env
```

Then edit `.env` and set the `DDS_ENV` variable:

- `local` — for local testing (recommended for the demo)
- `robot` — to connect to the real robot

### 3. Build the Docker image

Build the deployment image (this may take around 40 minutes the first time):

```bash
make build-deploy
```

### 4. Launch the demo

Start the Docker container:

```bash
make deploy
```

After a few moments, **RViz** should open and display the TIAGo robot model. You can interact with the robot by dragging the interactive markers in RViz, as shown in the demo video below.

![Demo Video](assets/tiago_pro_rviz.gif)
---

## MuJoCo Simulation (Tiago Pro)

Instead of the real robot, the OpenSoT/cartesian_interface pipeline can drive a MuJoCo
simulation via the `tiago_pro_mujoco_bridge` package. The robot assets (XMLs + meshes)
are vendored in this repo under `robots/pal_tiago_pro/`.

### 1. Start the dev container

```bash
make dev
```

### 2. Build and launch, inside the container

Build once:

```bash
cd /home/forest_ws
colcon build --symlink-install --packages-select tiago_control_node tiago_pro_mujoco_bridge
source install/setup.bash
source setup.bash
```

Then, in two separate terminals (both inside the container): the normal real-robot bringup, unchanged -

```bash
ros2 launch tiago_control_node bringup.launch.py robot_model:=pro   # Launches RViz2
```

- and the MuJoCo bridge, which stands in for the real Tiago Pro hardware:

```bash
ros2 launch tiago_pro_mujoco_bridge mujoco_bridge.launch.py
```

This starts two nodes: `mujoco_sim_node` (owns the MuJoCo model/physics/viewer, publishes
`/joint_states`, drives actuators from `/opensot/joint_states`) and
`episode_orchestrator_node` (handles episode resets - see `/mujoco_bridge/end_episode` below).

Before running anything below: in RViz, **uncheck "Enable Task"** on both interactive markers (right-click each marker → Enable Task). `cartesian_interface_node` republishes whatever pose the marker holds at 100Hz whenever it's enabled, which fights any scripted plan on the same topic.

#### A note on the two argument styles below

Everything here is either a **launch file** or a **node run directly**, and each takes
arguments differently:

- Launch files (`bringup.launch.py`, `mujoco_bridge.launch.py`) - `ros2 launch <pkg> <file> arg:=value`
- Nodes run with `ros2 run` (`pose_commander`, `episode_manager`) - `ros2 run <pkg> <exe> --ros-args -p param:=value`, one `-p` per parameter

Both end up setting the same kind of thing (a ROS2 parameter on the node); it's just that a
launch file wraps `ros2 run` and exposes its own argument names via `DeclareLaunchArgument`,
which don't have to (and here, mostly don't) match the underlying parameter names 1:1.

#### `bringup.launch.py` arguments

| Argument | Default | Purpose |
|---|---|---|
| `robot_model` | `pro` | `pro` or `dual` - which Tiago variant's URDF/OpenSoT stack to bring up. |

#### `mujoco_bridge.launch.py` arguments

| Argument | Default | Purpose |
|---|---|---|
| `mujoco_xml_path` | `/home/forest_ws/robots/pal_tiago_pro/xmls/scene_tiago_pro.xml` | Path to the MuJoCo scene XML to load. |
| `viewer` | `true` | Show the MuJoCo viewer window. Set `false` for faster headless data collection - RViz/`cartesian_interface_node` do nothing for the scripted paths below either way, so the viewer is the main thing worth turning off. |
| `fps` | `90.0` | Physics/render loop rate (Hz). Must be `>=` `episode_log_fps`, or recording silently gets capped at `fps` instead (see `mujoco_sim_node.py`'s startup warning). |
| `episode_log_fps` | `90.0` | How often a step is appended to the episode log (Hz) - matches the real Tiago controller's rate. |
| `command_topic` | `/opensot/joint_states` | Topic the sim reads commanded joint positions from. |
| `joint_states_topic` | `/joint_states` | Topic the sim publishes its own joint state on. |
| `gripper_speed` | `0.8` | Gripper open/close ramp speed (rad/s). |
| `target_object_joint` | `cube_freejoint` | MuJoCo freejoint name of the object tracked/randomized for episodes. |
| `episode_log_dir` | `/tmp/tiago_pro_episodes` | Directory dataset files are saved into (matches the docker-compose `./data` mount - avoid changing unless it's mounted somewhere else too). |
| `dataset_name` | `dataset` | Base filename (no `.h5`) for this run's HDF5 file - give different runs different names to keep them in separate files instead of always appending `demo_N` groups to the same one, e.g. `dataset_name:=basket_v2` → `/tmp/tiago_pro_episodes/basket_v2.h5`. |
| `save_failed_episodes` | `false` | Keep failed episodes in the log too, instead of discarding them. |
| `object_x_range` | `[0.55, 0.65]` | Table-frame x range (meters) the object is respawned into on reset. |
| `object_y_range` | `[-0.20, -0.10]` | Table-frame y range (meters) the object is respawned into on reset. |
| `base_frame` | `opensot/base_link` | Frame the published target-object pose is expressed in. |

Example overriding several at once:

```bash
ros2 launch tiago_pro_mujoco_bridge mujoco_bridge.launch.py dataset_name:=basket_v2 viewer:=false episode_log_fps:=30 save_failed_episodes:=true
```

### 3. Run a single pick-and-place plan (tuning the plan)

```bash
ros2 run tiago_control_node pose_commander
```

Runs the waypoint list defined in `tasks/pick_place_basket.py`'s `PLAN` once, then holds the final pose - that file is plain waypoint/geometry data (no ROS2), so it's the one to read/edit to change what the robot does; `pose_commander.py` itself is just the generic engine that runs whatever `PLAN` it's given. Useful for iterating on the plan itself.

`pose_commander`'s own parameters (`ros2 run tiago_control_node pose_commander --ros-args -p <param>:=<value>`):

| Parameter | Default | Purpose |
|---|---|---|
| `base_frame` | `opensot/base_link` | Frame `PoseStamped` targets are stamped with - must match what `tiago_pro_opensot_node` expects the Cartesian task's base frame to be. |
| `publish_rate` | `30.0` | How often (Hz) the current target is re-published to `/cartesian_interface/{side}/target_pose` while a waypoint is held. |

To reset the sim (robot back to home, object respawned at a new random table position) between attempts without going through the full episode loop below:

```bash
ros2 service call /mujoco_bridge/end_episode std_srvs/srv/SetBool "{data: false}"
```

(the `data` field is just a success flag it logs - irrelevant for this manual use, `false` is fine.)

### 4. Collect a dataset automatically

```bash
ros2 run tiago_control_node episode_manager --ros-args -p num_episodes:=50
```

Loops: reset the episode (robot home, object randomized on the table) → run the pick-and-place plan → judge success by whether the object landed in the basket → repeat. A failed episode (an exception during the plan, a timed-out reset, ...) is logged and skipped rather than stopping the run. Only successful episodes are saved (`save_failed_episodes:=true` on `mujoco_bridge.launch.py` to keep failures too, e.g. for debugging).

`episode_manager` subclasses `pose_commander`'s node, so it takes `base_frame`/`publish_rate` too (see above), plus its own:

| Parameter | Default | Purpose |
|---|---|---|
| `num_episodes` | `10` | How many reset → plan → judge cycles to run before stopping. |

Each successful episode lands as its own group in `<episode_log_dir>/<dataset_name>.h5` (default `/tmp/tiago_pro_episodes/dataset.h5`).
dataset.h5
└── data
    ├── demo_0
    │   ├── actions                # flat (T, 16): [right_pos(3), right_quat_xyzw(4),
    │   │                          #   left_pos(3), left_quat_xyzw(4), right_gripper(1), left_gripper(1)]
    │   └── obs/
    │       ├── eef_{left,right}_pose   # commanded pose (x,y,z,qx,qy,qz,qw), same source as actions
    │       ├── eef_{left,right}_pos/_rot  # MuJoCo ground truth (extra, sim-only, (w,x,y,z) quat)
    │       ├── joint_pos_opensot, joint_pos_real, joint_vel_real
    │       └── target_object_pose      # (x,y,z,qx,qy,qz,qw)
    ├── demo_1
    │   └── ...
    └── ...
```

`obs/eef_{side}_pose` and the `actions` pose columns are both sourced from `/cartesian_interface/{side}/target_pose` (the commanded pose) - not MuJoCo's physics ground truth - to match how the real-robot pipeline defines this key.

With the dev container (`make dev`), `/tmp/tiago_pro_episodes/` is bind-mounted to `data/` in this repo (git-ignored) - no extra setup needed, `data/<dataset_name>.h5` just shows up there once episodes start saving.

### 5. Test a trained policy in sim

```bash
# Let the policy actually drive the arm:
ros2 topic pub /streamdeck/teleop_mode std_msgs/msg/String "data: replay" --once

# Hand control back to the RViz markers/teleop:
ros2 topic pub /streamdeck/teleop_mode std_msgs/msg/String "data: rviz" --once
```

This lets you start the inference script, sanity-check what it wants to do first (it publishes
its own RViz markers - see the script), and only arm it when you're satisfied, rather than it
grabbing control the instant it starts.

## Coming Soon

Documentation for running the controller on the real TIAGo robot will be added soon.
