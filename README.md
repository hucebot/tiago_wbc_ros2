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

#### Excluding the left arm / torso (single-arm data collection)

`tiago_pro_opensot_node` (the node `bringup.launch.py robot_model:=pro` starts) reads two extra
booleans from `config/params.yaml` - useful when you only want the right arm active for a
demonstration and don't want the left arm or torso wandering into the dataset:

| Parameter | Default | Purpose |
|---|---|---|
| `disable_left_arm` | `false` | Deactivates the left-arm's Cartesian and manipulability tasks. |
| `disable_torso` | `false` | Excludes torso from the Cartesian tasks' active-joints mask. |

Both are read once at node startup (inside `setup_opensot_stack()`), not live - edit
`params.yaml` and relaunch `bringup.launch.py`; `ros2 param set` on an already-running node has
no effect.

**Neither flag is a hard lock.** `disable_left_arm` only deactivates `g_left`/`manip_left` - the
arm is still free to be pulled by whatever else touches it (Postural, collision avoidance), it
isn't pinned to a fixed joint position. `disable_torso` only excludes torso from the *Cartesian*
tasks' Jacobians - `manip_left`/`manip_right` (Manipulability tasks, not Cartesian) aren't
masked, so they can still move it, and `CollisionAvoidance` isn't a `Task` at all so it's never
masked either. If you need a true hard freeze at a fixed pose (e.g. for perfectly reproducible
episodes), zero the corresponding entries of `VelocityLimits` instead - see `TORSO_DQ_IDX` /
`ARM_LEFT_DQ_SLICE` in `tiago_pro_opensot_node.py`.

#### `mujoco_bridge.launch.py` arguments

| Argument | Default | Purpose |
|---|---|---|
| `mujoco_xml_path` | `/home/forest_ws/robots/pal_tiago_pro/xmls/scene_tiago_pro.xml` | Path to the MuJoCo scene XML to load. |
| `viewer` | `true` | Show the MuJoCo viewer window. Set `false` for faster headless data collection - RViz/`cartesian_interface_node` do nothing for the scripted paths below either way, so the viewer is the main thing worth turning off. |
| `fps` | `90.0` | Physics/render loop rate (Hz). Must be `>=` `episode_log_fps`, or recording silently gets capped at `fps` instead (see `mujoco_sim_node.py`'s startup warning). |
| `episode_log_fps` | tracks `fps` | REQUESTED recording rate (Hz) - defaults to whatever `fps` is set to (so `fps:=120` alone gives 120Hz collection, no need to set both), but can be set explicitly lower to log at a reduced rate. If physics stepping + ROS overhead take longer per iteration than `1/fps` allows, the real rate silently comes out lower than this. Don't trust this number for what a dataset was actually recorded at - every saved episode stamps the real, *measured* rate as its own `fps` attr instead (see the schema below). |
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

`PLAN`'s waypoints are splined into one continuous motion (position via a shape-preserving PCHIP spline, orientation via a rotation spline), not run as separate straight-line segments - a waypoint's `hold` is how many seconds after the previous one it's reached, not a pause after arriving. A waypoint with no `right`/`left` key (e.g. a gripper-only step) doesn't move that side at all; the spline holds its position flat across that time span instead of drifting toward whatever the next real waypoint is.

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
```
dataset.h5
└── data
    ├── demo_0                     # attrs: success, num_steps, attempt_index,
    │   │                          #   fps (MEASURED recording rate), requested_fps (episode_log_fps asked for - see above)
    │   ├── actions                # flat (T, 16): [right_pos(3), right_quat_xyzw(4),
    │   │                          #   left_pos(3), left_quat_xyzw(4), right_gripper(1), left_gripper(1)]
    │   └── obs/
    │       ├── eef_{left,right}_pose   # COMMANDED /cartesian_interface/{side}/target_pose (x,y,z,qx,qy,qz,qw) - same values as the actions pose columns
    │       ├── eef_{left,right}_pos/_rot  # MuJoCo's own ACHIEVED ground truth instead, (w,x,y,z) quat order (extra, sim-only, debug only - unused by training)
    │       ├── joint_pos_opensot, joint_pos_real, joint_vel_real
    │       └── target_object_pose      # (x,y,z,qx,qy,qz,qw)
    ├── demo_1
    │   └── ...
    └── ...
```

`obs/eef_{side}_pose` and the `actions` pose columns are the COMMANDED `/cartesian_interface/{side}/target_pose` value (via `get_log_entry()`), matching exactly what the real robot pipeline already logs there (it has no ground-truth "current achieved pose" signal to use instead, so it echoes the commanded value too). This used to be MuJoCo's own achieved ground truth instead, on the theory that sim could afford to be more accurate than real and it'd transfer with some tuning - it doesn't: replaying a recorded "achieved" pose as a new target reintroduces OpenSoT's own tracking lag on top of the lag that already happened once during collection, which `src/tools/tiago_replay.py` (open-loop action replay) surfaced concretely as the gripper closing on air - it fires on the original recording's schedule, but the replayed arm hasn't caught up to that row's target yet. A policy trained on the achieved-pose signal would hit the exact same lag at inference, for the same reason, so this needs to match the real pipeline's semantics rather than be more sim-accurate than necessary. MuJoCo's own achieved ground truth is still available for debugging as `obs/eef_{side}_pos/_rot` (e.g. to compare achieved vs. commanded tracking error) - see `mujoco_sim_node.py`'s `get_log_entry()` docstring.

**Datasets collected before this change keep the old (achieved-pose) semantics** - re-collect rather than mixing the two in one training run.

**No temporal downsampling right now.** `timid.tasks.tiago.Tiago` used to downsample training data toward a target `sampling_freq`, to avoid a policy learning to predict "no change" when consecutive steps are too close together. That's disabled for now (deliberately, not a bug) - check the measured `fps` on a freshly-collected dataset before assuming this still holds; it should now track `episode_log_fps` closely (a stack of fixes landed for a bug that used to cap it around ~25Hz regardless of configuration - see `mujoco_sim_node.py`'s main loop and `episode_recorder.py`'s `time.monotonic()` note), so per-step deltas may now be smaller than when this was last checked. Revisit downsampling if that turns out to matter.

With the dev container (`make dev`), `/tmp/tiago_pro_episodes/` is bind-mounted to `data/` in this repo (git-ignored) - no extra setup needed, `data/<dataset_name>.h5` just shows up there once episodes start saving.

### 5. Verify the dataset before training

Before handing a freshly-collected `.h5` file to training, run the replay/verification
script's whole-dataset check (`tiago_replay.py --check-all path/to/dataset.h5`) against it.
It runs a per-episode sanity check (NaN/Inf, step-to-step deltas, gripper-toggle freezing)
across every episode, plus three dataset-wide checks worth actually reading the output of:

- **fps consistency** - are all episodes really recorded at the same rate, and is it what
  you expect (see `episode_log_fps`'s note above on requested vs. measured).
- **Boundary contamination** - an anomalously large jump packed into an episode's first few
  steps, which would mean a stray frame or two of the *previous* episode leaked into this
  one's start (this is a closed bug for data collected after this repo's `save_and_clear()`
  fix - the check exists to confirm that, and to catch it again if it ever regresses).
- **Starting-configuration consistency** - every episode should start from the same
  post-reset home configuration; an outlier here means that episode's reset didn't actually
  settle before recording resumed.

For a frame-perfect visual spot-check of a specific episode (no controller in the loop, so
no tracking error or execution artifacts to second-guess - purely "is this what got
recorded"), the same script's `--ground-truth --sim` mode teleports MuJoCo's joint/object
state directly from the recorded observations each step.

**Current status:** `--check-all` and `--ground-truth --sim` above are still planned, not
built yet. What exists today, in `src/tools/tiago_replay.py`, is a narrower but more
diagnostic check - actions-only, open-loop replay:

```bash
python3 src/tools/tiago_replay.py --dataset /tmp/tiago_pro_episodes/dataset.h5 --demo 0
```

(bringup.launch.py and mujoco_bridge.launch.py must already be running). This resets the
robot home, restores the episode's recorded object start position, then drives OpenSoT/MuJoCo
using *only* the recorded `actions` stream, published open-loop at the recorded rate - no
per-step ground-truth teleport, and no joint-state seeding by default (`--teleport-joint-state`
is available as an off-by-default debug escape hatch) - because that's exactly the information
a deployed policy would have and no more. If this doesn't reproduce the recorded outcome,
no policy trained on this data can be expected to either: the problem is upstream of the
policy (the dataset, the collection timing, or the open-loop OpenSoT<->MuJoCo pipeline).
Prints SUCCESS/FAILURE plus how far the final object position diverges from what was
recorded.

### 6. Test a trained policy in sim

```bash
# Let the policy actually drive the arm:
ros2 topic pub /streamdeck/teleop_mode std_msgs/msg/String "data: replay" --once

# Hand control back to the RViz markers/teleop:
ros2 topic pub /streamdeck/teleop_mode std_msgs/msg/String "data: rviz" --once
```

This lets you start the inference script, sanity-check what it wants to do first (it publishes
its own RViz markers - see the script), and only arm it when you're satisfied, rather than it
grabbing control the instant it starts.

## Franka Panda (WBC ablation)

`panda_control_node`/`panda_mujoco_bridge` are a single-arm counterpart of
`tiago_control_node`/`tiago_pro_mujoco_bridge`, built to answer one specific question: is
TIAGo's whole-body-control complexity (dual-arm coupling, redundancy resolution,
collision-avoidance interaction, mobile base) the source of policy-performance problems, or
something else? Panda goes through the *same* OpenSoT-mediated Cartesian-control pipeline
and the *same* pick-cube-into-basket task (identical scene geometry - see
`robots/panda/xmls/scene_panda.xml`), just with a far simpler single-arm OpenSoT stack - not
a from-scratch simpler IK - so a working-vs-not-working comparison is actually evidence
about WBC, not about a different control approach entirely.

```bash
# Terminal 1 - OpenSoT ghost + solver (no tiago_dual_cartesio_config needed - the URDF is
# vendored directly in this repo, see robots/panda/urdf/panda.urdf's header comment):
ros2 launch panda_control_node bringup_panda.launch.py

# Terminal 2 - MuJoCo bridge + episode orchestrator (the orchestrator is
# tiago_pro_mujoco_bridge's own node, reused unmodified - it's already robot-agnostic):
ros2 launch panda_mujoco_bridge panda_bridge.launch.py

# Terminal 3 - pose_commander/episode_manager are REUSED from tiago_control_node directly,
# not copied - but their frame parameters default to TIAGo's names, so these two overrides
# are REQUIRED or TF lookups will silently fail against frames that don't exist in Panda's
# tree:
ros2 run panda_control_node episode_manager --ros-args \
  -p num_episodes:=50 \
  -p base_frame:=opensot/link0 \
  -p frames.right_gripper:=ee_panda
```

Everything else (HDF5 schema conventions, HZ-tuning levers, the `use_rviz`/`viewer` flags,
`fps`/`episode_log_fps` tracking) works the same way as the TIAGo MuJoCo workflow above -
see `panda_sim_node.py`'s docstring for the one schema difference (8-D single-arm actions
under an `eef_right_pose`/`right`-keyed convention, not TIAGo's 16-D dual-arm layout).

**Not yet built/verified**: this was authored without access to a compiled
`pyopensot`/`xbot2_interface` environment (only available in this project's dev container),
so the OpenSoT stack, grasp-offset tuning in `tasks/pick_place_basket_panda.py`, and the
gripper open/closed ctrl values haven't been run end-to-end yet - check the viewer on first
run and nudge those constants the same way TIAGo's own task-file comments already describe.

## Coming Soon

Documentation for running the controller on the real TIAGo robot will be added soon.
