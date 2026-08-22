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

```bash
cd /home/forest_ws
colcon build --symlink-install --packages-select tiago_control_node tiago_pro_mujoco_bridge
source install/setup.bash
source setup.bash
ros2 launch tiago_control_node bringup.launch.py robot_model:=pro use_mujoco_sim:=true # Launches RViz2 and Mujoco
```

Launch arguments worth knowing about:

| Argument | Default | Purpose |
|---|---|---|
| `mujoco_xml_path` | `robots/pal_tiago_pro/xmls/scene_tiago_pro.xml` | Scene to load |
| `episode_log_path` | `/tmp/tiago_pro_episodes/dataset.h5` | Single HDF5 file episodes are appended to (see below) |
| `mujoco_viewer` | `true` | Show the MuJoCo viewer window. Set `false` for faster headless data collection - RViz/`cartesian_interface_node` do nothing for the scripted paths below either way, so the viewer is the main thing worth turning off. |

Before running anything below: in RViz, **uncheck "Enable Task"** on both interactive markers (right-click each marker → Enable Task). `cartesian_interface_node` republishes whatever pose the marker holds at 100Hz whenever it's enabled, which fights any scripted plan on the same topic.

### 3. Run a single pick-and-place plan (tuning the plan)

```bash
ros2 run tiago_control_node pose_commander
```

Runs the waypoint list defined in `pose_commander.py`'s `PLAN` once, then holds the final pose. Useful for iterating on the plan itself. To reset the sim (robot back to home, object respawned at a new random table position) between attempts without going through the full episode loop below:

```bash
ros2 service call /mujoco_bridge/end_episode std_srvs/srv/SetBool "{data: false}"
```

(the `data` field is just a success flag it logs - irrelevant for this manual use, `false` is fine.)

### 4. Collect a dataset automatically

```bash
ros2 run tiago_control_node episode_manager --ros-args -p num_episodes:=50
```

Loops: reset the episode (robot home, object randomized on the table) → run the pick-and-place plan → judge success by whether the object landed in the basket → repeat. Only successful episodes are saved (`save_failed_episodes:=true` on `mujoco_bridge_node` to keep failures too, e.g. for debugging).

Each successful episode lands as its own group in the single HDF5 file at `episode_log_path`. Schema matches the [Dont-Be-Brave](../Dont-Be-Brave) `timid.tasks.Tiago` loader exactly (see its `tasks/tiago.py` and `config/tiago_config.py`):

```
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

With the dev container (`make dev`), this file is bind-mounted to `data/dataset.h5` in this repo (git-ignored) - no extra setup needed, it just shows up there once episodes start saving.

## Coming Soon

Documentation for running the controller on the real TIAGo robot will be added soon.
