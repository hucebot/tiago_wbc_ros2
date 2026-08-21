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

To run pose commander where you can have a point plan
```bash
ros2 run tiago_control_node pose_commander
```


## Coming Soon

Documentation for running the controller on the real TIAGo robot will be added soon.
