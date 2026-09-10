.PHONY: tiago tiago-pro dev shell deploy down xhost build-deploy push-deploy

# Extract variables from .env file
include .env
export

TAG ?= latest
IMAGE_NAME = tiago_wbc_ros2:$(TAG)

# Image used for the dev shell. Defaults to the prebuilt base so `make dev`
# works straight after a clone; override with `make dev DEV_IMAGE=...` to use a
# locally built image instead.
DEV_IMAGE ?= dtotsila/tiago_wbc_ros2
DDS_ENV ?= local
ROBOT_MODEL ?= pro

# Grants Docker access to your local X11 display for RViz
xhost:
	@xhost +local:docker > /dev/null 2>&1 || true

# ---------------------------------------------------------
# DEVELOPMENT: prebuilt image + live-mounted ./src, no compose, no image build.
# Inside the shell:
#   colcon build --packages-select tiago_control_node && source install/setup.bash
#   ros2 launch tiago_control_node bringup.launch.py robot_model:=$ROBOT_MODEL
# ---------------------------------------------------------
dev: xhost
	docker run --rm -it \
		--name opensot_dev_instance \
		--network host --ipc host --pid host --privileged \
		-e DISPLAY=$(DISPLAY) \
		-e ROS_DOMAIN_ID=2 \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e DDS_ENV=$(DDS_ENV) \
		-e ROBOT_MODEL=$(ROBOT_MODEL) \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(CURDIR)/configs:/home/configs:ro \
		-v $(CURDIR)/src:/home/forest_ws/src \
		$(DEV_IMAGE) bash

# ---------------------------------------------------------
# DEPLOYMENT (Immutable production image, no code mounts)
# ---------------------------------------------------------

# Run the demo for the original TIAGo (dual arms)
tiago: xhost
	ROBOT_MODEL=dual docker compose up opensot_deploy

# Run the demo for TIAGo Pro
tiago-pro: xhost
	ROBOT_MODEL=pro docker compose up opensot_deploy

# # Physical robots
# tiago: xhost
# 	DDS_ENV=robot ROBOT_MODEL=dual USE_SIM=false docker compose up opensot_deploy

# tiago-pro: xhost
# 	DDS_ENV=robot ROBOT_MODEL=pro USE_SIM=false docker compose up opensot_deploy

# MuJoCo simulation
mj-tiago: xhost
	DDS_ENV=local ROBOT_MODEL=dual USE_SIM=true docker compose up opensot_deploy

mj-tiago-pro: xhost
	DDS_ENV=local ROBOT_MODEL=pro USE_SIM=true docker compose up opensot_deploy

# Local stack without a robot backend
local-tiago: xhost
	DDS_ENV=local ROBOT_MODEL=dual USE_SIM=false docker compose up opensot_deploy

local-tiago-pro: xhost
	DDS_ENV=local ROBOT_MODEL=pro USE_SIM=false docker compose up opensot_deploy


deploy: xhost
	docker compose up opensot_deploy

build-deploy:
	@echo "Building production image: $(IMAGE_NAME)"
	docker build --target dep -t $(IMAGE_NAME) -f .ci/Dockerfile .

# Pushes the production image to the registry
push-deploy:
	@echo "Pushing production image: $(IMAGE_NAME)"
	docker push $(IMAGE_NAME)

# ---------------------------------------------------------
# UTILS
# ---------------------------------------------------------

# Extra shell in the running container. `docker exec` skips the image entrypoint
# (so no CYCLONEDDS_URI, no sourced workspaces) -- re-run it here so the shell
# has the exact same ROS env as the launched process.
#   make shell                              -> deploy container
#   make shell SHELL_CONTAINER=opensot_dev_instance
SHELL_CONTAINER ?= opensot_deploy_instance
shell:
	docker exec -it $(SHELL_CONTAINER) /entrypoint.sh bash

# Stops and removes the deploy container and its network
down:
	docker compose down
