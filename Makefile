.PHONY: dev deploy build push down xhost build-deploy push-deploy

# Extract variables from .env file
include .env
export

# FIX: Add a default fallback value for TAG if it is empty!
TAG ?= latest

IMAGE_NAME = tiago_wbc_ros2:$(TAG)

# Grants Docker access to your local X11 display for RViz
xhost:
	@xhost +local:docker > /dev/null 2>&1 || true

# ---------------------------------------------------------
# DEVELOPMENT (Builds dev image, mounts local code)
# ---------------------------------------------------------
dev: xhost
	docker compose up opensot_dev -d --build
	docker exec -it opensot_dev_instance bash

build:
	docker compose build opensot_dev

# Note: Pushing a 'dev' target usually requires an 'image: ...' tag in the yaml.
# If you actually use this, make sure opensot_dev has an image tag in docker-compose.yml!
push:
	docker compose push opensot_dev

# ---------------------------------------------------------
# DEPLOYMENT (Immutable production image, no code mounts)
# ---------------------------------------------------------
deploy: xhost
	docker compose up opensot_deploy

# Builds the production image (targets 'dep' stage in Dockerfile)
build-deploy:
	@echo "Building production image: $(IMAGE_NAME)"
	docker build --target dep -t $(IMAGE_NAME) -f .ci/Dockerfile .

# Pushes the production image to GitLab
push-deploy:
	@echo "Pushing production image: $(IMAGE_NAME)"
	docker push $(IMAGE_NAME)

# ---------------------------------------------------------
# UTILS
# ---------------------------------------------------------

# Attach to the running development container
attach:
	@docker attach opensot_dev_instance

# Stops and removes all containers and networks
down:
	docker compose down
