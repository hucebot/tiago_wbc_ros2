#!/bin/bash

# Source the base ROS 2 setup
source "/opt/ros/humble/setup.bash"

# Source your virtual environment
source "/home/.base/bin/activate"

# Source your custom workspaces
if [ -f "/home/forest_ws/setup.bash" ]; then
    source "/home/forest_ws/setup.bash"
fi

if [ -f "/home/forest_ws/install/setup.bash" ]; then
    source "/home/forest_ws/install/setup.bash"
fi

if [ -f "/home/ros2_ws/install/local_setup.bash" ]; then
    source "/home/ros2_ws/install/local_setup.bash"
fi

# Hardcode the crucial paths
export PYTHONPATH="/home/forest_ws/install/lib/python3.10/site-packages:/usr/lib/python3/dist-packages:$PYTHONPATH"

export LD_LIBRARY_PATH="/home/forest_ws/install/lib/roboptim-core:/home/forest_ws/install/lib:$LD_LIBRARY_PATH"


# CycloneDDS auto-configuration. Single owner of CYCLONEDDS_URI: picks
# /home/configs/cyclonedds_<DDS_ENV>.xml (mounted from ./configs) if it exists.
DDS_ENV="${DDS_ENV:-local}"
USE_SIM="${USE_SIM:-false}"

if [ "$DDS_ENV" = "robot" ] && [ "$USE_SIM" = "true" ]; then
    echo "[Entrypoint] ERROR: USE_SIM=true is not allowed with DDS_ENV=robot."
    echo "[Entrypoint] MuJoCo must run with DDS_ENV=local."
    exit 1
fi
DDS_PROFILE="/home/configs/cyclonedds_${DDS_ENV}.xml"
if [ -f "$DDS_PROFILE" ]; then
    export CYCLONEDDS_URI="file://${DDS_PROFILE}"
    echo "[Entrypoint] CycloneDDS profile: ${DDS_PROFILE} (DDS_ENV=${DDS_ENV})"
else
    echo "[Entrypoint] No CycloneDDS profile for DDS_ENV='${DDS_ENV}' (${DDS_PROFILE} missing); using DDS defaults"
fi

DDS_ENV=robot USE_SIM=true ...

# Execute the command passed to the container
exec "$@"
