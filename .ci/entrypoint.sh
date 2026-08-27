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


# CycloneDDS Auto-Configuration
# Reads the DDS_ENV variable from your docker-compose/.env and automatically applies the correct XML profile!
if [ "$DDS_ENV" = "local" ]; then
    export CYCLONEDDS_URI="file:///home/configs/cyclonedds_local.xml"
    echo "🟢 [Entrypoint] Network Mode: LOCAL (Loopback Only)"
elif [ "$DDS_ENV" = "robot" ]; then
    export CYCLONEDDS_URI="file:///home/configs/cyclonedds_robot.xml"
    echo "🔴 [Entrypoint] Network Mode: ROBOT (Hardware Connected)"
else
    echo "🟡 [Entrypoint] Network Mode: DEFAULT (No DDS profile applied)"
fi

# Execute the command passed to the container
exec "$@"
