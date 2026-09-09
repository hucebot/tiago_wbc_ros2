# Deployment image: the heavy dependency stack (ROS + Pinocchio + xbot2_interface
# + OpenSoT + QP solvers + PAL description packages) lives in the base image
# below; this file only layers in `tiago_control_node`.
#
# TODO(P4): the base is an unpinned personal-namespace image. Pin it by digest
# under the hucebot org, and keep it regenerable from .ci/Dockerfile.base
# (best-effort recipe, not yet CI-verified).
FROM dtotsila/tiago_wbc_ros2 AS dep

# Copy source code into the image
COPY ./src /home/forest_ws/src

# Build ROS 2 Package
WORKDIR /home/forest_ws
RUN source /opt/ros/humble/setup.bash && source /home/forest_ws/setup.bash && \
    colcon build --parallel-workers 2 --packages-select tiago_control_node --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log

# Runtime Setup
ENV XDG_RUNTIME_DIR=/tmp/runtime-root
RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root

# Bake the RMW / domain so `docker exec` shells (which skip the entrypoint and
# compose env) still talk to the right middleware. compose/-e still override.
# CYCLONEDDS_URI stays entrypoint-only (it depends on DDS_ENV) -- use `make shell`.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=2

RUN echo 'source /home/.base/bin/activate' >> ~/.bashrc && \
    echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source /home/forest_ws/install/setup.bash' >> ~/.bashrc && \
    echo 'source /home/ros2_ws/install/setup.bash' >> ~/.bashrc

COPY ./.ci/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Launch the package using the ROS 2 resource index
CMD ["ros2", "launch", "tiago_control_node", "bringup.launch.py"]
