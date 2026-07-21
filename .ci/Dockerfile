FROM dtotsila/tiago_wbc_ros2:latest as prebuilt
FROM prebuilt AS dev
WORKDIR /home/forest_ws/src/
# Force the dev shell to source your compiled workspace automatically
RUN echo 'if [ -f "/home/forest_ws/install/setup.bash" ]; then source "/home/forest_ws/install/setup.bash"; fi' >> ~/.bashrc
CMD ["/bin/bash"]

FROM prebuilt AS dep
# Copy source code into the image
COPY ./src /home/forest_ws/src

# Build ROS 2 Package
WORKDIR /home/forest_ws
RUN source /opt/ros/humble/setup.bash && source /home/forest_ws/setup.bash && \
    colcon build --packages-select tiago_control_node --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log

# Runtime Setup
ENV XDG_RUNTIME_DIR=/tmp/runtime-root
RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root

RUN echo 'source /home/.base/bin/activate' >> ~/.bashrc && \
    echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source /home/forest_ws/install/setup.bash' >> ~/.bashrc && \
    echo 'source /home/ros2_ws/install/setup.bash' >> ~/.bashrc

COPY ./.ci/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Launch the package using the ROS 2 resource index
CMD ["ros2", "launch", "tiago_control_node", "bringup.launch.py"]