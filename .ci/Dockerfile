FROM osrf/ros:humble-desktop AS base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="Europe/Paris"
ENV HHCM_FOREST_CLONE_DEFAULT_PROTO=https
ENV PYTHONUNBUFFERED=1

# System & ROS Build Dependencies
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    terminator gedit locate cmake-curses-gui python3-pip python3-venv \
    liburdfdom-dev ros-humble-moveit-core libxcb-cursor0 gdb \
    ros-humble-xacro ros-humble-joint-state-publisher-gui \
    libopenblas-dev ros-humble-rosidl-generator-dds-idl \
    ros-humble-ament-cmake-auto ros-humble-pal-statistics \
    ros-humble-controller-interface ros-humble-hardware-interface \
    libqhull-dev nlohmann-json3-dev coinor-libipopt-dev \
    ros-humble-realsense2-camera ros-humble-realsense2-description \
    ros-humble-rmw-cyclonedds-cpp \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Setup Virtual Env
WORKDIR /home
RUN python3 -m venv /home/.base && \
    echo "source /home/.base/bin/activate" >> ~/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Forest & Matlogger
WORKDIR /home/forest_ws
RUN source /home/.base/bin/activate && \
    pip install --no-cache-dir --upgrade ttictoc "setuptools<70.0.0" hhcm-forest mujoco "numpy<2.0.0" jinja2 typeguard h5py viser && \
    forest init && \
    forest add-recipes git@github.com:advrhumanoids/multidof_recipes.git --tag ros2 && \
    forest grow matlogger2 --verbose --jobs $(nproc) --pwd user

# Third-party Builds (Coal, Pinocchio, xbot2, Solvers)
# COAL
WORKDIR /home/forest_ws/src
RUN git clone --depth 1 -b devel https://github.com/coal-library/coal.git && \
    cd coal && git submodule update --init && \
    mkdir -p /home/forest_ws/build/coal && cd /home/forest_ws/build/coal && \
    source /opt/ros/humble/setup.bash && source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release -DBUILD_PYTHON_INTERFACE=OFF -DCOAL_HAS_QHULL=ON ../../src/coal && \
    make -j2 && make install && \
    rm -rf /home/forest_ws/build/coal /home/forest_ws/src/coal

# PINOCCHIO
WORKDIR /home/forest_ws/src
RUN git clone --depth 1 -b v3.8.0 https://github.com/stack-of-tasks/pinocchio.git && \
    cd pinocchio && git submodule update --init && \
    mkdir -p /home/forest_ws/build/pinocchio && cd /home/forest_ws/build/pinocchio && \
    source /opt/ros/humble/setup.bash && source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release -DBUILD_WITH_URDF_SUPPORT=ON -DBUILD_WITH_COLLISION_SUPPORT=OFF -DBUILD_TESTING=FALSE -DBUILD_PYTHON_INTERFACE=OFF ../../src/pinocchio && \
    make -j2 && make install && \
    rm -rf /home/forest_ws/build/pinocchio /home/forest_ws/src/pinocchio

# xbot2_interface
WORKDIR /home/forest_ws/src
RUN git clone --depth 1 -b devel https://github.com/hucebot/xbot2_interface.git && \
    mkdir -p /home/forest_ws/build/xbot2_interface && cd /home/forest_ws/build/xbot2_interface && \
    source /opt/ros/humble/setup.bash && source /home/forest_ws/setup.bash && \
    cmake -DXBOT2_IFC_BUILD_TESTS=ON -DXBOT2_IFC_BUILD_ROS=OFF -DXBOT2_IFC_BUILD_ROS2=OFF -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/xbot2_interface && \
    make -j2 && make install && \
    rm -rf /home/forest_ws/build/xbot2_interface /home/forest_ws/src/xbot2_interface

# osqp
WORKDIR /home/forest_ws/src
RUN git clone https://github.com/oxfordcontrol/osqp.git && \
    cd osqp && git checkout 0b34f2ef5c5eec314e7945762e1c8167e937afbd && git submodule update --init && \
    mkdir -p /home/forest_ws/build/osqp && cd /home/forest_ws/build/osqp && \
    source /opt/ros/humble/setup.bash && source /home/forest_ws/setup.bash && \
    cmake -DDLONG=OFF -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/osqp && \
    make -j2 && make install && \
    rm -rf /home/forest_ws/build/osqp /home/forest_ws/src/osqp

# proxQP
WORKDIR /home/forest_ws/src
RUN git clone https://github.com/Simple-Robotics/proxsuite.git && \
    cd proxsuite && git checkout 9f58ebd7bf42fb364de995d25679dc3f675dee7f && git submodule update --init && \
    mkdir -p /home/forest_ws/build/proxsuite && cd /home/forest_ws/build/proxsuite && \
    source /opt/ros/humble/setup.bash && source /home/forest_ws/setup.bash && \
    cmake -DBUILD_WITH_VECTORIZATION_SUPPORT=OFF -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/proxsuite && \
    make -j2 && make install && \
    rm -rf /home/forest_ws/build/proxsuite /home/forest_ws/src/proxsuite

# qpSWIFT
WORKDIR /home/forest_ws/src
RUN git clone --depth 1 https://github.com/qpSWIFT/qpSWIFT.git && \
    mkdir -p /home/forest_ws/build/qpSWIFT && cd /home/forest_ws/build/qpSWIFT && \
    source /opt/ros/humble/setup.bash && source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/qpSWIFT && \
    make -j2 && make install && \
    rm -rf /home/forest_ws/build/qpSWIFT /home/forest_ws/src/qpSWIFT

# roboptim (NO --depth 1 ALLOWED! The CMake system needs full git history for versioning)
WORKDIR /home/forest_ws/src
RUN git clone https://github.com/hucebot/roboptim-core.git && cd roboptim-core && git submodule update --init && \
    mkdir -p /home/forest_ws/build/roboptim-core && cd /home/forest_ws/build/roboptim-core && \
    source /opt/ros/humble/setup.bash && source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install ../../src/roboptim-core && \
    make -j2 && make install && \
    rm -rf /home/forest_ws/build/roboptim-core /home/forest_ws/src/roboptim-core

RUN git clone https://github.com/hucebot/roboptim-core-plugin-ipopt.git && cd roboptim-core-plugin-ipopt && git submodule update --init && \
    mkdir -p /home/forest_ws/build/roboptim-core-plugin-ipopt && cd /home/forest_ws/build/roboptim-core-plugin-ipopt && \
    source /opt/ros/humble/setup.bash && source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DDISABLE_TESTS=ON ../../src/roboptim-core-plugin-ipopt && \
    make -j2 && make install && \
    rm -rf /home/forest_ws/build/roboptim-core-plugin-ipopt /home/forest_ws/src/roboptim-core-plugin-ipopt

RUN git clone https://github.com/hucebot/roboptim-capsule.git && cd roboptim-capsule && git submodule update --init && \
    mkdir -p /home/forest_ws/build/roboptim-capsule && cd /home/forest_ws/build/roboptim-capsule && \
    source /opt/ros/humble/setup.bash && source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install ../../src/roboptim-capsule && \
    make -j2 && make install && \
    rm -rf /home/forest_ws/build/roboptim-capsule /home/forest_ws/src/roboptim-capsule

RUN git clone https://github.com/hucebot/robot_capsule_generator.git && \
    mkdir -p /home/forest_ws/build/robot_capsule_generator && cd /home/forest_ws/build/robot_capsule_generator && \
    source /opt/ros/humble/setup.bash && source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install ../../src/robot_capsule_generator && \
    make -j2 && make install && \
    rm -rf /home/forest_ws/build/robot_capsule_generator /home/forest_ws/src/robot_capsule_generator

RUN echo "export LD_LIBRARY_PATH=/home/forest_ws/install/lib/roboptim-core/:$LD_LIBRARY_PATH" >> ~/.bashrc

# mujoco
WORKDIR /home/forest_ws/src
RUN git clone --depth 1 https://github.com/deepmind/mujoco.git && \
    mkdir -p /home/forest_ws/build/mujoco && cd /home/forest_ws/build/mujoco && \
    source /opt/ros/humble/setup.bash && source /home/forest_ws/setup.bash && \
    cmake -DMUJOCO_BUILD_EXAMPLES=OFF -DMUJOCO_BUILD_TESTS=OFF ../../src/mujoco && \
    make -j2 && make install && \
    rm -rf /home/forest_ws/build/mujoco /home/forest_ws/src/mujoco

# ROS2 Workspace (Description/Robot packages)
WORKDIR /home/ros2_ws/src
RUN git clone --depth 1 -b humble-devel https://github.com/pal-robotics/tiago_robot.git && \
    git clone --depth 1 -b humble-devel https://github.com/pal-robotics/tiago_dual_robot.git && \
    git clone --depth 1 -b humble-devel https://github.com/pal-robotics/omni_base_robot.git && \
    git clone --depth 1 -b humble-devel https://github.com/pal-robotics/pal_gripper.git && \
    git clone --depth 2 -b ros2 https://github.com/hucebot/tiago_dual_cartesio_config.git && \
    git clone --depth 1 -b humble-devel https://github.com/pal-robotics/tiago_pro_robot.git && \
    git clone --depth 1 -b humble-devel https://github.com/pal-robotics/pal_urdf_utils.git && \
    git clone --depth 1 -b humble-devel https://github.com/pal-robotics/pal_pro_gripper.git && \
    git clone --depth 1 -b humble-devel https://github.com/pal-robotics/pal_sea_arm.git && \
    git clone --depth 1 -b humble-devel https://github.com/pal-robotics/tiago_pro_head_robot.git

# Resolve any missing dependencies for these packages via rosdep
WORKDIR /home/ros2_ws
RUN apt-get update && \
    rosdep update && \
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble && \
    rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/humble/setup.bash && colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log

RUN echo "source /home/ros2_ws/install/local_setup.bash" >> ~/.bashrc

# Persistent Environment Setup
ENV PYTHONPATH="/home/.base/lib/python3.10/site-packages:/home/forest_ws/install/lib/python3.10/site-packages:/usr/lib/python3/dist-packages:${PYTHONPATH}"
ENV LD_LIBRARY_PATH="/home/forest_ws/install/lib:${LD_LIBRARY_PATH}"
ENV PATH="/home/.base/bin:${PATH}"
ENV CMAKE_PREFIX_PATH="/home/forest_ws/install:${CMAKE_PREFIX_PATH}"

# --- STAGE 2: Dev (For Local Mounting) ---
FROM base AS prebuilt
WORKDIR /home/forest_ws

# Copy OpenSoT code into the image
COPY ./external/OpenSoT /home/forest_ws/code/OpenSoT

# Build OpenSoT
WORKDIR /home/forest_ws/code/build
RUN source /opt/ros/humble/setup.bash && source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install \
          -DCMAKE_BUILD_TYPE:STRING=Release \
          -DCMAKE_PREFIX_PATH="/home/forest_ws/install;/opt/ros/humble" \
          ../OpenSoT && \
    make -j2 && make install && \
    rm -rf /home/forest_ws/code/build

# Environment for the Entrypoint
ENV PYTHONPATH="/root/.local/lib/python3.10/site-packages:/home/forest_ws/install/lib/python3.10/site-packages:${PYTHONPATH}"
ENV LD_LIBRARY_PATH="/home/forest_ws/install/lib:${LD_LIBRARY_PATH}"

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
    colcon build --parallel-workers 2 --packages-select tiago_control_node --cmake-args -DCMAKE_BUILD_TYPE=Release && \
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
