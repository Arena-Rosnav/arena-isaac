# --- Stage 1: Builder (Strict Python 3.11 + ROS Jazzy) ---
FROM osrf/ros:jazzy-desktop AS builder

# 1. Setup Deadsnakes for Python 3.11 (Jazzy is native 3.12)
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository ppa:deadsnakes/ppa -y && apt-get update

RUN apt-get install -y \
    build-essential cmake git curl \
    python3.11 python3.11-dev python3.11-venv \
    python3-colcon-common-extensions && rm -rf /var/lib/apt/lists/*

# 2. Create Venv and SET PATH (Forced Activation)
RUN python3.11 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"
ENV VIRTUAL_ENV="/opt/venv"

# 3. STRICT CHECK: Fail build if version is not 3.11
RUN python --version && \
    python -c "import sys; assert sys.version_info[:2] == (3, 11), 'ERROR: Python version is not 3.11!'"

WORKDIR /opt/msg_build

RUN mkdir src && git clone https://github.com/Spaarsh/arena-isaac -b arena5 src/isaacsim_msgs

# 4. Install build dependencies (Including NumPy for the C-extensions)
RUN . /opt/ros/jazzy/setup.sh && \
    pip install --upgrade pip && \
    pip install catkin_pkg empy==3.3.4 lark numpy

# 5. Build - Forcing the directory name to python3.11
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build \
    --packages-select isaacsim_msgs \
    --cmake-args \
    -DPYTHON_EXECUTABLE=/opt/venv/bin/python \
    -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.11.so.1.0 \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.11 \
    -DROSIDL_GENERATOR_PY_PYTHON_EXECUTABLE=/opt/venv/bin/python

# --- Stage 2: Final Isaac Sim Image ---
FROM nvcr.io/nvidia/isaac-sim:5.1.0

# 6. Switch to root to fix system libs and paths
USER root

# Install cattrs directly into Isaac's internal site-packages to ensure it persists
RUN /isaac-sim/python.sh -m pip install cattrs pyyaml

# Copy the libpython3.11 library from builder (missing in Isaac 5.1.0 base)
COPY --from=builder /usr/lib/x86_64-linux-gnu/libpython3.11.so.1.0 /usr/lib/x86_64-linux-gnu/
RUN ldconfig

# Copy the built messages
COPY --from=builder /opt/msg_build/install /opt/isaac_bridge_msgs

# 7. Persistent Environment Setup
# We write to /root/.bashrc because Isaac Sim's launch scripts often wipe 'ENV' variables
RUN echo 'export ROS_DISTRO=jazzy' >> /root/.bashrc && \
    echo 'export ACCEPT_EULA=Y' >> /root/.bashrc && \
    echo 'export PRIVACY_CONSENT=Y' >> /root/.bashrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> /root/.bashrc && \
    echo 'export AMENT_PREFIX_PATH=/opt/isaac_bridge_msgs/isaacsim_msgs:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy:/opt/arena_ws/install/arena_robots:$AMENT_PREFIX_PATH' >> /root/.bashrc && \
    echo 'export ROS_DOMAIN_ID=1' >> /root/.bashrc && \
    echo 'export PYTHONPATH=/opt/isaac_bridge_msgs/isaacsim_msgs/lib/python3.11/site-packages:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/python:/opt/arena_ws/src/Arena/:/opt/arena_ws/src/Arena/arena_simulation_setup/src/:$PYTHONPATH' >> /root/.bashrc && \
    echo 'export LD_LIBRARY_PATH=/opt/isaac_bridge_msgs/isaacsim_msgs/lib:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/rclpy/:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib/:$LD_LIBRARY_PATH' >> /root/.bashrc

# Set standard ENV as well for non-interactive shells
ENV ROS_DISTRO=jazzy \
    AMENT_PREFIX_PATH=/opt/isaac_bridge_msgs/isaacsim_msgs:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy \
    PYTHONPATH=/opt/isaac_bridge_msgs/isaacsim_msgs/lib/python3.11/site-packages:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/python \
    LD_LIBRARY_PATH=/opt/isaac_bridge_msgs/isaacsim_msgs/lib:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/rclpy/:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib/

WORKDIR /opt/arena_ws
ENTRYPOINT ["bash"]
