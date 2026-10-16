# syntax=docker/dockerfile:1
# =============================================================================
# Go Aero eVTOL — Virtual Jetson Orin NX (ZED Box Mini) SIL Container
# =============================================================================
# Multi-architecture image (`linux/arm64` on Apple Silicon Macs — matching the
# physical Jetson Orin NX `aarch64` ISA — and `linux/amd64` on AWS EC2 GPU
# instances) that reproduces the exact companion-computer software stack:
#   - Ubuntu 22.04 (Jammy) + ROS 2 Humble
#   - Fast DDS (`rmw_fastrtps_cpp`)
#   - MAVROS (`ros-humble-mavros`, `ros-humble-mavros-extras`) + GeographicLib
#   - TF2 (`tf2_ros`, `tf2_sensor_msgs`) + `sensor_msgs_py`
#   - Unmodified `VFS-Go-Aero/visual-obstacle-detection`
#     (`feature/visual_obstacle_detection_2` @ `cba2f42`)
#   - Simulation helpers (`jetson_sil_launch.py`, `synthetic_zed_publisher.py`,
#     `latency_relay.py`)
#
# Base image overrides (`--build-arg BASE_IMAGE=...`):
#   1. Default (multi-arch CPU/SIL on macOS arm64 & Linux amd64):
#        --build-arg BASE_IMAGE=ros:humble-ros-base-jammy
#   2. AWS EC2 GPU with ZED SDK 5.1 (for ZED IPC sim streaming):
#        --build-arg BASE_IMAGE=stereolabs/zed:5.1-devel-cuda12.6-ubuntu22.04
#   3. Physical Jetson Orin NX (JetPack 6.1 / L4T R36.4.x):
#        --build-arg BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.4.0
# =============================================================================

ARG BASE_IMAGE=ros:humble-ros-base-jammy
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_DOMAIN_ID=0 \
    PYTHONUNBUFFERED=1

# Ensure ROS 2 Humble apt repository is present even when building from a
# non-ROS base image (such as stereolabs/zed or nvcr.io/nvidia/l4t-jetpack).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        gnupg2 \
        lsb-release \
        locales \
    && locale-gen en_US en_US.UTF-8 \
    && if [ ! -f /etc/apt/sources.list.d/ros2.list ] \
       && [ ! -f /etc/apt/sources.list.d/ros2-latest.list ] \
       && [ ! -f /etc/apt/sources.list.d/ros2.sources ] \
       && ! grep -rq "packages.ros.org/ros2/ubuntu" /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null; then \
        curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg && \
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
            > /etc/apt/sources.list.d/ros2.list; \
    fi \
    && apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        bzip2 \
        chrony \
        geographiclib-tools \
        git \
        iproute2 \
        iputils-ping \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-pip \
        python3-pytest \
        python3-rosdep \
        ros-humble-geometry-msgs \
        ros-humble-mavros \
        ros-humble-mavros-extras \
        ros-humble-mavros-msgs \
        ros-humble-nav-msgs \
        ros-humble-rmw-fastrtps-cpp \
        ros-humble-ros-base \
        ros-humble-sensor-msgs \
        ros-humble-sensor-msgs-py \
        ros-humble-tf2-ros \
        ros-humble-tf2-sensor-msgs \
        ros-humble-visualization-msgs \
        wget \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets required by MAVROS on startup.
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Install pymavlink for optional direct PX4 OBSTACLE_DISTANCE telemetry in latency_relay.py.
RUN pip3 install --no-cache-dir pymavlink

# Clone and build the unmodified VFS-Go-Aero/visual-obstacle-detection workspace.
ARG VOD_REPO=https://github.com/VFS-Go-Aero/visual-obstacle-detection.git
ARG VOD_BRANCH=feature/visual_obstacle_detection_2
ARG VOD_COMMIT=cba2f42

WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src /ros2_ws/sim_scripts \
    && git clone --branch "${VOD_BRANCH}" "${VOD_REPO}" /ros2_ws/src/visual-obstacle-detection \
    && if [ -n "${VOD_COMMIT}" ]; then \
        git -C /ros2_ws/src/visual-obstacle-detection checkout "${VOD_COMMIT}"; \
    fi \
    && source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install --packages-select visual_obstacle_detection \
    && mkdir -p /ros2_ws/install/visual_obstacle_detection/share/visual_obstacle_detection \
    && if [ ! -e /ros2_ws/install/visual_obstacle_detection/share/visual_obstacle_detection/excluded_boxes.xml ]; then \
        if [ -f /ros2_ws/src/visual-obstacle-detection/visual_obstacle_detection/excluded_boxes.xml ]; then \
            ln -sfn /ros2_ws/src/visual-obstacle-detection/visual_obstacle_detection/excluded_boxes.xml \
                /ros2_ws/install/visual_obstacle_detection/share/visual_obstacle_detection/excluded_boxes.xml; \
        elif [ -f /ros2_ws/src/visual-obstacle-detection/excluded_boxes.xml ]; then \
            ln -sfn /ros2_ws/src/visual-obstacle-detection/excluded_boxes.xml \
                /ros2_ws/install/visual_obstacle_detection/share/visual_obstacle_detection/excluded_boxes.xml; \
        fi; \
    fi

# Copy SIL launch, synthetic ZED point-cloud publisher, and latency relay scripts.
COPY simulation/jetson_sil_launch.py \
     simulation/synthetic_zed_publisher.py \
     simulation/latency_relay.py \
     /ros2_ws/sim_scripts/

COPY docker/jetson-sil/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /ros2_ws/sim_scripts/*.py

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "/ros2_ws/sim_scripts/jetson_sil_launch.py", "use_sim_time:=false", "use_synthetic_zed:=true"]
