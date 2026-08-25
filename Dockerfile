# LIO-SAM - ROS 2 Jazzy port. Self-contained image for this repository.
#
# Everything referenced lives inside this repo, so a standalone clone can just:
#   docker compose build && docker compose up -d

ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop
ARG ROS_DISTRO

SHELL ["/bin/bash", "-c"]

# Dependencies first, so editing source does not invalidate this layer.
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      ros-${ROS_DISTRO}-perception-pcl \
      ros-${ROS_DISTRO}-pcl-msgs \
      ros-${ROS_DISTRO}-vision-opencv \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-gtsam \
      ros-${ROS_DISTRO}-robot-localization \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# Optional tooling, also from README section 1: the offline converters in
# scripts/go1/ and scripts/urdf_extrinsics.py. Neither is needed to build or
# run the nodes - delete this layer if you only want the SLAM stack.
# pip needs --break-system-packages on noble (PEP 668); the fallback covers
# older pip that does not know the flag.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-urdfdom-py \
      python3-numpy \
      python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    && (python3 -m pip install --no-cache-dir --break-system-packages rosbags tqdm \
        || python3 -m pip install --no-cache-dir rosbags tqdm)

ENV WS=/root/ros2_ws
WORKDIR ${WS}

# Safety net. package.xml is the real dependency contract; the explicit list
# above only mirrors README section 1, and the two can drift.
#
# Only package.xml is copied at this point, so editing source does not re-run
# the (network-bound) rosdep update on every rebuild.
COPY package.xml src/lio_sam/package.xml
RUN apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

# Build context is this repo's root
COPY . src/lio_sam

# ---------------------------------------------------------------------------
# OPTIONAL: HesaiLidar ROS 2 driver.
#
# Only scripts/bunker/convert_bunker.py needs this - it drives the driver to
# decode raw /hesai/lidar_packets into point clouds. Nothing else in this repo
# uses it, so it is off by default: it pulls a large SDK submodule plus
# libboost-all-dev for a tool most users will never run.
#
# Uncomment to include it. Keep it ABOVE the colcon build below, so the driver
# is built in the same invocation.
#
# Afterwards the driver still has to be configured for packet replay
# (source_type: 3, correction files, ros_recv_packet_topic) - see
# scripts/bunker/README.md. Edit it at
#     ${WS}/src/HesaiLidar_ROS_2.0/config/config.yaml
# and NOT the copy under install/share: the driver bakes PROJECT_PATH in at
# compile time and reads its config from the source tree at runtime, so the
# installed copy is never read.
#
# RUN apt-get update && apt-get install -y --no-install-recommends \
#       git \
#       libyaml-cpp-dev \
#       libboost-all-dev \
#     && rm -rf /var/lib/apt/lists/* \
#     && git clone --recurse-submodules --depth 1 \
#          https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git \
#          src/HesaiLidar_ROS_2.0
# ---------------------------------------------------------------------------

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "source ${WS}/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
