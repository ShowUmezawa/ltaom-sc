# Noetic compatibility environment; official Melodic image download failed.
FROM fishros2/ros@sha256:4a7705e75022cc5ff60cecb7b5b40bc16492e94b1391a82ab1dda93ab1436b4c
SHELL ["/bin/bash", "-c"]
# ROS is already installed. This image's ROS apt key has expired; retain it
# disabled and install only signed Ubuntu dependencies, without bypassing GPG.
RUN mv /etc/apt/sources.list.d/ros-fish.list /etc/apt/sources.list.d/ros-fish.list.disabled && \
    apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    git libgoogle-glog-dev libgflags-dev libsuitesparse-dev libboost-all-dev python3-numpy && \
    rm -rf /var/lib/apt/lists/*
WORKDIR /work
# Mount workspace and run build_dependencies.sh, then build_workspace.sh.
