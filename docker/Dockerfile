# System dependencies only; build pinned Ceres/GTSAM separately.
FROM fishros2/ros:humble-desktop-full
SHELL ["/bin/bash", "-c"]
RUN apt-get -o Dir::Etc::sourcelist=/etc/apt/sources.list -o Dir::Etc::sourceparts=- update && \
    DEBIAN_FRONTEND=noninteractive apt-get -o Dir::Etc::sourcelist=/etc/apt/sources.list -o Dir::Etc::sourceparts=- install -y --no-install-recommends \
    git libgoogle-glog-dev libgflags-dev libsuitesparse-dev libmetis-dev libboost-all-dev && \
    rm -rf /var/lib/apt/lists/*
# Tag is a bootstrap recipe; exact tested local image ID is in locks/environment.json.
WORKDIR /repo
