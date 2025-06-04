FROM osrf/ros@sha256:f2f03a667950570edcaa14ed414a636912acd44ea9c25c10b5a7c5d44eea5142 AS dev

# NOTE: this updates the GPG key for ros2 apt repository (to be deleted when new ros2 jazzy docker image is released)
RUN rm /etc/apt/sources.list.d/ros2-latest.list \
  && rm /usr/share/keyrings/ros2-latest-archive-keyring.gpg

RUN apt-get update \
  && apt-get install -y ca-certificates curl

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb
    
# ********************************************************
# * Install Dependencies *
# ********************************************************

# Override default shell, because ROS needs bash
SHELL [ "/bin/bash", "-c" ]

# Install ros_dev_tools
RUN sudo apt update && sudo apt install -y \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Install Linux packages
# xterm: to use teleop_twist_keyboard
# xvfb: virtual display for headless testing
RUN apt-get update && apt-get install -y \
    xterm \
    xvfb \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install Docker Engine
RUN curl -fsSL https://get.docker.com -o get-docker.sh && \
    sh get-docker.sh && \
    rm get-docker.sh

# Install ROS debug packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-demo-nodes-cpp \
    ros-jazzy-demo-nodes-py \
    ros-jazzy-rqt-graph \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-rqt-robot-steering \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-topic-tools \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros2-control \
    ros-jazzy-diff-drive-controller \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    ros-jazzy-twist-mux \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-fuse \
    && rm -rf /var/lib/apt/lists/*

# Install cyclonedds rmw and set up the configuration
RUN apt-get update && apt-get install -y \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV RMW_CYCLONEDDS_URI=file:///home/ws/config/cyclonedds.xml

# ********************************************************
# * Set up the user *
# ********************************************************
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
ENV SHELL=/bin/bash

# Define entrypoint
RUN mkdir -p /home/ws
COPY docker/entrypoint.sh /home/ws
RUN chmod +x /home/ws/entrypoint.sh
ENTRYPOINT [ "/home/ws/entrypoint.sh" ]
CMD ["/bin/bash"]

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

# ********************************************************
# * Production *
# ********************************************************
FROM dev AS prod

# Copy our packages into the image and build them
USER root
WORKDIR /home/ws
COPY src src
COPY docker/docker-compose.yml docker/
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release