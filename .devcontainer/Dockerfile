FROM ros@sha256:9749355a0760334fa5ea3c660f2de1ecc5e8a5af8047e4bbaa9afa8a7843da80 AS dev

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
    && rm -rf /var/lib/apt/lists/*

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
COPY .devcontainer/entrypoint.sh /home/ws
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
COPY config config
COPY src src
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release