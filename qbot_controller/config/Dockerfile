FROM ros:jazzy-ros-base

###############################################################################
# 1. System update + essential tools
###############################################################################
RUN apt-get update && apt-get install -y \
    sudo \
    git \
    curl \
    gnupg2 \
    zsh \
    python3-pip \
    python3-venv \
    python3-argcomplete \
    python3-colcon-common-extensions \
    build-essential \
    can-utils \
    i2c-tools \
    usbutils \
    joystick \
    && rm -rf /var/lib/apt/lists/*

###############################################################################
# 2. Create non-root runtime user
###############################################################################
RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu && \
    chmod 0440 /etc/sudoers.d/ubuntu

###############################################################################
# 3. Install only required ROS 2 runtime packages
###############################################################################
RUN apt-get update && apt-get install -y \
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-imu-tools \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-diagnostics \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

###############################################################################
# 4. Environment configuration
###############################################################################
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0

USER ubuntu
WORKDIR /home/ubuntu

###############################################################################
# 5. Zsh + ROS sourcing
###############################################################################
RUN echo "source /opt/ros/\${ROS_DISTRO}/setup.zsh" >> ~/.zshrc && \
    echo "source /opt/ros/\${ROS_DISTRO}/share/ros2cli/environment/ros2-argcomplete.zsh" >> ~/.zshrc && \
    echo "autoload -U compinit && compinit" >> ~/.zshrc

###############################################################################
# 6. Clean entrypoint
###############################################################################
ENTRYPOINT []
CMD ["/usr/bin/zsh"]