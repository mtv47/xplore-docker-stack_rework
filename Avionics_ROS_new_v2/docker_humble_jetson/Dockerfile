##########################################
# Xplore Electronics image
###########################################
FROM dustynv/ros:humble-desktop-l4t-r36.2.0 AS base

ARG DEBIAN_FRONTEND=noninteractive

# Set up ROS repository and clean up
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    sudo apt update && \
    sudo apt install -y ros2-apt-source && \
    sudo rm -f /etc/apt/sources.list.d/ros2.list /usr/share/keyrings/ros-archive-keyring.gpg

ENV USERNAME=xplore
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user and set up environment
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    apt-get update && \
    apt-get install -y sudo git-core bash-completion && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/${USERNAME}/.bashrc && \
    echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/; fi" >> /home/${USERNAME}/.bashrc && \
    echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/${USERNAME}/.bashrc && \
    rm -rf /var/lib/apt/lists/*

# Install packages and Python dependencies
RUN apt-get update && apt-get upgrade -y && \
    ACCEPT_EULA=Y apt-get install -y --no-install-recommends \
        ubuntu-restricted-extras \
        libserialport-dev \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev && \
    pip install numpy pymodbus pyserial minimalmodbus && \
    rm -rf /var/lib/apt/lists/*

USER $USERNAME

WORKDIR /home/$USERNAME/dev_ws/src
COPY . .
WORKDIR /home/$USERNAME/dev_ws

# Add the source of the project to the .bashrc and remove confidential source code
RUN echo "if [ -f /home/${USERNAME}/dev_ws/install/setup.bash ]; then source /home/${USERNAME}/dev_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc && \
    sudo rm -rf /home/$USERNAME/dev_ws/src/*
