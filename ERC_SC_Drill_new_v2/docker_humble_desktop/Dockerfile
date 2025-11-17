###########################################
# Xplore Navigation image
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-desktop-nvidia

ARG DEBIAN_FRONTEND=noninteractive

# Install ROS 2 packages and configure USB rules
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y ros-humble-rmw-cyclonedds-cpp && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules && \
    /etc/init.d/udev restart && \
    rm -rf /var/lib/apt/lists/*

USER $USERNAME

WORKDIR /home/$USERNAME/dev_ws/src
COPY . .
WORKDIR /home/$USERNAME/dev_ws

# Install ROS 2 dependencies, configure bashrc, and remove confidential source code
RUN rosdep update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y -r && \
    echo "if [ -f /home/${USERNAME}/dev_ws/install/setup.bash ]; then source /home/${USERNAME}/dev_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc && \
    sudo rm -rf /home/$USERNAME/dev_ws/src/*
