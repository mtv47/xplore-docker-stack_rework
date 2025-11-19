###########################################
# Xplore Rover image Build Docker
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-desktop as build-rover

ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /home/$USERNAME

# Install Rover packages and configure USB rules, and install python dependencies
RUN rm /usr/share/keyrings/ros-archive-keyring.gpg && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    apt update && apt install ros2-apt-source && \
    rm /etc/apt/sources.list.d/ros2.list && \
    rm /usr/share/keyrings/ros-archive-keyring.gpg && \
    apt-get update && apt-get upgrade -y && \
    ACCEPT_EULA=Y apt-get install -y --no-install-recommends \
        curl \
        python3-opencv \
        ubuntu-restricted-extras \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-bringup && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules && \
    /etc/init.d/udev restart && \
    rm -rf /var/lib/apt/lists/* /var/cache/apt/* && \
    apt-get clean && \
    pip install --no-cache-dir \
        numpy \
        aiohttp \
        aiortc \
        aiohttp_cors \
        ping3 \
        tornado \
        requests \
        bson \
        pymongo

WORKDIR /home/$USERNAME/dev_ws/src
COPY . .
WORKDIR /home/$USERNAME/dev_ws

USER $USERNAME

# Install ROS 2 Nav dependencies and remove confidential source code
RUN rosdep update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y -r && \
    sudo rm -rf /home/$USERNAME/dev_ws/src/*
