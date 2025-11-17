###########################################
# Xplore Navigation image
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-desktop-nvidia

ARG DEBIAN_FRONTEND=noninteractive

# Install ROS 2 Nav packages and configure USB rules
RUN add-apt-repository ppa:borglab/gtsam-release-4.1 && \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-robot-localization \
        ros-${ROS_DISTRO}-slam-toolbox \
        ros-${ROS_DISTRO}-test-msgs \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-libg2o \
        ros-${ROS_DISTRO}-depthai-ros \
        ros-${ROS_DISTRO}-librealsense2* \
        ros-${ROS_DISTRO}-realsense2-* \
        ros-${ROS_DISTRO}-foxglove-bridge \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-gazebo-ros2-control \
        ros-${ROS_DISTRO}-rqt-tf-tree \
        python3-zmq \
        libyaml-cpp-dev \
        lcov \
        libtins-dev \
        net-tools \
        iproute2 \
        dnsmasq \
        libgtsam-dev \
        libgtsam-unstable-dev && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules && \
    /etc/init.d/udev restart && \
    rm -rf /var/lib/apt/lists/*

# Add Lidar Host
# RUN echo "169.254.55.220  os-122140001125.local" >> /etc/hosts
# Can be replaced with the --add-host argument https://docs.docker.com/reference/cli/docker/container/run/#add-host

USER $USERNAME

WORKDIR /home/$USERNAME/dev_ws/src
COPY . .
WORKDIR /home/$USERNAME/dev_ws

# Install ROS 2 Nav dependencies and configure bashrc
RUN rosdep update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y -r && \
    echo "if [ -f /home/${USERNAME}/dev_ws/install/setup.bash ]; then source /home/${USERNAME}/dev_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc

# Install additional Python packages and remove confidential source code
RUN pip install --no-cache-dir \
        opencv-python \
        opencv-contrib-python==4.6.0.66 \
        depthai \
        pygame_gui \
        pygame \
        transforms3d \
        pyserial \
        open3d && \
    pip uninstall -y numpy && \
    pip install --no-cache-dir numpy==1.22.4 && \
    sudo rm -rf /home/$USERNAME/dev_ws/src/*
