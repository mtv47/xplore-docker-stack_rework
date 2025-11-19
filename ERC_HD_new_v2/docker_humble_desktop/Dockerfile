###########################################
# Xplore Handling Device image
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

# Set up ROS repository, install packages, configure librealsense, USB rules, and Python dependencies
RUN rm /usr/share/keyrings/ros-archive-keyring.gpg && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    apt update && \
    apt install -y ros2-apt-source && \
    rm /etc/apt/sources.list.d/ros2.list && \
    rm /usr/share/keyrings/ros-archive-keyring.gpg && \
    apt-get update && apt-get upgrade -y && \
    mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/librealsense.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-depthai-ros \
        ros-${ROS_DISTRO}-hardware-interface \
        ros-${ROS_DISTRO}-control-toolbox \
        ros-${ROS_DISTRO}-moveit-servo \
        ros-${ROS_DISTRO}-controller-manager \
        ros-${ROS_DISTRO}-gazebo-ros \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-cv-bridge \
        python3-zmq \
        python3-opencv \
        libyaml-cpp-dev \
        lcov \
        libssl-dev \
        libusb-1.0-0-dev \
        libudev-dev \
        pkg-config \
        libgtk-3-dev \
        wget \
        cmake \
        build-essential \
        libglfw3-dev \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        libopencv-dev \
        at \
        unzip \
        dkms \
        git && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules && \
    /etc/init.d/udev restart && \
    pip install --no-cache-dir -v evdev depthai pyrealsense2 ultralytics opencv-python opencv-contrib-python==4.7.0.72 numpy==1.24.0 && \
    pip install --no-cache-dir --upgrade pip && \
    rm -rf /var/lib/apt/lists/* /var/cache/apt/* && \
    apt-get clean && \
    rm -rf /home/$USERNAME/dev_ws/src/*

USER $USERNAME

WORKDIR /home/$USERNAME/dev_ws/src
COPY . .
WORKDIR /home/$USERNAME/dev_ws