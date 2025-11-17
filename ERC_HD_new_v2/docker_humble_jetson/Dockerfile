
FROM ghcr.io/epflxplore/docker_commons:humble-jetson-nx-hd

ENV DEBIAN_FRONTEND=noninteractive

# Set up ROS repository and install base packages
RUN rm /usr/share/keyrings/ros-archive-keyring.gpg && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    apt update && \
    apt install -y ros2-apt-source && \
    rm /etc/apt/sources.list.d/ros2.list && \
    rm /usr/share/keyrings/ros-archive-keyring.gpg && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
        cmake \
        build-essential \
        libssl-dev \
        libusb-1.0-0-dev \
        libusb-1.0-0 \
        libudev-dev \
        pkg-config \
        libgtk-3-dev \
        python3 \
        python3-setuptools \
        python3-dev \
        python3-pip \
        python3-pybind11 \
        v4l-utils \
        libxinerama-dev \
        libxcursor-dev \
        libcanberra-gtk-module \
        libcanberra-gtk3-module && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install ROS 2 HD packages and configure USB rules
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-depthai-ros \
        ros-${ROS_DISTRO}-hardware-interface \
        ros-${ROS_DISTRO}-control-toolbox \
        ros-${ROS_DISTRO}-moveit-servo \
        ros-${ROS_DISTRO}-controller-manager \
        ros-${ROS_DISTRO}-librealsense2* \
        ros-${ROS_DISTRO}-realsense2-* \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-pick-ik \
        python3-opencv \
        libopencv-dev \
        python3-zmq \
        libyaml-cpp-dev \
        lcov && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules && \
    echo '\
# Load and unload nvidia-uvm module\n\
ACTION=="add", DEVPATH=="/bus/pci/drivers/nvidia", RUN+="/sbin/modprobe nvidia-uvm"\n\
ACTION=="remove", DEVPATH=="/bus/pci/drivers/nvidia", RUN+="/sbin/modprobe -r nvidia-uvm"\n\
\n\
# Trigger nvidia-smi on nvidia driver load\n\
ACTION=="add", DEVPATH=="/bus/pci/drivers/nvidia", RUN+="/usr/bin/nvidia-smi"\n\
\n\
# Create device node for nvidia-uvm\n\
ACTION=="add", DEVPATH=="/module/nvidia_uvm", SUBSYSTEM=="module", RUN+="/usr/bin/nvidia-modprobe -u -c=0"\n\
' | tee /etc/udev/rules.d/99-nvidia.rules && \
    /etc/init.d/udev restart && \
    rm -rf /var/lib/apt/lists/*

ENV NVIDIA_DRIVERS_CAPABILITIES=all
USER $USERNAME

# Install Python packages and ML models
RUN sudo pip install -U jetson-stats && \
    pip install --no-cache-dir \
        git+https://github.com/ChaoningZhang/MobileSAM.git \
        git+https://github.com/facebookresearch/segment-anything.git \
        evdev \
        depthai \
        pyrealsense2 \
        ultralytics \
        timm \
        opencv-python==4.7.0.72 \
        opencv-contrib-python==4.7.0.72 \
        numpy==1.24.0

# With this installation, we have torch 2.8.0 and torchvision 0.23.0. The issue with that is that the GPU won't be accessible inside the docker
# as the base image (jetson containers) was built using torch 2.1.0. Since for ERC 2025 we won't use the GPU even with our models, we leave this
# setup. To keep the torch 2.1.0, you need to uninstall torch before switching user (uncomment the line), and place 
# /opt/torch-2.1.0-cp310-cp310-linux_aarch64.whl after installing numpy, on the same line. With this you won't have again the GPU, but to solve
# that, you need to run the perception node as ROOT, and torch needs to be installed in the root user. 

WORKDIR /home/$USERNAME/dev_ws/src
COPY . .
WORKDIR /home/$USERNAME/dev_ws

# Remove confidential source code
RUN sudo rm -rf /home/$USERNAME/dev_ws/src/*
