
FROM ghcr.io/epflxplore/docker_commons:humble-jetson-nx-hd

ENV DEBIAN_FRONTEND=noninteractive

RUN rm /usr/share/keyrings/ros-archive-keyring.gpg
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN apt update && apt install ros2-apt-source  
RUN rm /etc/apt/sources.list.d/ros2.list
RUN rm /usr/share/keyrings/ros-archive-keyring.gpg

RUN apt-get update && apt-get install -y --no-install-recommends \
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
    libcanberra-gtk3-module \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install ROS 2 HD packages
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-depthai-ros \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-librealsense2*\
    ros-${ROS_DISTRO}-realsense2-*\
    ros-${ROS_DISTRO}-cv-bridge \
    python3-opencv \
    libopencv-dev \
    python3-zmq \
    libyaml-cpp-dev \
    lcov

RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-pick-ik


# Add USB rules
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules

RUN echo '\
# Load and unload nvidia-uvm module\n\
ACTION=="add", DEVPATH=="/bus/pci/drivers/nvidia", RUN+="/sbin/modprobe nvidia-uvm"\n\
ACTION=="remove", DEVPATH=="/bus/pci/drivers/nvidia", RUN+="/sbin/modprobe -r nvidia-uvm"\n\
\n\
# Trigger nvidia-smi on nvidia driver load\n\
ACTION=="add", DEVPATH=="/bus/pci/drivers/nvidia", RUN+="/usr/bin/nvidia-smi"\n\
\n\
# Create device node for nvidia-uvm\n\
ACTION=="add", DEVPATH=="/module/nvidia_uvm", SUBSYSTEM=="module", RUN+="/usr/bin/nvidia-modprobe -u -c=0"\n\
' | sudo tee /etc/udev/rules.d/99-nvidia.rules

ENV NVIDIA_DRIVERS_CAPABILITIES=all

RUN /etc/init.d/udev restart 

#RUN pip uninstall torch -y
USER $USERNAME

# sudo is required for this package (documentation)
RUN sudo pip install -U jetson-stats

# These 2 models are for segment stuff that appear on an image. The first one if faster.
RUN echo "Installing Segment Anything and MobileSAM models"
RUN pip install git+https://github.com/ChaoningZhang/MobileSAM.git
RUN pip install git+https://github.com/facebookresearch/segment-anything.git

RUN pip install --no-cache-dir -v evdev depthai pyrealsense2 ultralytics timm opencv-python==4.7.0.72 opencv-contrib-python==4.7.0.72 numpy==1.24.0

# With this installation, we have torch 2.8.0 and torchvision 0.23.0. The issue with that is that the GPU won't be accessible inside the docker
# as the base image (jetson containers) was built using torch 2.1.0. Since for ERC 2025 we won't use the GPU even with our models, we leave this
# setup. To keep the torch 2.1.0, you need to uninstall torch before switching user (uncomment the line), and place 
# /opt/torch-2.1.0-cp310-cp310-linux_aarch64.whl after installing numpy, on the same line. With this you won't have again the GPU, but to solve
# that, you need to run the perception node as ROOT, and torch needs to be installed in the root user. 

# Set a diretory to store the project
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .

# Set a directory to build the project
WORKDIR /home/$USERNAME/dev_ws

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*

# Remove all the confidential Xplore source code from the image
RUN sudo rm -rf /home/$USERNAME/dev_ws/src/*
