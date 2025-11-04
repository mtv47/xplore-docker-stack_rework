###########################################
# Xplore Handling Device image
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

RUN rm /usr/share/keyrings/ros-archive-keyring.gpg
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN apt update && apt install ros2-apt-source  
RUN rm /etc/apt/sources.list.d/ros2.list
RUN rm /usr/share/keyrings/ros-archive-keyring.gpg

# Install ROS 2 HD packages
RUN apt-get update && apt-get upgrade -y

RUN apt-get update
RUN apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-depthai-ros \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    python3-zmq \
    libyaml-cpp-dev \
    lcov \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    libgtk-3-dev\
    wget \
    cmake\
    build-essential \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    at \
    unzip \
    build-essential \
    cmake \
    dkms \
    git \
    ros-${ROS_DISTRO}-cv-bridge \
    python3-opencv \
    libopencv-dev

RUN mkdir -p /etc/apt/keyrings

# # Install librealsense
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list
RUN apt-get update

#RUN apt-get update && apt-get install -y \
#    librealsense2-dkms \
#    librealsense2-utils


# Add USB rules
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
RUN /etc/init.d/udev restart


# Install the necessary Python packages
RUN pip install --no-cache-dir -v evdev depthai pyrealsense2 ultralytics opencv-python opencv-contrib-python==4.7.0.72 numpy==1.24.0

USER $USERNAME

# Ensure Python and pip are installed and updated
RUN pip install --no-cache-dir --upgrade pip


# Set a diretory to store the project
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .

# Set a directory to build the project
WORKDIR /home/$USERNAME/dev_ws

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*

# Remove all the confidential Xplore source code from the image
RUN sudo rm -rf /home/$USERNAME/dev_ws/src/*