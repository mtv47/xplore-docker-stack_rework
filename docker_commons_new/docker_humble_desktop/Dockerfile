###########################################
# Base image 
###########################################
FROM ubuntu:22.04 AS xplore_common

ARG DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# Install timezone
ENV TZ=Europe/Zurich
RUN apt-get update && apt-get install -y tzdata \
    && ln -sf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone \
    && dpkg-reconfigure tzdata

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    wget

# Install ROS 2 Humble
RUN add-apt-repository universe \
    && apt-get update \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get upgrade -y && apt-get install -y ros-humble-desktop

# Clean up
RUN rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    gdb \
    git \
    openssh-client \
    python3-argcomplete \
    python3-pip \
    ros-dev-tools \
    vim

RUN rosdep init || echo "rosdep already initialized"

ENV USERNAME=xplore
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Add sudo support for the non-root user
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set up autocompletion for user
RUN apt-get update && apt-get install -y git-core bash-completion \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/${USERNAME}/.bashrc \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/; fi" >> /home/${USERNAME}/.bashrc \
    && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/${USERNAME}/.bashrc

# Clean up
RUN rm -rf /var/lib/apt/lists/*

ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

# Install EposCmd library
WORKDIR /usr/local/lib
COPY ./EposCmd/lib/libEposCmd.so.6.7.1.0 .
COPY ./EposCmd/lib/libftd2xx.so.1.4.27 .
RUN ln -s libEposCmd.so.6.7.1.0 libEposCmd.so
RUN ln -s libftd2xx.so.1.4.27 libftd2xx.so

WORKDIR /usr/lib
RUN ln -s /usr/local/lib/libEposCmd.so libEposCmd.so
RUN ln -s /usr/local/lib/libftd2xx.so libftd2xx.so

WORKDIR /usr/include
COPY ./EposCmd/include/Definitions.h .
COPY ./EposCmd/include/ftd2xx.h .
RUN mv Definitions.h EposCmd.h

WORKDIR /etc/udev/rules.d
COPY ./EposCmd/misc/99-ftdi.rules .
COPY ./EposCmd/misc/99-epos4.rules .
RUN /etc/init.d/udev restart

# Install common ROS 2 packages
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

WORKDIR /home/$USERNAME

# Allow non-root user to access the serial ports
RUN usermod -aG dialout $USERNAME
# Allow non-root user to access the video devices
RUN usermod -aG video $USERNAME

# Add RMW_IMPLEMENTATION to .bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/${USERNAME}/.bashrc
# Add the source of the project to the .bashrc
RUN echo "if [ -f /home/${USERNAME}/dev_ws/install/setup.bash ]; then source /home/${USERNAME}/dev_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc

# Clean up
RUN rm -rf /var/lib/apt/lists/*

###########################################
#  Dev+Gazebo+Nvidia image 
###########################################
FROM xplore_common AS gazebo_nvidia

ARG DEBIAN_FRONTEND=noninteractive

# Install nvidia-driver version 470
RUN apt-get update && apt-get install -y nvidia-driver-470

# Install CUDA Toolkit 11.8
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb \
    && dpkg -i cuda-keyring_1.0-1_all.deb
RUN apt-get update && apt-get install -y --no-install-recommends cuda-toolkit-11-8

# Source CUDA
ENV CUDA_HOME=/usr/local/cuda-11.8
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:$LD_LIBRARY_PATH
ENV PATH=${CUDA_HOME}/bin:$PATH

################
# Expose the nvidia driver to allow opengl 
# Dependencies for glvnd and X11.
################
RUN apt-get update \
    && apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6

# Install Gazebo
RUN curl -sSL http://get.gazebosim.org | sh

# Install Gazebo ROS packages
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs

# Clean up
RUN rm -rf /var/lib/apt/lists/*
RUN rm -f cuda-keyring_1.0-1_all.deb

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1