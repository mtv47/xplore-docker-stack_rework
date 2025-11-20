###########################################
# Base image 
###########################################
FROM ubuntu:22.04 AS xplore_common

ARG DEBIAN_FRONTEND=noninteractive

# Other env vars
ENV LANG en_US.UTF-8
ENV TZ=Europe/Zurich
ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

# User args and envs
ENV USERNAME=xplore
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Install language, timezone and common programs
RUN apt-get update && apt-get install -y locales tzdata\
    && locale-gen en_US en_US.UTF-8 \   
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && ln -sf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone \
    && dpkg-reconfigure tzdata \
    && apt-get install -y --no-install-recommends \
        curl \
        gnupg2 \
        lsb-release \
        sudo \
        software-properties-common \
        wget \
        bash-completion \
        build-essential \
        cmake \
        gdb \
        git \
        openssh-client \
        python3-argcomplete \
        python3-pip \
        vim \
        git-core \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/* \
    && apt-get clean

# Install ROS 2 Humble
RUN add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update \
    && apt-get install -y ros-humble-desktop ros-dev-tools\
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/* \
    && apt-get clean \
    && rosdep init || echo "rosdep already initialized"

# Create a non-root user and set up autocompletion for user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/${USERNAME}/.bashrc \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/; fi" >> /home/${USERNAME}/.bashrc \
    && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/${USERNAME}/.bashrc

# Install EposCmd library
COPY ./EposCmd/lib/libEposCmd.so.6.7.1.0 /usr/local/lib/
COPY ./EposCmd/lib/libftd2xx.so.1.4.27 /usr/local/lib/
COPY ./EposCmd/include/Definitions.h /usr/include/EposCmd.h
COPY ./EposCmd/include/ftd2xx.h /usr/include/
COPY ./EposCmd/misc/99-ftdi.rules /etc/udev/rules.d/
COPY ./EposCmd/misc/99-epos4.rules /etc/udev/rules.d/
RUN /etc/init.d/udev restart \
    && ln -s /usr/local/lib/libEposCmd.so.6.7.1.0 /usr/local/lib/libEposCmd.so \
    && ln -s /usr/local/lib/libftd2xx.so.1.4.27 /usr/local/lib/libftd2xx.so \
    && ln -s /usr/local/lib/libEposCmd.so /usr/lib/libEposCmd.so \
    && ln -s /usr/local/lib/libftd2xx.so /usr/lib/libftd2xx.so

# Install common ROS 2 packages and configure user groups
# and add RMW_IMPLEMENTATION to .bashrc and Add the source of the project to the .bashrc
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && usermod -aG dialout $USERNAME \
    && usermod -aG video $USERNAME \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/* \
    && apt-get clean \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/${USERNAME}/.bashrc \
    && echo "if [ -f /home/${USERNAME}/dev_ws/install/setup.bash ]; then source /home/${USERNAME}/dev_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc

# Set working directory
WORKDIR /home/$USERNAME

###########################################
#  Dev+Gazebo+Nvidia image 
###########################################
FROM xplore_common AS gazebo_nvidia

ARG DEBIAN_FRONTEND=noninteractive

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1
# Source CUDA
ENV CUDA_HOME=/usr/local/cuda-11.8
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:$LD_LIBRARY_PATH
ENV PATH=${CUDA_HOME}/bin:$PATH

# Install nvidia-driver, CUDA, OpenGL dependencies, Gazebo and Gazebo ROS packages
RUN apt-get update \
    && apt-get install -y nvidia-driver-470 \
    && wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb \
    && dpkg -i cuda-keyring_1.0-1_all.deb \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        cuda-toolkit-11-8 \
        libglvnd0 \
        libgl1 \
        libglx0 \
        libegl1 \
        libxext6 \
        libx11-6 \
        gnupg \
    && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get -y install ignition-fortress \
    && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/*

FROM xplore_common AS gazebo_nvidia_new

ARG DEBIAN_FRONTEND=noninteractive

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1
# Source CUDA
ENV CUDA_HOME=/usr/local/cuda-11.8
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:$LD_LIBRARY_PATH
ENV PATH=${CUDA_HOME}/bin:$PATH

# Install only CUDA runtime libraries instead of full toolkit
RUN apt-get update \
    && apt-get install -y nvidia-driver-470 \
    && wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb \
    && dpkg -i cuda-keyring_1.0-1_all.deb \
    && rm cuda-keyring_1.0-1_all.deb \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        cuda-cudart-11-8 \
        cuda-libraries-11-8 \
        libcublas-11-8 \
        libcufft-11-8 \
        libcurand-11-8 \
        libcusolver-11-8 \
        libcusparse-11-8 \
        libnpp-11-8 \
        libnvjpeg-11-8 \
        libglvnd0 \
        libgl1 \
        libglx0 \
        libegl1 \
        libxext6 \
        libx11-6 \
        gnupg \
    && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get -y install ignition-fortress \
    && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/*