###########################################
# Base image 
###########################################
FROM dustynv/pytorch:2.1-r36.2.0 AS gazebo_nvidia

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

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1

# Install language and timezone
RUN apt-get update && apt-get install -y locales tzdata\
    && locale-gen en_US en_US.UTF-8 \   
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && ln -sf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone \
    && dpkg-reconfigure tzdata \
    && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
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
    software-properties-common \
    git-core \
    python3-colcon-common-extensions \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y ros-dev-tools \
    ros-humble-desktop \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep \
    && apt-get clean

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && rosdep init \ && rosdep update

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/${USERNAME}/.bashrc \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/; fi" >> /home/${USERNAME}/.bashrc \
    && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/${USERNAME}/.bashrc

# Install EposCmd library
COPY ./EposCmd/lib/libEposCmd.so.6.7.1.0 /usr/local/lib/
COPY ./EposCmd/lib/libftd2xx.so.1.4.8 /usr/local/lib/
COPY ./EposCmd/include/Definitions.h /usr/include/EposCmd.h
COPY ./EposCmd/include/ftd2xx.h /usr/include/
COPY ./EposCmd/misc/99-ftdi.rules /etc/udev/rules.d/
COPY ./EposCmd/misc/99-epos4.rules /etc/udev/rules.d/
RUN /etc/init.d/udev restart \
    && ln -s /usr/local/lib/libEposCmd.so.6.7.1.0 /usr/local/lib/libEposCmd.so \
    && ln -s /usr/local/lib/libftd2xx.so.1.4.8 /usr/local/lib/libftd2xx.so \
    && ln -s /usr/local/lib/libEposCmd.so /usr/lib/libEposCmd.so \
    && ln -s /usr/local/lib/libftd2xx.so /usr/lib/libftd2xx.so

# Install common ROS 2 packages
RUN apt-get update && apt-get upgrade -y --fix-broken \
    && apt-get purge -y '*opencv*' \
    && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/* \
    && usermod -aG dialout $USERNAME \
    && usermod -aG video $USERNAME

# Add RMW_IMPLEMENTATION to .bashrc and Add the source of the project to the .bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/${USERNAME}/.bashrc \
    && echo "if [ -f /home/${USERNAME}/dev_ws/install/setup.bash ]; then source /home/${USERNAME}/dev_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc

# Set working directory
WORKDIR /home/$USERNAME

# Install Gazebo
RUN add-apt-repository ppa:openrobotics/gazebo11-non-amd64
RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo \
    libgazebo-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo ROS packages
# RUN apt-get install -y --no-install-recommends \
#     ros-${ROS_DISTRO}-gazebo-ros-pkgs
