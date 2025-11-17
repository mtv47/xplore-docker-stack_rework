###########################################
# Base image 
###########################################
FROM dustynv/ros:foxy-desktop-l4t-r35.4.1 AS gazebo_nvidia

ARG DEBIAN_FRONTEND=noninteractive

# Other env vars
ENV LANG=en_US.UTF-8
ENV TZ=Europe/Zurich
ENV ROS_DISTRO=foxy
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.8/site-packages
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

# Install programs and create non-root user
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
    git-core \
    openssh-client \
    python3-argcomplete \
    python3-pip \
    ros-dev-tools \
    vim \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    && rosdep init || echo "rosdep already initialized" \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/*

# Set up autocompletion and environment for user
RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/${USERNAME}/.bashrc \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/; fi" >> /home/${USERNAME}/.bashrc \
    && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/${USERNAME}/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/${USERNAME}/.bashrc \
    && echo "if [ -f /home/${USERNAME}/dev_ws/install/setup.bash ]; then source /home/${USERNAME}/dev_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc

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

# Install common ROS 2 packages and configure user groups
RUN apt-get update && apt-get upgrade -y \
    && apt-get purge -y '*opencv*' \
    && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/* \
    && usermod -aG dialout $USERNAME \
    && usermod -aG video $USERNAME

WORKDIR /home/$USERNAME

# Install Gazebo and Gazebo ROS packages
RUN curl -sSL http://get.gazebosim.org | sh \
    && apt-get update && apt-get upgrade -y \
    && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/*