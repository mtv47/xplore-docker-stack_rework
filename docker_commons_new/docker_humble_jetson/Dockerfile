###########################################
# Base image 
###########################################
FROM dustynv/ros:humble-desktop-l4t-r36.2.0 AS gazebo_nvidia

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
# Create a non-root user and set up autocompletion for user

RUN rm /usr/share/keyrings/ros-archive-keyring.gpg \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && apt update && apt install ros2-apt-source \
    && rm /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y locales tzdata\
    && locale-gen en_US en_US.UTF-8 \   
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && ln -sf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone \
    && dpkg-reconfigure tzdata \
    && apt-get update \
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
        ros-dev-tools \
        vim \
        git-core \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/* \
    && apt-get clean \
    && rosdep init || echo "rosdep already initialized" \
    && groupadd --gid $USER_GID $USERNAME \
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
RUN ln -s /usr/local/lib/libEposCmd.so.6.7.1.0 /usr/local/lib/libEposCmd.so \
    && ln -s /usr/local/lib/libftd2xx.so.1.4.8 /usr/local/lib/libftd2xx.so \
    && ln -s /usr/local/lib/libEposCmd.so /usr/lib/libEposCmd.so \
    && ln -s /usr/local/lib/libftd2xx.so /usr/lib/libftd2xx.so \
    && /etc/init.d/udev restart

# Install Gazebo
# Install common ROS 2 packages and configure user groups
# Add RMW_IMPLEMENTATION to .bashrc and Add the source of the project to the .bashrc
RUN add-apt-repository ppa:openrobotics/gazebo11-non-amd64 \
    && apt-get update && apt-get upgrade -y \
    && apt-get purge -y '*opencv*' \
    && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        gazebo \
        libgazebo-dev \
        libglvnd0 \
        libgl1 \
        libglx0 \
        libegl1 \
        libxext6 \
        libx11-6 \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/* \
    && apt-get clean \
    && usermod -aG dialout $USERNAME \
    && usermod -aG video $USERNAME \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/${USERNAME}/.bashrc \
    && echo "if [ -f /home/${USERNAME}/dev_ws/install/setup.bash ]; then source /home/${USERNAME}/dev_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc

# Set working directory
WORKDIR /home/$USERNAME

# Install Gazebo ROS packages
# RUN apt-get install -y --no-install-recommends \
#     ros-${ROS_DISTRO}-gazebo-ros-pkgs