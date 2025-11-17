###########################################
# Xplore Rover image Build Docker
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-desktop as build-rover

ARG DEBIAN_FRONTEND=noninteractive

# Install all packages in a single layer to reduce image size
RUN sudo rm /usr/share/keyrings/ros-archive-keyring.gpg && \
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    sudo apt update && sudo apt install ros2-apt-source && \
    sudo rm /etc/apt/sources.list.d/ros2.list && \ 
    sudo rm /usr/share/keyrings/ros-archive-keyring.gpg && \
    sudo apt-get update && sudo apt-get upgrade -y && \
    ACCEPT_EULA=Y sudo apt-get install -y --no-install-recommends \
        libyaml-cpp-dev \
        iproute2 \
        kmod \
        busybox \
        ubuntu-restricted-extras \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        gcc-arm-none-eabi \
        libstdc++-arm-none-eabi-newlib \
        libnewlib-arm-none-eabi \
        gdb-multiarch \
        libserialport-dev && \
    rm -rf /var/lib/apt/lists/*

USER $USERNAME

RUN python3 -m pip install numpy pymodbus pyserial

# Set a diretory to store the project and a directory to build the project
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .
WORKDIR /home/$USERNAME/dev_ws

# Add the source of the project to the .bashrc and remove confidential source code
RUN echo "if [ -f /home/${USERNAME}/dev_ws/install/setup.bash ]; then source /home/${USERNAME}/dev_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc && \
    sudo rm -rf /home/$USERNAME/dev_ws/src/*