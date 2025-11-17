###########################################
# Xplore Navigation image
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-jetson

ARG DEBIAN_FRONTEND=noninteractive

# Install ROS 2 packages and configure USB rules
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y ros-humble-rmw-cyclonedds-cpp && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules && \
    /etc/init.d/udev restart && \
    rm -rf /var/lib/apt/lists/*

USER $USERNAME

# Install Python dependencies
RUN pip install --no-cache-dir \
    evdev \
    opencv-python==4.7.0.72 \
    opencv-contrib-python==4.7.0.72 \
    numpy==1.24.0

WORKDIR /home/$USERNAME/dev_ws/src
COPY . .
WORKDIR /home/$USERNAME/dev_ws

# Configure bashrc and remove confidential source code
RUN echo "if [ -f /home/${USERNAME}/dev_ws/install/setup.bash ]; then source /home/${USERNAME}/dev_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc && \
    sudo rm -rf /home/$USERNAME/dev_ws/src/*
