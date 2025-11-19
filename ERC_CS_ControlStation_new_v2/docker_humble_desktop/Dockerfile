###########################################
# Xplore Control Station image Build Docker
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /usr/local/nvm
ENV NVM_DIR=/usr/local/nvm
ENV NODE_VERSION=20.14.0
ENV NODE_PATH=$NVM_DIR/v$NODE_VERSION/lib/node_modules
ENV PATH=$NVM_DIR/versions/node/v$NODE_VERSION/bin:$PATH

# Set up ROS repository, install packages and install nvm
RUN rm /usr/share/keyrings/ros-archive-keyring.gpg && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    apt update && apt install -y ros2-apt-source && \
    rm /etc/apt/sources.list.d/ros2.list && \
    rm /usr/share/keyrings/ros-archive-keyring.gpg && \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rosbridge-server \
        ros-humble-rmw-cyclonedds-cpp \
        net-tools \
        iputils-ping && \
    rm -rf /var/lib/apt/lists/* /var/cache/apt/* && \
    apt-get clean && \
    curl --silent -o- https://raw.githubusercontent.com/creationix/nvm/v0.39.7/install.sh | bash

USER $USERNAME

# Install Node.js and Python dependencies
SHELL ["/bin/bash", "-c", "source $NVM_DIR/nvm.sh"]
RUN nvm install $NODE_VERSION && \
    nvm alias default $NODE_VERSION && \
    [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh" && \
    [ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion" && \
    nvm use default && \
    pip install --no-cache-dir pymongo tornado && \
    sudo rm -rf /home/$USERNAME/dev_ws/src/*

WORKDIR /home/$USERNAME/dev_ws/src
COPY . .
WORKDIR /home/$USERNAME/dev_ws
