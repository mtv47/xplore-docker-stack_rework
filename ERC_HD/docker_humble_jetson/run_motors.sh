#!/bin/bash

XAUTH=/tmp/.docker.xauth
USERNAME=xplore
CONTAINER_NAME=hd_humble_motors
IMAGE_NAME=ghcr.io/epflxplore/hd:humble-jetson-test

DOCKER_COMMAND="sudo chown -R $USERNAME:$USERNAME /home/$USERNAME; source src/docker_humble_jetson/attach.sh; ros2 run ethercat_device_configurator motor_control"

# Function to check if a Docker container is running
is_container_running() {
    if [ "$(docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null)" == "true" ]; then
        echo "true"
    else
        echo "false"
    fi
}

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""

# Get the current working directory and parent directory
current_dir=$(pwd)
parent_dir=$(dirname "$current_dir")

docker run -it \
    --name $CONTAINER_NAME \
    --rm \
    --privileged \
    --net=host \
    --user root \
    -e DISPLAY=unix$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $XAUTH:$XAUTH \
    -v /run/user/1000/at-spi:/run/user/1000/at-spi \
    -v /dev:/dev \
    -v $parent_dir:/home/$USERNAME/dev_ws/src \
    -v hd_humble_jetson_home_volume:/home/$USERNAME \
    $IMAGE_NAME \
    /bin/bash -c "$DOCKER_COMMAND"