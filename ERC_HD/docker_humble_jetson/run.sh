XAUTH=/tmp/.docker.xauth

echo "Preparing Xauthority data..."

# Always create the file if missing
if [ ! -f $XAUTH ]; then
    touch $XAUTH
fi

# Always (re)populate the Xauthority with the real SSH xauth cookie
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# Make sure it’s readable
chmod a+r $XAUTH

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."



echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."

# Get the current working directory
current_dir=$(pwd)

# Use dirname to get the parent directory
parent_dir=$(dirname "$current_dir")

USERNAME=xplore

docker run -it \
    --name hd_humble \
    --rm \
    --runtime=nvidia \
    --gpus all \
    --privileged \
    --net=host \
    --pid=host \
    --group-add video \
    --group-add render \
    --group-add input \
    --ipc=host \
    --device /dev/bus/usb \
    -e DISPLAY=unix$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -v $XAUTH:$XAUTH \
    -v /dev:/dev \
    -v /run/jtop.sock:/run/jtop.sock \
    -v /run/user/user/1000/at-spi:/run/user/1000/at-spi \
    -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra:ro \
    -v /usr/local/cuda:/usr/local/cuda:ro \
    -v $parent_dir:/home/xplore/dev_ws/src \
    -v /home/xplore-hd/Documents/photos_competition:/home/xplore/dev_ws/photos_competition \
    -v hd_humble_jetson_home_volume:/home/xplore \
    -v ~/Documents/ERC_HD/docker_humble_jetson/cyclonedds.xml:/home/xplore/cyclonedds.xml:ro \
    -e CYCLONEDDS_URI="file:///home/xplore/cyclonedds.xml" \
    ghcr.io/epflxplore/hd:humble-jetson-test \
    /bin/bash