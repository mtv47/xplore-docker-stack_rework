###########################################
# Xplore Navigation image
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-jetson

ARG DEBIAN_FRONTEND=noninteractive

ENV PYTHONPATH=/usr/local/lib/python3.10/dist-packages
ENV PYTHONPATH=/usr/local/lib/python3.10/dist-packages/pyrealsense2:$PYTHONPATH

ENV GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models
ENV GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins

# Install dependencies for building librealsense with Python bindings
# Clone, build and install librealsense with Python bindings
# Install ROS 2 Nav packages and configure USB rules
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
        cmake \
        build-essential \
        libssl-dev \
        libusb-1.0-0-dev \
        libudev-dev \
        pkg-config \
        libgtk-3-dev \
        python3 \
        python3-setuptools \
        python3-dev \
        python3-pip \
        python3-pybind11 \
        v4l-utils \
        libxinerama-dev \
        libxcursor-dev \
        libcanberra-gtk-module \
        libcanberra-gtk3-module && \
    git clone https://github.com/IntelRealSense/librealsense.git /librealsense && \
    cd /librealsense && \
    git submodule update --init --recursive && \
    ./librealsense/scripts/setup_udev_rules.sh && \
    cd /librealsense/build && \
    cmake ../ \
        -DBUILD_PYTHON_BINDINGS:bool=true \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        -DFORCE_RSUSB_BACKEND=ON \
        -DBUILD_WITH_CUDA:bool=true \
        -DBUILD_EXAMPLES=true \
        -DCMAKE_BUILD_TYPE=release && \
    make uninstall && \
    make clean && \
    make VERBOSE=1 -j$(nproc) pyrealsense2 && \
    make install && \
    mkdir -p /usr/local/lib/python3.10/dist-packages/pyrealsense2 && \
    cp /librealsense/build/release/pyrealsense2.cpython-310-aarch64-linux-gnu.so* /usr/local/lib/python3.10/dist-packages/pyrealsense2/ && \
    cp /librealsense/wrappers/python/pyrealsense2/__init__.py /usr/local/lib/python3.10/dist-packages/pyrealsense2/ && \
    add-apt-repository ppa:borglab/gtsam-release-4.1 && \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-robot-localization \
        ros-${ROS_DISTRO}-slam-toolbox \
        ros-${ROS_DISTRO}-test-msgs \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-libg2o \
        ros-${ROS_DISTRO}-depthai-ros \
        ros-${ROS_DISTRO}-librealsense2* \
        ros-${ROS_DISTRO}-realsense2-* \
        ros-${ROS_DISTRO}-foxglove-bridge \
        ros-${ROS_DISTRO}-tf-transformations \
        ros-${ROS_DISTRO}-tf2* \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-imu-filter-madgwick \
        ros-humble-ament-lint-auto \
        ros-humble-ament-lint-common \
        ros-humble-pcl-conversions \
        ros-humble-ament-copyright \
        ros-humble-ament-flake8 \
        ros-humble-ament-pep257 \
        ros-humble-ros2launch \
        ros-humble-pcl-msgs \
        python3-zmq \
        libyaml-cpp-dev \
        lcov \
        libtins-dev \
        net-tools \
        iproute2 \
        dnsmasq \
        libgtsam-dev \
        libgtsam-unstable-dev \
        libgeographic-dev \
        iputils-ping && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules && \
    /etc/init.d/udev restart && \
    rm -rf /var/lib/apt/lists/* /var/cache/apt/* && \
    apt-get clean

# Add Lidar Host
# RUN echo "169.254.55.220  os-122140001125.local" >> /etc/hosts
# Can be replaced with the --add-host argument https://docs.docker.com/reference/cli/docker/container/run/#add-host

# Setting up the CycloneDDS Config to allow ROS2 to communicate with both the Olive X1 IMU USB-Etherner interface and the Jetson's Ethernet interface
# COPY docker_humble_jetson/cyclonedds.xml /home/${USERNAME}/cyclonedds.xml
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
#     CYCLONEDDS_URI="file:///home/${USERNAME}/cyclonedds.xml"
# ^ this is done at runtime in the run.sh

USER $USERNAME

WORKDIR /home/$USERNAME/dev_ws/src
COPY . .
WORKDIR /home/$USERNAME/dev_ws

# Install jetson-stats
# Install ROS 2 Nav dependencies and configure bashrc
# CAUTION: the exec depend of gazebo-ros in xplore_description's package.xml will cause the build to crash!
# will give the error: can't locate ros-humble-gazebo-ros
# Install additional Python packages and remove confidential source code
RUN sudo pip install -U jetson-stats && \
    rosdep update && \
    rosdep install -r -y -i --from-path src --rosdistro $ROS_DISTRO --skip-keys="ros-humble-gazebo-ros" && \
    echo "if [ -f /home/${USERNAME}/dev_ws/install/setup.bash ]; then source /home/${USERNAME}/dev_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc && \
    pip install --no-cache-dir \
        opencv-python \
        opencv-contrib-python==4.6.0.66 \
        depthai \
        pygame_gui \
        pygame \
        transforms3d \
        pyserial && \
    sudo rm -rf /home/$USERNAME/dev_ws/src/*