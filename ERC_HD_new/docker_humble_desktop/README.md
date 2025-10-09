# ERC_HD Docker Desktop Environment# ERC_HD Docker Desktop Environment



This directory contains the Docker configuration for running the **ERC Handling Device (ERC_HD)** system in a desktop development environment using **ROS 2 Humble**. The Heavy Duty (HD) system provides robotic arm control, manipulation, and processing capabilities for the EPFLXplore rover platform.This directory contains the Docker configuration for running the **ERC Handling Device (ERC_HD)** system in a desktop development environment using **ROS2 Humble**.



## 🐳 Overview## 🐳 Overview



The desktop Docker environment provides a complete, containerized development and testing platform for the robotic arm system. It includes all necessary dependencies, tools, and configurations to run the HD stack on x86_64 desktop systems with optional GPU acceleration.The desktop Docker environment provides a complete, containerized development and testing platform for the robotic arm system. It includes all necessary dependencies, tools, and configurations to run the HD stack on x86_64 desktop systems with optional GPU acceleration.



## 📁 Files Description## 📁 Files Description



### Core Files### `Dockerfile` → Defines the containerized HD development environment

- **Base Image**: Extends `ghcr.io/epflxplore/docker_commons:humble-desktop`

#### `Dockerfile`- **ROS2 Integration**: Installs MoveIt, hardware interfaces, and control packages

Defines the containerized HD development environment:- **Computer Vision**: Includes OpenCV, Intel RealSense, DepthAI, and YOLO

- **Base Image**: Extends `ghcr.io/epflxplore/docker_commons:humble-desktop`- **System Libraries**: EtherCAT support, USB rules, and development tools

- **ROS 2 Integration**: Installs MoveIt, hardware interfaces, and control packages- **Python Packages**: AI/ML libraries (ultralytics, numpy, opencv-python)

- **Computer Vision**: Includes OpenCV, Intel RealSense, DepthAI, and YOLO- **Security**: Removes confidential source code from final image

- **System Libraries**: EtherCAT support, USB rules, and development tools- **User Setup**: Configures xplore user with proper permissions

- **Python Packages**: AI/ML libraries (ultralytics, numpy, opencv-python)

- **Security**: Removes confidential source code from final image### `run.sh` → Launches the Docker container with GPU support and full configuration

- **User Setup**: Configures xplore user with proper permissions- **GPU Support**: Configures NVIDIA runtime for hardware acceleration

- **X11 Forwarding**: Sets up GUI access for RViz, Gazebo, and other tools

#### `Makefile`- **Hardware Access**: Mounts `/dev` for USB device access (sensors, serial ports)

Comprehensive management system that consolidates all container operations:- **Network**: Uses host networking for ROS2 communication

```bash- **Volume Mounts**:

make help         # Show all available commands  - Source code: `$parent_dir:/home/xplore/dev_ws/src`

make run          # Run with GPU support  - Home persistence: `hd_humble_desktop_home_volume:/home/xplore`

make run-no-gpu   # Run without GPU support  - Device access: `/dev:/dev`

make attach       # Attach to running container- **User Management**: Runs as current user with dialout group access

make gpu-check    # Check GPU availability- **Privileges**: Runs with `--privileged` for hardware access

```

### `run_no_gpu.sh` → Alternative launcher without GPU acceleration requirements

All environment setup and configuration is handled automatically within the Makefile targets - no external scripts required.- **CPU-Only**: Removes NVIDIA runtime and GPU-specific configurations

- **Compatibility**: For systems without NVIDIA GPUs or Docker runtime

## 🛠️ Usage Workflows- **Same Features**: Maintains all other functionality (X11, hardware, networking)

- **Fallback Option**: Ensures development environment works on any system

### Quick Start

### `attach.sh` → Sets up the ROS2 environment inside the container

1. **View Available Commands**:- **DDS Middleware**: Sets CycloneDDS as ROS2 middleware

   ```bash- **Display**: Configures X11 display forwarding

   make help- **Python Path**: Sets up Python package paths for custom messages

   ```- **ROS2 Setup**: Sources the workspace setup script



2. **Check GPU Support** (optional):## 🛠️ Docker Image Details

   ```bash

   make gpu-check### Base Image

   ```- **Base**: `ghcr.io/epflxplore/docker_commons:humble-desktop`

- **ROS Distribution**: ROS2 Humble Hawksbill

3. **Start HD Desktop Container**:- **OS**: Ubuntu 22.04 (Jammy)

   ```bash

   # With GPU support (recommended for computer vision workloads)### Key Dependencies

   make run

   #### ROS2 Packages

   # Without GPU support (fallback for systems without NVIDIA GPU)- `ros-humble-moveit` - Motion planning framework

   make run-no-gpu- `ros-humble-depthai-ros` - Intel OAK camera integration

   ```- `ros-humble-hardware-interface` - Hardware abstraction layer

- `ros-humble-control-toolbox` - Control algorithms

4. **Attach to Running Container**:- `ros-humble-moveit-servo` - Real-time servo control

   ```bash- `ros-humble-controller-manager` - ROS2 control management

   make attach- `ros-humble-gazebo-ros` - Simulation environment

   ```- `ros-humble-rmw-cyclonedx-cpp` - DDS middleware

- `ros-humble-cv-bridge` - OpenCV-ROS bridge

### Available Make Commands

#### Computer Vision & AI

- **`make run`**: Run HD Desktop container with NVIDIA GPU support and hardware acceleration- **Intel RealSense SDK** - Depth camera support

- **`make run-no-gpu`**: Run HD Desktop container without GPU support (CPU-only fallback)- **OpenCV** (4.7.0.72) - Computer vision library

- **`make attach`**: Attach to running container with proper ROS 2 environment setup- **YOLO Ultralytics** - Object detection models

- **`make stop`**: Stop the HD Desktop container- **DepthAI** - Intel OAK camera AI processing

- **`make clean`**: Remove containers, volumes, and images (with confirmation)

- **`make status`**: Show status of containers, volumes, and images#### System Libraries

- **`make docker-check`**: Verify Docker is running- **EtherCAT Support** - Real-time industrial communication

- **`make x11-setup`**: Setup X11 forwarding manually- **USB/UDev rules** - Device access permissions

- **`make gpu-check`**: Check NVIDIA GPU and Docker runtime availability- **OpenGL/Mesa** - Graphics rendering

- **GTK3** - GUI framework support

### Development Workflow

## 🚀 Usage

1. **Start Development Environment**:

   ```bash### Prerequisites

   # Use GPU version if available for better performance

   make run1. **Docker Engine** with BuildKit support

   ```2. **NVIDIA Docker Runtime** (for GPU support)

3. **X11 Server** running on host system

2. **Inside Container Development**:4. **xauth** utility for X11 forwarding

   ```bash

   # Build the workspace### Quick Start

   colcon build

   #### With GPU Support (Recommended)

   # Source the environment```bash

   source install/setup.bashcd /path/to/ERC_HD/docker_humble_desktop

   ./run.sh

   # Launch HD system components```

   ros2 launch hd_pkg hd_launch.py

   #### Without GPU Support

   # Run individual nodes for testing```bash

   ros2 run hd_pkg manipulator_nodecd /path/to/ERC_HD/docker_humble_desktop

   ```./run_no_gpu.sh

```

3. **Visualization and Debugging**:

   ```bash### Manual Container Launch

   # Launch RViz for visualization

   rviz2If the scripts don't work, you can manually run:

   

   # Monitor robot state```bash

   ros2 topic echo /joint_states# Setup X11 authentication

   sudo rm -rf /tmp/.docker.xauth  # If having issues

   # Test manipulation commandsxauth nlist :0 | sed -n 's/^\..*/ffff&/p' | xauth -f /tmp/.docker.xauth nmerge -

   ros2 action send_goal /move_arm hd_msgs/action/MoveArm "{target_pose: ...}"chmod a+r /tmp/.docker.xauth

   ```

# Run container

### GPU vs No-GPU Usagedocker run -it \

    --name hd_humble_desktop \

#### When to use `make run` (with GPU):    --rm \

- **Computer Vision**: Object detection, image processing, depth estimation    --runtime=nvidia \

- **Simulation**: Gazebo with realistic rendering    --gpus all \

- **AI/ML Workloads**: Neural network inference, YOLO detection    --privileged \

- **Performance**: Faster processing for intensive visual tasks    --net=host \

    -e DISPLAY=$DISPLAY \

#### When to use `make run-no-gpu`:    -e QT_X11_NO_MITSHM=1 \

- **Development Machines**: Systems without NVIDIA GPUs    -e XAUTHORITY=/tmp/.docker.xauth \

- **Basic Testing**: Algorithm development without visual processing    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \

- **CI/CD Environments**: Automated testing pipelines    -v /tmp/.docker.xauth:/tmp/.docker.xauth \

- **Compatibility**: Ensuring code works on any system    -v /dev:/dev \

    -v $(dirname $(pwd)):/home/xplore/dev_ws/src \

## 🔧 Environment Configuration    -v hd_humble_desktop_home_volume:/home/xplore \

    ghcr.io/epflxplore/hd:humble-desktop

### ROS 2 Configuration```

- **Distribution**: ROS 2 Humble Hawksbill

- **RMW Implementation**: CycloneDDS (rmw_cyclonedds_cpp)## 🔧 Container Configuration

- **Domain ID**: Default (0) - configurable via `ROS_DOMAIN_ID`

- **Architecture**: x86_64 desktop systems### Environment Variables

- **`RMW_IMPLEMENTATION`**: `rmw_cyclonedds_cpp` (DDS middleware)

### Automatic Environment Setup- **`DISPLAY`**: X11 display forwarding

The Makefile automatically configures all necessary environment variables:- **`QT_X11_NO_MITSHM`**: Qt X11 compatibility

- **RMW_IMPLEMENTATION**: `rmw_cyclonedds_cpp`- **`PYTHONPATH`**: Custom Python package paths

- **DISPLAY**: X11 display forwarding for GUI applications

- **QT_X11_NO_MITSHM**: X11 compatibility for Qt applications### Volume Mounts

- **PYTHONPATH**: Complete ROS 2 Python package paths for custom messages- **Source Code**: `$(parent_dir):/home/xplore/dev_ws/src`

- **GPU Support**: NVIDIA runtime and GPU access (when using `make run`)- **Home Directory**: `hd_humble_desktop_home_volume:/home/xplore`

- **Device Access**: `/dev:/dev` (for hardware interfaces)

### Volume Mounts Explained- **X11 Socket**: `/tmp/.X11-unix:/tmp/.X11-unix:rw`



| Host Path | Container Path | Purpose |### Network Configuration

|-----------|----------------|---------|- **Mode**: `--net=host` (shares host network stack)

| `../` (parent directory) | `/home/xplore/dev_ws/src` | Live source code editing |- **Purpose**: Enables direct ROS2 communication with external nodes

| `hd_humble_desktop_home_volume` | `/home/xplore` | Persistent home directory |

| `/dev` | `/dev` | Hardware device access (USB, serial) |## 🎯 Development Workflow

| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 socket for GUI applications |

| `/run/user/1000/at-spi` | `/run/user/1000/at-spi` | Accessibility support |### Inside the Container



### Hardware Access1. **Build the workspace**:

- **Privileged Mode**: Full hardware access for EtherCAT, USB devices   ```bash

- **Host Networking**: Direct network interface access for real-time communication   cd /home/xplore/dev_ws

- **Device Mounting**: Complete `/dev` access for sensors and actuators   colcon build --symlink-install

- **GPU Access**: NVIDIA GPU acceleration for computer vision (when available)   ```



## 🔍 Troubleshooting2. **Source the environment**:

   ```bash

### Container Issues   source attach.sh  # Configured environment

```bash   # or manually:

# Check container status   source install/setup.bash

make status   ```



# Stop and restart if needed3. **Launch the HD system**:

make stop   ```bash

make run   ros2 launch hd_launch new.launch.py

```   ```



### GPU Issues4. **Run individual components**:

```bash   ```bash

# Check GPU availability   # Motor control

make gpu-check   ros2 run ethercat_device_configurator motor_control

   

# If GPU issues, use CPU fallback   # FSM interface

make run-no-gpu   ros2 run hd_fsm fsm

```   

   # Perception

### X11 Display Issues   ros2 run perception perception_node

```bash   ```

# Setup X11 manually

make x11-setup### GUI Applications

- **RViz2**: 3D visualization and debugging

# Restart container- **Gazebo**: Physics simulation

make stop- **RQT**: ROS2 debugging tools

make run- **Camera viewers**: Real-time camera feeds

```

## 🐛 Troubleshooting

### Hardware Access Problems

```bash### X11 Display Issues

# Verify device permissions (run on host)```bash

ls -la /dev/ttyUSB*# Reset X11 authentication

ls -la /dev/video*sudo rm -rf /tmp/.docker.xauth



# Check if user is in dialout group (on host)# Check X11 forwarding

groups $USERecho $DISPLAY

xauth list

# The container runs privileged for hardware access```

```

### GPU Access Problems

### ROS 2 Communication Issues```bash

```bash# Verify NVIDIA Docker support

# Inside container, verify configurationdocker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi

echo $RMW_IMPLEMENTATION

# Should output: rmw_cyclonedds_cpp# Check container GPU access

nvidia-docker run --rm ghcr.io/epflxplore/hd:humble-desktop nvidia-smi

# Test ROS 2 communication```

ros2 node list

ros2 topic list### Permission Errors

ros2 service list```bash

```# Fix ownership inside container

sudo chown -R xplore:xplore /home/xplore

## 🚀 Development Tips

# Check device permissions on host

### Performance Optimizationls -la /dev/ttyUSB* /dev/video*

- **Use GPU version** when available for computer vision tasks```

- **GPU acceleration** significantly improves object detection and image processing

- **Real-time performance** for manipulation control loops### Network Connectivity

```bash

### Hardware Integration# Test ROS2 communication

- **EtherCAT Support**: Direct hardware communication with robotic armros2 node list

- **USB Device Access**: Cameras, sensors, and communication devicesros2 topic list

- **Serial Communication**: Direct access to serial ports and devices

# Check DDS configuration

### Development Environmentecho $RMW_IMPLEMENTATION

- **Hot Reloading**: Python files update automatically due to volume mounting```

- **C++ Development**: Use `colcon build` for incremental compilation

- **Visualization**: Full X11 forwarding supports RViz, Gazebo, and custom GUIs## 📋 Development Tips



### Container Management### Building and Testing

- Use `make status` to monitor container health and resource usage- Use `colcon build --symlink-install` for faster iteration

- Use `make clean` to completely reset the development environment- Run `colcon test` for unit testing

- Use `make attach` to open additional terminals in the running container- Use `--packages-select` for selective building



## 📋 Migration from Shell Scripts### Debugging

- Enable verbose logging: `export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"`

This Makefile replaces the following shell scripts with **zero external dependencies**:- Use `ros2 run rqt_console rqt_console` for log monitoring

- `run.sh` → `make run` (with GPU support)- Attach additional terminals: `docker exec -it hd_humble_desktop bash`

- `run_no_gpu.sh` → `make run-no-gpu` (without GPU support)

- `attach.sh` → `make attach` (environment setup now inline)### Performance Optimization

- Monitor resource usage: `docker stats hd_humble_desktop`

All functionality has been preserved and enhanced with:- Use `htop` inside container for process monitoring

- **Zero External Scripts**: Everything is self-contained in the Makefile- Profile with `ros2 run rqt_top rqt_top`

- **GPU Detection**: Automatic checking of GPU availability

- **Better Error Handling**: Comprehensive status reporting and validation## 🔗 Integration

- **Unified Interface**: Consistent commands across development scenarios

- **Automatic Environment Setup**: No manual configuration requiredThis Docker environment integrates with:

- **Enhanced Debugging**: Built-in status and diagnostic commands- **ERC_CS_ControlStation**: Command and telemetry interface

- **ERC_NAV**: Navigation coordination

## 🏗️ System Requirements- **External Hardware**: EtherCAT motors, cameras, sensors

- **Simulation**: Gazebo physics engine

### Minimum Requirements

- **Docker**: Docker Engine with compose support---

- **X11**: X11 server for GUI applications (XQuartz on macOS)

- **Memory**: 4GB RAM minimum, 8GB recommended**Note**: This environment is designed for development and testing. For production deployment on embedded systems, use the `docker_humble_jetson` configuration.
- **Storage**: 10GB available space for images and volumes

### Recommended for Full Features
- **NVIDIA GPU**: For computer vision and simulation acceleration
- **NVIDIA Docker Runtime**: For GPU passthrough support
- **USB Ports**: For hardware device connectivity
- **Network**: Stable network connection for ROS 2 communication