# ERC_HD Docker Desktop Environment

This directory contains the Docker configuration for running the **ERC Handling Device (ERC_HD)** system in a desktop development environment using **ROS2 Humble**.

## 🐳 Overview

The desktop Docker environment provides a complete, containerized development and testing platform for the robotic arm system. It includes all necessary dependencies, tools, and configurations to run the HD stack on x86_64 desktop systems with optional GPU acceleration.

## 📁 Files Description

### `Dockerfile` → Defines the containerized HD development environment
- **Base Image**: Extends `ghcr.io/epflxplore/docker_commons:humble-desktop`
- **ROS2 Integration**: Installs MoveIt, hardware interfaces, and control packages
- **Computer Vision**: Includes OpenCV, Intel RealSense, DepthAI, and YOLO
- **System Libraries**: EtherCAT support, USB rules, and development tools
- **Python Packages**: AI/ML libraries (ultralytics, numpy, opencv-python)
- **Security**: Removes confidential source code from final image
- **User Setup**: Configures xplore user with proper permissions

### `run.sh` → Launches the Docker container with GPU support and full configuration
- **GPU Support**: Configures NVIDIA runtime for hardware acceleration
- **X11 Forwarding**: Sets up GUI access for RViz, Gazebo, and other tools
- **Hardware Access**: Mounts `/dev` for USB device access (sensors, serial ports)
- **Network**: Uses host networking for ROS2 communication
- **Volume Mounts**:
  - Source code: `$parent_dir:/home/xplore/dev_ws/src`
  - Home persistence: `hd_humble_desktop_home_volume:/home/xplore`
  - Device access: `/dev:/dev`
- **User Management**: Runs as current user with dialout group access
- **Privileges**: Runs with `--privileged` for hardware access

### `run_no_gpu.sh` → Alternative launcher without GPU acceleration requirements
- **CPU-Only**: Removes NVIDIA runtime and GPU-specific configurations
- **Compatibility**: For systems without NVIDIA GPUs or Docker runtime
- **Same Features**: Maintains all other functionality (X11, hardware, networking)
- **Fallback Option**: Ensures development environment works on any system

### `attach.sh` → Sets up the ROS2 environment inside the container
- **DDS Middleware**: Sets CycloneDDS as ROS2 middleware
- **Display**: Configures X11 display forwarding
- **Python Path**: Sets up Python package paths for custom messages
- **ROS2 Setup**: Sources the workspace setup script

## 🛠️ Docker Image Details

### Base Image
- **Base**: `ghcr.io/epflxplore/docker_commons:humble-desktop`
- **ROS Distribution**: ROS2 Humble Hawksbill
- **OS**: Ubuntu 22.04 (Jammy)

### Key Dependencies

#### ROS2 Packages
- `ros-humble-moveit` - Motion planning framework
- `ros-humble-depthai-ros` - Intel OAK camera integration
- `ros-humble-hardware-interface` - Hardware abstraction layer
- `ros-humble-control-toolbox` - Control algorithms
- `ros-humble-moveit-servo` - Real-time servo control
- `ros-humble-controller-manager` - ROS2 control management
- `ros-humble-gazebo-ros` - Simulation environment
- `ros-humble-rmw-cyclonedx-cpp` - DDS middleware
- `ros-humble-cv-bridge` - OpenCV-ROS bridge

#### Computer Vision & AI
- **Intel RealSense SDK** - Depth camera support
- **OpenCV** (4.7.0.72) - Computer vision library
- **YOLO Ultralytics** - Object detection models
- **DepthAI** - Intel OAK camera AI processing

#### System Libraries
- **EtherCAT Support** - Real-time industrial communication
- **USB/UDev rules** - Device access permissions
- **OpenGL/Mesa** - Graphics rendering
- **GTK3** - GUI framework support

## 🚀 Usage

### Prerequisites

1. **Docker Engine** with BuildKit support
2. **NVIDIA Docker Runtime** (for GPU support)
3. **X11 Server** running on host system
4. **xauth** utility for X11 forwarding

### Quick Start

#### With GPU Support (Recommended)
```bash
cd /path/to/ERC_HD/docker_humble_desktop
./run.sh
```

#### Without GPU Support
```bash
cd /path/to/ERC_HD/docker_humble_desktop
./run_no_gpu.sh
```

### Manual Container Launch

If the scripts don't work, you can manually run:

```bash
# Setup X11 authentication
sudo rm -rf /tmp/.docker.xauth  # If having issues
xauth nlist :0 | sed -n 's/^\..*/ffff&/p' | xauth -f /tmp/.docker.xauth nmerge -
chmod a+r /tmp/.docker.xauth

# Run container
docker run -it \
    --name hd_humble_desktop \
    --rm \
    --runtime=nvidia \
    --gpus all \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=/tmp/.docker.xauth \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /tmp/.docker.xauth:/tmp/.docker.xauth \
    -v /dev:/dev \
    -v $(dirname $(pwd)):/home/xplore/dev_ws/src \
    -v hd_humble_desktop_home_volume:/home/xplore \
    ghcr.io/epflxplore/hd:humble-desktop
```

## 🔧 Container Configuration

### Environment Variables
- **`RMW_IMPLEMENTATION`**: `rmw_cyclonedds_cpp` (DDS middleware)
- **`DISPLAY`**: X11 display forwarding
- **`QT_X11_NO_MITSHM`**: Qt X11 compatibility
- **`PYTHONPATH`**: Custom Python package paths

### Volume Mounts
- **Source Code**: `$(parent_dir):/home/xplore/dev_ws/src`
- **Home Directory**: `hd_humble_desktop_home_volume:/home/xplore`
- **Device Access**: `/dev:/dev` (for hardware interfaces)
- **X11 Socket**: `/tmp/.X11-unix:/tmp/.X11-unix:rw`

### Network Configuration
- **Mode**: `--net=host` (shares host network stack)
- **Purpose**: Enables direct ROS2 communication with external nodes

## 🎯 Development Workflow

### Inside the Container

1. **Build the workspace**:
   ```bash
   cd /home/xplore/dev_ws
   colcon build --symlink-install
   ```

2. **Source the environment**:
   ```bash
   source attach.sh  # Configured environment
   # or manually:
   source install/setup.bash
   ```

3. **Launch the HD system**:
   ```bash
   ros2 launch hd_launch new.launch.py
   ```

4. **Run individual components**:
   ```bash
   # Motor control
   ros2 run ethercat_device_configurator motor_control
   
   # FSM interface
   ros2 run hd_fsm fsm
   
   # Perception
   ros2 run perception perception_node
   ```

### GUI Applications
- **RViz2**: 3D visualization and debugging
- **Gazebo**: Physics simulation
- **RQT**: ROS2 debugging tools
- **Camera viewers**: Real-time camera feeds

## 🐛 Troubleshooting

### X11 Display Issues
```bash
# Reset X11 authentication
sudo rm -rf /tmp/.docker.xauth

# Check X11 forwarding
echo $DISPLAY
xauth list
```

### GPU Access Problems
```bash
# Verify NVIDIA Docker support
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi

# Check container GPU access
nvidia-docker run --rm ghcr.io/epflxplore/hd:humble-desktop nvidia-smi
```

### Permission Errors
```bash
# Fix ownership inside container
sudo chown -R xplore:xplore /home/xplore

# Check device permissions on host
ls -la /dev/ttyUSB* /dev/video*
```

### Network Connectivity
```bash
# Test ROS2 communication
ros2 node list
ros2 topic list

# Check DDS configuration
echo $RMW_IMPLEMENTATION
```

## 📋 Development Tips

### Building and Testing
- Use `colcon build --symlink-install` for faster iteration
- Run `colcon test` for unit testing
- Use `--packages-select` for selective building

### Debugging
- Enable verbose logging: `export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"`
- Use `ros2 run rqt_console rqt_console` for log monitoring
- Attach additional terminals: `docker exec -it hd_humble_desktop bash`

### Performance Optimization
- Monitor resource usage: `docker stats hd_humble_desktop`
- Use `htop` inside container for process monitoring
- Profile with `ros2 run rqt_top rqt_top`

## 🔗 Integration

This Docker environment integrates with:
- **ERC_CS_ControlStation**: Command and telemetry interface
- **ERC_NAV**: Navigation coordination
- **External Hardware**: EtherCAT motors, cameras, sensors
- **Simulation**: Gazebo physics engine

---

**Note**: This environment is designed for development and testing. For production deployment on embedded systems, use the `docker_humble_jetson` configuration.