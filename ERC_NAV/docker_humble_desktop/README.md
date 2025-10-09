# Docker Humble Desktop Environment

This directory contains the Docker configuration and scripts for running the ERC Navigation stack in a desktop development environment using ROS2 Humble.

## Overview

The `docker_humble_desktop` environment is designed for:
- **Development and Testing**: Safe environment for developing navigation algorithms
- **Simulation**: Running navigation stack with Gazebo simulation
- **Desktop Debugging**: Full GUI access for visualization tools (RViz, rqt, etc.)
- **Cross-platform Development**: Consistent environment across different host systems

## Files Description

— **Dockerfile** → Defines the navigation development container image
  * Base Image: Extends `ghcr.io/epflxplore/docker_commons:humble-desktop-nvidia`
  * Navigation Stack: Installs complete Nav2 framework and dependencies
  * Sensor Support: Includes RealSense, OAK-D, and LiDAR drivers
  * Development Tools: Adds debugging and visualization packages
  * Python Libraries: Installs OpenCV, Open3D, and robotics-specific packages
  * Cleanup: Removes source code from final image for security

— **build.sh** → Builds the Docker navigation image
  * Pull Strategy: Forces pull of latest base image with `--pull`
  * Cache: Uses `--no-cache` for clean builds
  * Progress: Shows detailed build output with `--progress=plain`
  * Tagging: Creates image tagged as `ghcr.io/epflxplore/nav:humble-desktop`
  * Context: Uses parent directory as build context

— **run.sh** → Launches the Docker container with proper configuration
  * GUI Support: Configures X11 forwarding for ROS2 GUI tools
  * Hardware Access: Mounts `/dev` for USB device access (sensors, serial ports)
  * Network: Uses host networking for ROS2 communication
  * Volume Mounts:
    · Source code: `$parent_dir:/home/xplore/dev_ws/src`
    · Home persistence: `nav_humble_desktop_home_volume:/home/xplore`
    · Device access: `/dev:/dev`
  * User Management: Runs as current user with dialout group access
  * Privileges: Runs with `--privileged` for hardware access
  * GPU: Enables NVIDIA GPU acceleration with `--runtime=nvidia --gpus all`
  * LiDAR Host: Adds hostname mapping for Ouster LiDAR (`os-122140001125.local`)

— **run_no_gpu.sh** → Alternative container launch without GPU acceleration
  * Same Configuration: Identical to run.sh except GPU settings
  * Compatibility: For systems without NVIDIA GPU or Docker runtime
  * Performance: Reduced performance for 3D visualization and simulation
  * Fallback: Software rendering for OpenGL applications

— **attach.sh** → Connects additional terminal sessions to running container
  * Container Access: Executes bash shell in existing `nav_humble_desktop` container
  * Multi-terminal: Enables multiple concurrent development sessions
  * Environment: Inherits all ROS2 environment variables and workspace setup
  * Usage: Run after container is started with run.sh

## Prerequisites

— **System Requirements**
  * Docker Engine: Version 20.10 or higher
  * NVIDIA Docker Runtime: For GPU-accelerated applications (recommended)
  * X11 Server: For GUI applications (Linux native / macOS with XQuartz)
  * Operating System: Linux (native) or macOS/Windows (with limitations)

— **Hardware Support**
  * GPU: NVIDIA GPU with CUDA support (optional but recommended)
  * Memory: Minimum 8GB RAM, 16GB recommended for full stack
  * Storage: At least 10GB free space for images and persistent volumes
  * USB Ports: Available for sensor connections (cameras, LiDAR, IMU)

## Installation & Setup

— **Step 1: Build the Docker Image**
```bash
cd docker_humble_desktop
./build.sh
```
  * Base Image: Pulls `ghcr.io/epflxplore/docker_commons:humble-desktop-nvidia`
  * Dependencies: Installs ROS2 navigation packages and sensor drivers
  * Configuration: Sets up development environment and user permissions
  * Output: Creates image tagged as `ghcr.io/epflxplore/nav:humble-desktop`

— **Step 2: Launch the Container**
```bash
# With GPU support (recommended)
./run.sh

# Without GPU support (fallback)
./run_no_gpu.sh
```
  * Environment: Automatically configures X11, ROS2, and hardware access
  * Workspace: Mounts parent directory as ROS2 workspace source
  * Persistence: Creates named volume for user home directory

— **Step 3: Access Additional Terminals**
```bash
./attach.sh
```
  * Multi-session: Enables multiple concurrent terminal sessions
  * Environment: Inherits all container environment and workspace setup

## Container Configuration

### Installed Packages

The container includes:

#### ROS2 Navigation Stack
- `ros-humble-navigation2`: Complete Nav2 framework
- `ros-humble-nav2-bringup`: Launch configurations
- `ros-humble-robot-localization`: EKF and sensor fusion
- `ros-humble-slam-toolbox`: SLAM capabilities

#### Sensor Support
- `ros-humble-depthai-ros`: OAK-D camera support
- `ros-humble-librealsense2*`: Intel RealSense cameras
- `ros-humble-realsense2-*`: RealSense ROS2 integration

#### Visualization & Tools
- `ros-humble-foxglove-bridge`: Modern web-based visualization
- `ros-humble-rqt-tf-tree`: Transform tree visualization
- `ros-humble-joint-state-publisher-gui`: Interactive joint control

#### Additional Libraries
- **GTSAM**: Graph-based SLAM library
- **OpenCV**: Computer vision (version 4.6.0.66 with contrib)
- **Open3D**: 3D data processing
- **NumPy**: Fixed at version 1.22.4 for compatibility

### Python Packages
- `opencv-python` and `opencv-contrib-python==4.6.0.66`
- `depthai`: OAK-D camera SDK
- `pygame` and `pygame_gui`: UI development
- `transforms3d`: 3D transformations
- `pyserial`: Serial communication
- `open3d`: Point cloud processing

## Container Features

### Networking
- **Host Network Mode**: Direct access to host network interfaces
- **LiDAR Host Mapping**: Automatic hostname resolution for Ouster LiDAR (`os-122140001125.local:169.254.55.220`)

### Hardware Access
- **USB Devices**: Full access to `/dev` for sensor connectivity
- **GPU Acceleration**: NVIDIA runtime with full GPU access
- **Camera Devices**: Automatic camera device permissions

### X11 Forwarding
- **GUI Support**: Full desktop application support
- **Xauthority**: Secure X11 authentication
- **Display Forwarding**: Native window rendering on host

### Volume Mounts
- **Source Code**: `../:/home/xplore/dev_ws/src` (parent directory mounted as ROS workspace)
- **Home Persistence**: `nav_humble_desktop_home_volume:/home/xplore` (persistent user data)
- **Device Access**: `/dev:/dev` (hardware device access)

## Usage Examples

— **Building and Running Navigation Stack**
```bash
# 1. Start container
./run.sh

# 2. Build workspace (inside container)
cd /home/xplore/dev_ws
colcon build --symlink-install
source install/setup.bash

# 3. Launch navigation
ros2 launch path_planning nav2_real.launch.py
```
  * Workspace: Automatically mounted and configured
  * Dependencies: All required packages pre-installed
  * Environment: ROS2 setup sourced automatically

— **Development Workflow**
  * Host Editing: Modify source code on host system with preferred IDE
  * Container Building: Compile and test inside consistent environment
  * Live Updates: Symlink install enables immediate code changes
  * GUI Tools: RViz, rqt, Gazebo work seamlessly with X11 forwarding

— **Debugging and Monitoring**
```bash
# System introspection
ros2 topic list
ros2 node list
ros2 service list

# Visualization tools
rviz2                    # 3D visualization
rqt                      # Plugin-based GUI tools
rqt_graph               # Node graph visualization
```

## Troubleshooting

— **X11 GUI Issues**
```bash
# Remove corrupted X authority and restart
sudo rm -rf /tmp/.docker.xauth
./run.sh
```
  * Symptoms: GUI applications fail to start or display
  * Cause: Corrupted X11 authentication data
  * Solution: Clean restart of X authority configuration

— **Permission Problems**
```bash
# Fix file ownership (container handles this automatically)
sudo chown -R $USER:$USER /path/to/workspace
```
  * Symptoms: Cannot write files or access devices
  * Cause: UID/GID mismatch between host and container
  * Prevention: Container automatically fixes ownership on startup

— **GPU Not Detected**
```bash
# Test NVIDIA Docker runtime
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi

# Fallback to CPU-only mode
./run_no_gpu.sh
```
  * Symptoms: Poor 3D rendering performance, CUDA errors
  * Cause: Missing NVIDIA Docker runtime or driver issues
  * Fallback: Software rendering mode available

— **Container Startup Failures**
```bash
# Remove conflicting container
docker ps -a | grep nav_humble_desktop
docker rm nav_humble_desktop

# Force rebuild if needed
./build.sh
```
  * Symptoms: Container name conflicts or startup errors
  * Cause: Previous container instances or corrupted image
  * Solution: Clean removal and rebuild

## Environment Variables

The container sets several important environment variables:
- `ROS_DISTRO=humble`: ROS2 distribution
- `DISPLAY`: X11 display forwarding
- `NVIDIA_VISIBLE_DEVICES=all`: GPU access
- `NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute`: GPU capabilities

## Integration with Development Tools

### VS Code
Use the "Dev Containers" extension to develop directly inside the container:
1. Install the Remote-Containers extension
2. Open the workspace folder
3. Select "Reopen in Container"

### IDE Support
The container includes development tools:
- GDB debugger
- CMake build system
- Python development tools
- Git for version control

## Performance Considerations

### Resource Allocation
- The container runs in privileged mode for hardware access
- GPU acceleration significantly improves simulation performance
- Network host mode provides best ROS2 communication performance

### Optimization Tips
- Use SSD storage for Docker volumes
- Allocate sufficient RAM to Docker (8GB minimum)
- Enable GPU passthrough for visualization acceleration

## Security Notes

⚠️ **Security Considerations**:
- Container runs in privileged mode for hardware access
- Full device access is granted (`/dev` mount)
- Host network mode bypasses Docker network isolation
- Only use in trusted development environments

## Related Documentation

- [Main ERC_NAV README](../README.md)
- [Nav2 Documentation](https://navigation.ros.org/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Docker Documentation](https://docs.docker.com/)

## Support

For issues specific to this Docker environment:
1. Check the troubleshooting section above
2. Verify your system meets the prerequisites
3. Consult the main navigation documentation
4. Contact the EPFLXplore navigation team