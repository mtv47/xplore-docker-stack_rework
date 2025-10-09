# Docker Humble Desktop - Development Environment

This folder contains the Docker configuration for the **development environment** of the ERC Science Drill system. This setup is designed for desktop development, testing, and debugging with full GUI support and hardware access.

## Overview

The Docker Humble Desktop environment provides:
- **ROS 2 Humble** development environment
- **GUI applications support** via X11 forwarding
- **Hardware device access** for testing with actual drill hardware
- **CycloneDDS** middleware for reliable communication
- **Development tools** and debugging capabilities

## Files Description

### `Dockerfile`
The main Docker image definition that:
- **Base Image**: Extends `ghcr.io/epflxplore/docker_commons:humble-desktop-nvidia`
- **ROS 2 Setup**: Installs ROS 2 Humble with CycloneDDS middleware
- **Hardware Support**: Configures USB rules for drill motor controllers (Vendor ID: 03e7)
- **Dependencies**: Installs and resolves all ROS package dependencies
- **Security**: Removes source code from final image (confidential code protection)

### `build.sh`
Build script for creating the Docker image:
```bash
./build.sh
```
- Builds image with tag: `ghcr.io/epflxplore/sc:humble-desktop`
- Uses `--no-cache` for fresh builds
- Includes build progress output
- Context: Parent directory (entire ERC_SC_Drill repository)

### `run.sh`
Container execution script with full development features:
```bash
./run.sh
```

**Key Features:**
- **X11 Forwarding**: Complete GUI support for ROS tools (RViz, rqt, etc.)
- **Hardware Access**: Full `/dev` directory mounting for motor controllers
- **Volume Mounting**: 
  - Source code: `parent_dir:/home/xplore/dev_ws/src`
  - Persistent home: `sc_humble_desktop_home_volume:/home/xplore`
- **Network**: Host networking for ROS communication
- **Privileges**: Required for hardware device access

### `attach.sh`
Environment configuration script executed inside the container:
- **RMW Implementation**: Sets CycloneDDS as ROS middleware
- **GUI Setup**: Configures X11 forwarding variables
- **Python Path**: Sets up custom ROS package paths
- **ROS Sourcing**: Sources the workspace setup

### `.dockerignore`
Excludes unnecessary files from build context:
- Version control files (`.git`, `.gitignore`)
- IDE configurations (`.vscode`, `.vs`)
- Build artifacts and temporary files
- Docker-related files to prevent recursion

## Quick Start

### Prerequisites
- **Docker** installed and running
- **X11 server** running (for GUI applications)
- **USB permissions** configured for motor controllers

### 1. Build the Image
```bash
cd docker_humble_desktop
./build.sh
```

### 2. Run the Container
```bash
./run.sh
```

### 3. Inside the Container
The container starts with a configured environment. You can immediately:

```bash
# Build the ROS workspace
cd /home/xplore/dev_ws
colcon build

# Source the workspace
source install/setup.bash

# Launch the drill system
ros2 launch erc_drill drill.launch.xml

# Or run individual components
ros2 run erc_drill SC_drill_fsm
ros2 run erc_drill_interface interface_cs
```

## Development Workflow

### Building and Testing
```bash
# Inside container - build workspace
colcon build --packages-select erc_drill erc_drill_interface custom_msg

# Run tests
colcon test --packages-select erc_drill erc_drill_interface

# Check for issues
colcon test-result --verbose
```

### Debugging
```bash
# Launch with debugging
ros2 launch erc_drill drill.launch.xml --ros-args --log-level debug

# Monitor topics
ros2 topic list
ros2 topic echo /SC/motor_status

# Check node graph
rqt_graph
```

### GUI Tools Available
- **RViz2**: 3D visualization
- **rqt**: ROS graphical tools
- **Gazebo**: Simulation environment
- **ROS 2 CLI tools**: All command-line utilities

## Hardware Integration

### Motor Controllers
The container is configured for drill motor controllers:
- **USB Vendor ID**: 03e7 (configured in udev rules)
- **Device Access**: Full `/dev` directory mounted
- **Permissions**: Container runs with necessary privileges

### Testing Hardware Connection
```bash
# Check USB devices
lsusb | grep 03e7

# Test motor communication
ros2 run erc_drill SC_motor_cmds
```

## Environment Variables

Key environment variables set in the container:

| Variable | Value | Purpose |
|----------|-------|---------|
| `RMW_IMPLEMENTATION` | `rmw_cyclonedx_cpp` | ROS 2 middleware |
| `DISPLAY` | `unix$DISPLAY` | X11 forwarding |
| `QT_X11_NO_MITSHM` | `1` | Qt GUI compatibility |
| `PYTHONPATH` | ROS package paths | Python module resolution |

## Troubleshooting

### X11 Issues
If GUI applications don't work:
```bash
# Clear X authority cache
sudo rm -rf /tmp/.docker.xauth

# Run as root if needed
sudo ./run.sh
```

### Permission Issues
```bash
# Fix file ownership inside container
sudo chown -R xplore:xplore /home/xplore/dev_ws
```

### Hardware Not Detected
```bash
# Check USB rules
cat /etc/udev/rules.d/80-movidius.rules

# Restart udev service
sudo /etc/init.d/udev restart
```

### Build Failures
```bash
# Clean build
rm -rf build install log
colcon build --cmake-clean-cache
```

## Differences from Jetson Version

| Feature | Desktop | Jetson |
|---------|---------|---------|
| **Purpose** | Development/Testing | Production Deployment |
| **GPU** | NVIDIA Desktop | NVIDIA Jetson |
| **GUI** | Full X11 Support | Limited/Headless |
| **Hardware** | Optional | Required |
| **Debugging** | Full Tools | Minimal |
| **Performance** | Development-focused | Optimized |

## Volume Management

### Persistent Data
- **Home Volume**: `sc_humble_desktop_home_volume` preserves user settings
- **Source Mount**: Live development with host filesystem
- **Log Persistence**: ROS logs maintained between runs

### Cleanup
```bash
# Remove container
docker rm sc_humble_desktop

# Remove volume (caution: loses user data)
docker volume rm sc_humble_desktop_home_volume

# Remove image
docker rmi ghcr.io/epflxplore/sc:humble-desktop
```

This development environment provides a complete, isolated ROS 2 workspace for ERC drill development with full hardware and GUI support.