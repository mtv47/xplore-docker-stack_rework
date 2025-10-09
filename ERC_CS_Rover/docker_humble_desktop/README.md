# Docker Humble Desktop Environment

This directory contains the Docker containerization setup for running the EPFLXplore Rover system on desktop/development environments using **ROS 2 Humble**. This containerized environment allows for consistent development, testing, and simulation across different host systems.

## Overview

The Docker environment provides:
- **Isolated Development Environment**: Consistent ROS 2 Humble setup across different machines
- **GUI Application Support**: X11 forwarding for RViz, Gazebo, and other graphical tools
- **Hardware Access**: Direct access to USB devices, cameras, and other peripherals
- **Cross-Platform Compatibility**: Works on Linux desktop systems with X11
- **Development Volume Mounting**: Live code editing with automatic synchronization

## Files Description

### Core Files

#### `Dockerfile`
Multi-stage Docker build configuration that:
- **Base Image**: `ghcr.io/epflxplore/docker_commons:humble-desktop`
- **Dependencies**: Installs ROS 2 Navigation stack, OpenCV, multimedia codecs
- **Python Packages**: Includes networking libraries (aiohttp, aiortc), database support (pymongo), and system monitoring
- **USB Rules**: Intel Movidius Neural Compute Stick support
- **Security**: Removes confidential source code from final image

#### `build.sh`
Docker image build script:
```bash
./build.sh
```
- Builds the image with `--no-cache` for clean builds
- Tags as `ghcr.io/epflxplore/rover:humble-desktop`
- Uses multi-stage build targeting `build-rover` stage

### Execution Scripts

#### `run.sh` (Linux Host)
Main execution script for Linux desktop systems:
```bash
./run.sh
```

**Features**:
- **X11 Forwarding**: Complete GUI application support with proper Xauthority setup
- **Network**: Host networking mode for seamless ROS 2 communication
- **RMW Implementation**: Uses CycloneDDS for reliable communication
- **Device Access**: Full `/dev` directory mounting for hardware access
- **Volume Mounts**:
  - Source code: `parent_dir -> /home/xplore/dev_ws/src`
  - Home persistence: `rover_humble_desktop_home_volume`
  - System integration: `/run/jtop.sock`, accessibility support

#### `run_mac.sh` (macOS Host)
macOS-specific execution script:
```bash
./run_mac.sh
```

**Key Differences from Linux version**:
- **Custom Network**: Uses `docker_humble_desktop_cs_frontend_net` for Control Station integration
- **X11 Compatibility**: Optimized X11 forwarding for macOS X11 servers (XQuartz)
- **Same Volume Structure**: Maintains consistency with Linux environment

#### `run_cameras.sh`
Specialized script for camera system development and testing:
```bash
./run_cameras.sh
```

**Purpose**:
- **Non-interactive Mode**: Runs specific camera node without shell access
- **Automatic Build**: Executes `colcon build` on startup
- **Camera Node Launch**: Starts `ros2 run rover_pkg new_camera_cs`
- **Development Focus**: Optimized for camera subsystem testing

#### `attach.sh`
Environment configuration script for manual container attachment:
```bash
source attach.sh
```

**Configuration**:
- **RMW Setup**: Configures CycloneDDS as default middleware
- **Python Path**: Sets up ROS 2 Python package discovery
- **Display Variables**: Configures X11 forwarding environment
- **ROS 2 Sourcing**: Loads workspace setup files

## Usage Workflows

### Development Workflow

1. **Build the Development Image**:
   ```bash
   cd docker_humble_desktop
   ./build.sh
   ```

2. **Start Development Container**:
   ```bash
   # For Linux
   ./run.sh
   
   # For macOS
   ./run_mac.sh
   ```

3. **Inside the Container**:
   ```bash
   # Build the workspace
   colcon build
   
   # Source the environment
   source install/setup.bash
   
   # Run the rover node
   ros2 launch rover_pkg launch.py
   ```

### Camera Development Workflow

1. **Build the Image** (if not already built):
   ```bash
   ./build.sh
   ```

2. **Run Camera Testing**:
   ```bash
   ./run_cameras.sh
   ```
   This automatically builds and starts the camera node for testing.

### Multi-Container Setup (with Control Station)

For integrated development with the Control Station:

1. **Start Control Station Network**:
   ```bash
   # In ERC_CS_ControlStation/docker_humble_desktop
   docker-compose up
   ```

2. **Connect Rover to CS Network**:
   ```bash
   # Use run_mac.sh which connects to the CS network
   ./run_mac.sh
   ```

## Environment Configuration

### ROS 2 Configuration
- **Distribution**: ROS 2 Humble Hawksbill
- **RMW Implementation**: CycloneDDS (rmw_cyclonedds_cpp)
- **Domain ID**: Default (0) - can be overridden with `ROS_DOMAIN_ID`
- **Network Discovery**: Multicast enabled for cross-container communication

### Volume Mounts Explained

| Host Path | Container Path | Purpose |
|-----------|----------------|---------|
| `../` (parent directory) | `/home/xplore/dev_ws/src` | Live source code editing |
| `rover_humble_desktop_home_volume` | `/home/xplore` | Persistent home directory |
| `/dev` | `/dev` | Hardware device access |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 socket for GUI apps |
| `/run/jtop.sock` | `/run/jtop.sock` | Jetson stats monitoring |

### Network Modes

- **Host Networking** (`run.sh`): Direct access to host network interfaces
- **Custom Network** (`run_mac.sh`): Integration with Control Station Docker network
- **Privileged Mode**: Required for hardware access and system-level operations

## Troubleshooting

### X11 Display Issues
```bash
# Remove corrupted Xauthority file
sudo rm -rf /tmp/.docker.xauth

# Restart the container
./run.sh
```

### Permission Issues
```bash
# The container automatically fixes ownership on startup
# If issues persist, run as root temporarily
sudo ./run.sh
```

### ROS 2 Communication Issues
```bash
# Inside container, check RMW configuration
echo $RMW_IMPLEMENTATION
# Should output: rmw_cyclonedds_cpp

# Test ROS 2 discovery
ros2 node list
ros2 topic list
```

### Container Already Running
```bash
# Stop existing container
docker stop rover_humble_desktop

# Or use --rm flag (already included in scripts)
```

## Development Tips

### Hot Reloading
- Python files are automatically updated due to volume mounting
- C++ files require `colcon build` inside the container
- Use `colcon build --packages-select <package_name>` for faster incremental builds

### Debugging
```bash
# Access running container
docker exec -it rover_humble_desktop /bin/bash

# Check container logs
docker logs rover_humble_desktop

# Monitor resource usage
docker stats rover_humble_desktop
```

### Integration Testing
- Use `run_mac.sh` to connect to Control Station network
- Test bidirectional communication between Rover and CS containers
- Monitor network traffic with ROS 2 introspection tools

## Hardware Requirements

### Host System
- **OS**: Ubuntu 20.04+ or macOS with XQuartz
- **Docker**: Version 20.10+
- **RAM**: Minimum 4GB available for container
- **Storage**: 10GB for image and volumes

### Connected Hardware
- USB cameras (Intel RealSense recommended)
- Gamepad controllers
- Serial devices (for motor controllers)
- Network interfaces (for rover communication)

This containerized environment ensures consistent development experience while providing full access to the rover's hardware capabilities and GUI tools necessary for development and testing.