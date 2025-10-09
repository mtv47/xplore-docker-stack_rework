# Docker Humble Desktop - Avionics Development Environment

## Overview

The `docker_humble_desktop` folder contains the Docker configuration for the **desktop development environment** of the Avionics ROS2 system. This environment is designed for local development, testing, and debugging of the avionics subsystem on x86/AMD64 desktop machines before deployment to the rover's Jetson platform.

## Purpose

This Docker environment serves multiple purposes:
- **Local Development**: Develop and test avionics code on desktop machines
- **Cross-platform Compatibility**: Ensure code works across different architectures
- **Isolated Environment**: Provide a consistent ROS2 Humble environment regardless of host OS
- **GUI Support**: Enable ROS2 GUI tools and visualization applications
- **Hardware Simulation**: Test avionics code without physical hardware

## Architecture

### Base Image
- **Base**: `ghcr.io/epflxplore/docker_commons:humble-desktop`
- **ROS2 Distribution**: Humble Hawksbill
- **Target Platform**: x86_64/AMD64 desktop systems
- **OS**: Ubuntu 22.04 LTS (Jammy)

### Key Components
- **ROS2 Humble**: Full desktop installation with development tools
- **CycloneDDS**: High-performance DDS middleware for ROS2 communication
- **Python Libraries**: `numpy`, `pymodbus`, `pyserial` for sensor communication
- **ARM Cross-compilation**: GCC ARM toolchain for STM32 development
- **GUI Support**: X11 forwarding for ROS2 visualization tools

## File Structure

```
docker_humble_desktop/
├── Dockerfile          # Docker image definition
├── build.sh           # Image build script
├── run.sh             # Container execution script
├── attach.sh          # Environment setup script
└── README.md          # This documentation
```

## Scripts Description

### 1. `Dockerfile`
**Purpose**: Defines the Docker image with all necessary dependencies

**Key Features**:
- Inherits from Xplore's common Docker base image
- Installs ROS2 Humble with desktop features
- Adds ARM GCC toolchain for STM32 cross-compilation
- Configures development libraries (libyaml-cpp, libserialport)
- Sets up Python environment with required packages
- Configures workspace structure at `/home/xplore/dev_ws`

**Security Note**: Source code is removed from the final image for confidentiality

### 2. `build.sh`
**Purpose**: Builds the Docker image

```bash
./build.sh
```

**What it does**:
- Builds image with tag `ghcr.io/epflxplore/elec:humble-desktop`
- Uses build context from parent directory (`..`)
- Shows detailed build progress with `--progress=plain`

### 3. `run.sh`
**Purpose**: Launches the Docker container with proper configuration

```bash
./run.sh
```

**Key Features**:
- **GUI Support**: Configures X11 forwarding for ROS2 GUI tools
- **Hardware Access**: Mounts `/dev` for USB device access (sensors, serial ports)
- **Network**: Uses host networking for ROS2 communication
- **Volume Mounts**:
  - Source code: `$parent_dir:/home/xplore/dev_ws/src`
  - Home persistence: `elec_humble_desktop_home_volume:/home/xplore`
  - Device access: `/dev:/dev`
- **User Management**: Runs as current user with dialout group access
- **Privileges**: Runs with `--privileged` for hardware access

**Dependencies**:
- Requires `canfd.sh` script (located two directories up)
- Needs X11 server running on host

### 4. `attach.sh`
**Purpose**: Sets up the ROS2 environment inside the container

```bash
source attach.sh
```

**Configuration**:
- **DDS Middleware**: Sets CycloneDDS as ROS2 middleware
- **Display**: Configures X11 display forwarding
- **Python Path**: Sets up Python package paths for custom messages
- **ROS2 Setup**: Sources the workspace setup script

**Usage**: Must be sourced before building or running ROS2 nodes

## Usage Workflow

### 1. Initial Setup
```bash
# Build the Docker image
./build.sh

# Launch the container
./run.sh
```

### 2. Development Workflow
```bash
# Inside the container, set up environment
source src/docker_humble_desktop/attach.sh

# Clean build (if needed)
rm -rf build/ install/ log/
cd ..  # Make sure you're in /home/xplore/dev_ws

# Build custom messages first
colcon build --packages-select custom_msg

# Build all packages
colcon build

# Source environment again
source src/docker_humble_desktop/attach.sh

# Launch the avionics system
ros2 launch avionics_nexus launch.py
```

### 3. Testing and Debugging
```bash
# Monitor topics
ros2 topic list
ros2 topic echo /EL/mass_topic

# Send test commands
ros2 topic pub /EL/servo_req custom_msg/msg/ServoRequest "{id: 1, increment: 50.0, zero_in: false}" --once

# Additional terminals
docker exec -it elec_humble_desktop bash
```

## Hardware Integration

### USB Device Access
The container has access to USB devices through:
- **Device Mounting**: `/dev:/dev` volume mount
- **Privileged Mode**: Full hardware access
- **Dialout Group**: Serial port access permissions

### Simulated Hardware
For development without physical hardware:
- USB device paths can be mocked
- Serial communication can be stubbed
- ROS2 topics can be manually published for testing

## GUI Applications

### X11 Forwarding Setup
The container supports GUI applications through:
- **X11 Socket**: Mounted `/tmp/.X11-unix`
- **Xauth**: Proper X11 authentication
- **Environment Variables**: `DISPLAY`, `QT_X11_NO_MITSHM`

### Supported GUI Tools
- **RViz2**: 3D visualization
- **rqt**: ROS2 GUI tools
- **Gazebo**: Simulation environment
- **PlotJuggler**: Real-time plotting

## Development Features

### Cross-compilation Support
- **ARM GCC Toolchain**: For STM32 microcontroller development
- **Newlib**: ARM-specific C library
- **GDB Multiarch**: Debugging support for ARM targets

### Python Development
- **Pre-installed Packages**: numpy, pymodbus, pyserial
- **Custom Message Support**: Python bindings for custom ROS2 messages
- **Interactive Development**: IPython and debugging tools

## Troubleshooting

### Common Issues

#### GUI Not Working
```bash
# Remove X11 auth file and restart
sudo rm -rf /tmp/.docker.xauth
./run.sh
```

#### USB Devices Not Accessible
- Ensure user is in dialout group on host system
- Check device permissions: `ls -la /dev/ttyUSB*`
- Verify container has privileged access

#### Build Failures
```bash
# Clean workspace
rm -rf build/ install/ log/

# Ensure you're in the right directory
cd /home/xplore/dev_ws

# Build step by step
colcon build --packages-select custom_msg
colcon build
```

#### ROS2 Communication Issues
```bash
# Check ROS2 environment
echo $RMW_IMPLEMENTATION  # Should be rmw_cyclonedx_cpp
ros2 doctor  # Check ROS2 system health
```

### Performance Optimization
- **Resource Limits**: Consider adding memory/CPU limits for resource-constrained systems
- **Volume Caching**: Use cached volumes for better I/O performance
- **Build Caching**: Leverage Docker layer caching for faster rebuilds

## Integration with Other Systems

### Communication with Other Subsystems
- **Network**: Host networking enables communication with other ROS2 nodes
- **Topics**: Standard ROS2 topic-based communication
- **Services**: ROS2 service calls for synchronous communication

### CI/CD Integration
This environment can be used for:
- **Automated Testing**: Run unit tests in isolated environment
- **Build Verification**: Ensure code builds correctly
- **Cross-platform Testing**: Validate code before Jetson deployment

## Security Considerations

- **Privileged Container**: Required for hardware access but increases attack surface
- **Source Code Removal**: Confidential code is removed from final image
- **User Permissions**: Runs as non-root user when possible
- **Network Isolation**: Consider using custom networks for production deployments

## Maintenance

### Updates
- Regularly update base image: `ghcr.io/epflxplore/docker_commons:humble-desktop`
- Keep Python packages updated
- Update ROS2 packages as needed

### Cleanup
```bash
# Remove old containers
docker container prune

# Remove old images
docker image prune

# Remove volume (careful - this deletes persistent data)
docker volume rm elec_humble_desktop_home_volume
```