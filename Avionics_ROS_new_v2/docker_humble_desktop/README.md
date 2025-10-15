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
├── Makefile           # Build and run commands (replaces shell scripts)
└── README.md          # This documentation
```

## Makefile Commands

All functionality is now consolidated into a comprehensive Makefile. Use `make help` to see all available commands.

### Core Commands

**Building and Running**:
- `make build` - Build the Avionics ROS Desktop Docker image
- `make run` - Run interactive container with CAN-FD and serial support
- `make attach` - Attach to running container with full ROS environment
- `make stop` - Stop the container

**Development**:
- `make dev-shell` - Run development shell without user switching
- `make hardware-check` - Check CAN and serial hardware availability
- `make status` - Show containers, volumes, and images status

**Maintenance**:
- `make clean` - Remove containers, volumes, and images
- `make docker-check` - Verify Docker is running
- `make x11-setup` - Setup X11 forwarding for GUI applications

### Key Features

**Dockerfile**:
- Inherits from Xplore's common Docker base image
- Installs ROS2 Humble with desktop features
- Adds ARM GCC toolchain for STM32 cross-compilation
- Configures development libraries (libyaml-cpp, libserialport)
- Sets up Python environment with required packages
**Container Features**:
- Configures workspace structure at `/home/xplore/dev_ws`
- Automated environment setup through Makefile
- Comprehensive hardware support for CAN-FD and serial devices
- X11 forwarding for GUI applications
- Automatic user permission management

**Security Note**: Source code is removed from the final image for confidentiality

## Usage Workflow

### 1. Initial Setup
```bash
# Build the Docker image
make build

# Launch the container
make run
```

### 2. Development Workflow
```bash
# Inside the container, environment is automatically configured
# The make attach command sets up the ROS2 environment

# Clean build (if needed)
rm -rf build/ install/ log/
cd ..  # Make sure you're in /home/xplore/dev_ws

# Build custom messages first
colcon build --packages-select custom_msg

# Build all packages
colcon build

# The environment is automatically configured when using make attach
```

### 3. Attach to Running Container
```bash
# Attach to container with full ROS environment
make attach

# The environment includes:
# - DDS Middleware: CycloneDDS as ROS2 middleware
# - Display: X11 display forwarding configured
# - Python Path: Custom message paths configured
# - ROS2 Setup: Workspace setup script sourced

# Launch the avionics system
ros2 launch avionics_nexus launch.py
```

### 4. Testing and Debugging
```bash
# Monitor topics
ros2 topic list
ros2 topic echo /EL/mass_topic

# Send test commands
ros2 topic pub /EL/servo_req custom_msg/msg/ServoRequest "{id: 1, increment: 50.0, zero_in: false}" --once

# Additional terminals - use make attach in new terminal
make attach
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
# Setup X11 forwarding manually
make x11-setup

# Or restart container
make stop
make run
```

#### USB Devices Not Accessible
- Ensure user is in dialout group on host system: `make hardware-check`
- Check device permissions: `ls -la /dev/ttyUSB*`
- Verify container has privileged access

#### CAN-FD Setup Issues
- The Makefile automatically handles missing canfd.sh script
- Check if script exists: `ls -la ../../canfd.sh`
- Configure CAN manually if needed

#### Build Failures
```bash
# Clean workspace (inside container)
rm -rf build/ install/ log/

# Ensure you're in the right directory
cd /home/xplore/dev_ws

# Build step by step
colcon build --packages-select custom_msg
colcon build
```

#### Container Not Starting
```bash
# Check Docker status
make docker-check

# Check container status
make status

# Clean up and rebuild
make clean
make build
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