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

### `Makefile`
Comprehensive command interface (replaces shell scripts) with all development functionality:

**Core Commands:**
- `make build` - Build the SC Drill Desktop image (with cache refresh)
- `make build-quick` - Build using existing cache for faster development
- `make run` - Run interactive container with full development features
- `make run-gpu` - Run with GPU acceleration support
- `make attach` - Attach to running container with environment setup
- `make stop` - Stop the container

**Development Tools:**
- `make gpu-check` - Check GPU availability for potential acceleration
- `make x11-setup` - Setup X11 forwarding for GUI applications

**Maintenance:**
- `make status` - Show containers, volumes, and images status
- `make clean` - Clean up containers, volumes, and images
- `make logs` - Show container logs
- `make docker-check` - Check if Docker is running

**Key Features:**
- **X11 Forwarding**: Complete GUI support for ROS tools (RViz, rqt, etc.)
- **Hardware Access**: Full `/dev` directory mounting for motor controllers
- **GPU Support**: Optional NVIDIA GPU acceleration
- **Volume Mounting**: 
  - Source code: `parent_dir:/home/xplore/dev_ws/src`
  - Persistent home: `sc_humble_desktop_home_volume:/home/xplore`
- **Network**: Host networking for ROS communication
- **Environment Setup**: Automatic ROS 2 environment configuration
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

## Available Make Commands

You can see all available commands by running:
```bash
make help
```

### Quick Command Reference
```bash
# Build and run
make build          # Build with fresh cache
make build-quick    # Build using existing cache
make run            # Run container (standard mode)
make run-gpu        # Run with GPU acceleration

# Development workflow
make attach         # Attach to running container
make stop           # Stop the container
make status         # Show container/volume/image status
make logs           # Show container logs

# Maintenance
make clean          # Clean up (interactive confirmation)
make docker-check   # Verify Docker is running
make x11-setup      # Setup X11 forwarding
make gpu-check      # Check GPU availability
```

## Quick Start

### Prerequisites
- **Docker** installed and running
- **X11 server** running (for GUI applications)
- **USB permissions** configured for motor controllers

### 1. Build the Image
```bash
cd docker_humble_desktop
make build        # Fresh build (recommended for first time)
# or
make build-quick  # Use cache for faster development builds
```

### 2. Run the Container
```bash
make run          # Standard mode
# or
make run-gpu      # With GPU acceleration support
```

### 3. Development Workflow
The container starts with a pre-configured environment:

```bash
# Build the ROS workspace (inside container)
cd /home/xplore/dev_ws
colcon build

# Environment is automatically sourced when you attach
# Launch the drill system
ros2 launch erc_drill drill.launch.xml

# Or use direct ROS commands inside container:
ros2 launch erc_drill drill.launch.xml
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
# Check USB devices for drill controllers
lsusb | grep 03e7

# List all connected devices
ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "No USB serial devices found"

# Test motor communication (inside container)
ros2 run erc_drill SC_motor_cmds
```

## Environment Variables

Key environment variables set in the container:

| Variable | Value | Purpose |
|----------|-------|---------|
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | ROS 2 middleware |
| `DISPLAY` | `unix$DISPLAY` | X11 forwarding |
| `QT_X11_NO_MITSHM` | `1` | Qt GUI compatibility |
| `PYTHONPATH` | ROS package paths | Python module resolution |

## Troubleshooting

### X11 Issues
If GUI applications don't work:
```bash
# Setup X11 forwarding manually
make x11-setup

# Or restart container
make stop
make run
```

### Permission Issues
```bash
# Fix file ownership inside container
sudo chown -R xplore:xplore /home/xplore/dev_ws
```

### Hardware Not Detected
```bash
# Check USB devices (looking for drill motor controllers)
lsusb | grep 03e7

# Verify device access inside container
ls -la /dev/ttyUSB* /dev/ttyACM*

# Check udev rules (if applicable)
cat /etc/udev/rules.d/*drill* 2>/dev/null || echo "No drill-specific udev rules found"
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