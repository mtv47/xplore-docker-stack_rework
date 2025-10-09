# Docker Humble Jetson - Production Deployment Environment

This folder contains the Docker configuration for the **production deployment environment** of the ERC Science Drill system. This setup is optimized for NVIDIA Jetson platforms and designed for autonomous operation in competition and field environments.

## Overview

The Docker Humble Jetson environment provides:
- **ROS 2 Humble** optimized for ARM64/Jetson architecture
- **Production-ready deployment** with autonomous startup capabilities
- **Computer vision support** with OpenCV and image processing libraries
- **Competition-specific features** including photo capture integration
- **Robust container management** with automatic restart and monitoring

## Files Description

### `Dockerfile`
Production-optimized Docker image definition:
- **Base Image**: `ghcr.io/epflxplore/docker_commons:humble-jetson` (ARM64 optimized)
- **Computer Vision**: Pre-installed OpenCV 4.7.0.72 with contrib modules
- **Input Handling**: evdev library for hardware input devices
- **Scientific Computing**: NumPy 1.24.0 for data processing
- **Hardware Integration**: USB rules for drill motor controllers
- **Optimized Build**: No dependency resolution during runtime

### `Makefile`
Comprehensive production deployment interface (replaces all shell scripts):

**Core Commands:**
- `make build` - Build production-optimized Jetson image
- `make run` - Run interactive container for development/testing
- `make run-drill` - Start drill stack automatically (production mode)
- `make attach` - Attach to running container with environment setup
- `make stop` - Stop the container

**Production Management:**
- `make launch-drill` - Launch drill stack in running container
- `make stop-drill` - Stop the drill stack (alias for stop)
- `make logs` - Show container logs

**Development Tools:**
- `make dev-shell` - Development shell without auto-launch
- `make drill-test` - Run drill subsystem tests
- `make drill-status` - Check drill hardware status
- `make hardware-check` - Check Jetson hardware and GPU availability

**Maintenance:**
- `make status` - Show containers, volumes, and images status
- `make clean` - Clean up containers, volumes, and images

**Key Production Features:**
- **Automatic Launch**: Directly executes `ros2 launch erc_drill drill.launch.xml`
- **Container Management**: Checks if container is running and reuses or creates new
- **Non-Interactive Mode**: Runs with `-i` flag for daemon-like operation
- **Smart Execution**: Attaches to existing container or starts new one
- **Photo Integration**: Mounts competition photo directory

**Production Workflow:**
1. Checks container status
2. If not running: Creates new container and launches drill system
- **Container Lifecycle Management**: Intelligent container state management
- **Automatic Launch**: Direct drill system startup with `ros2 launch erc_drill drill.launch.xml`
- **Production Monitoring**: Container health checks and restart capabilities
- **Volume Management**: Persistent home directory and competition photo access
- **Environment Setup**: Automatic ROS 2 environment configuration

## Production Deployment

### SystemD Service Integration
This Docker setup integrates with systemd service for autonomous operation:

```bash
# Example systemd service (drillnode.service)
[Unit]
Description=ERC Science Drill System
After=docker.service
Requires=docker.service

[Service]
Type=forking
WorkingDirectory=/path/to/ERC_SC_Drill_new/docker_humble_jetson
ExecStart=/usr/bin/make run-drill
ExecStop=/usr/bin/make stop
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

### Autonomous Startup
```bash
# Install the service
sudo cp drillnode.service /etc/systemd/system/
sudo systemctl daemon-reload

# Enable automatic startup
sudo systemctl enable drillnode.service

# Start service manually
sudo systemctl start drillnode.service

# Check status
sudo systemctl status drillnode.service
```

### Competition Deployment Workflow
1. **Build Image**: `make build` (typically done in CI/CD)
2. **Deploy to Jetson**: Transfer image to competition hardware
3. **Configure Service**: Install and enable systemd service
4. **Launch**: Service automatically starts drill system on boot

## Hardware Optimization

### Jetson-Specific Features
- **ARM64 Architecture**: Native compilation for Jetson Xavier/Orin
- **GPU Integration**: CUDA support for computer vision tasks
- **Memory Optimization**: Tuned for Jetson memory constraints
- **Power Management**: Optimized for mobile/battery operation

### Computer Vision Pipeline
```python
# Pre-installed packages for image processing
import cv2              # OpenCV 4.7.0.72
import numpy as np      # NumPy 1.24.0
from evdev import InputDevice  # Hardware input handling
```

### Photo Competition Integration
- **Mount Point**: `/home/xplore/dev_ws/photos_competition`
- **Host Path**: `/home/xplore-hd/Documents/photos_competition`
- **Purpose**: Stores sample images for competition scoring

## Container Management

### Production Container Lifecycle
```bash
# Check container status
docker ps | grep sc_humble_jetson

# View container logs
docker logs sc_humble_jetson

# Execute commands in running container
docker exec -it sc_humble_jetson bash

# Monitor resource usage
docker stats sc_humble_jetson
```

### Volume Management
| Volume | Purpose | Persistence |
|--------|---------|-------------|
| `sc_humble_jetson_home_volume` | User settings, logs | Persistent |
| Source mount | Live code deployment | Host filesystem |
| Photos mount | Competition images | Host filesystem |
| `/dev` mount | Hardware access | Host devices |

## Monitoring and Debugging

### Production Monitoring
```bash
# Check drill system status
docker exec sc_humble_jetson ros2 topic list

# Monitor drill commands
docker exec sc_humble_jetson ros2 topic echo /SC/drill_cmd

# Check motor status
docker exec sc_humble_jetson ros2 topic echo /SC/motor_status

# View system logs
make logs
```

### Debug Mode
For debugging in production:
```bash
# Stop production container
make stop

# Run interactive session
make run

# Or use development shell
make dev-shell

# Manual launch for debugging
make launch-drill

# Or manually inside container
ros2 launch erc_drill drill.launch.xml --ros-args --log-level debug
```

## Performance Optimization

### Resource Limits
The container is optimized for Jetson hardware:
- **Memory**: Efficient memory usage for embedded systems
- **CPU**: ARM64 native performance
- **GPU**: CUDA acceleration for vision tasks
- **Network**: Host networking for minimal latency

### Competition Readiness
- **Fast Startup**: Pre-built image with all dependencies
- **Reliable Operation**: Robust error handling and recovery
- **Minimal Overhead**: Streamlined for competition performance
- **Hardware Integration**: Direct hardware access for motors and sensors

## Differences from Desktop Version

| Feature | Jetson (Production) | Desktop (Development) |
|---------|--------------------|-----------------------|
| **Architecture** | ARM64 | x86_64 |
| **Purpose** | Competition/Field | Development/Testing |
| **Startup** | Autonomous | Manual |
| **Interactivity** | Non-interactive | Full interactive |
| **Dependencies** | Pre-installed | Development tools |
| **GPU Support** | Jetson CUDA | Desktop NVIDIA |
| **Container Management** | Production scripts | Development scripts |
| **Photo Integration** | Competition photos | Development data |

## Troubleshooting

### Container Issues
```bash
# Force container recreation
make stop
make clean
make run-drill

# Check container health
make status
```
docker inspect sc_humble_jetson

# View detailed logs
docker logs --details sc_humble_jetson
```

### Hardware Issues
```bash
# Check USB devices
docker exec sc_humble_jetson lsusb

# Test motor controllers
docker exec sc_humble_jetson ros2 run erc_drill SC_motor_cmds

# Check udev rules
docker exec sc_humble_jetson cat /etc/udev/rules.d/80-movidius.rules
```

### Performance Issues
```bash
# Monitor resources
docker stats sc_humble_jetson

# Check ROS performance
docker exec sc_humble_jetson ros2 doctor

# Network diagnostics
docker exec sc_humble_jetson ros2 daemon status
```

## Security Considerations

- **Privileged Access**: Required for hardware control
- **Host Networking**: Necessary for ROS communication
- **Device Access**: Full `/dev` access for motor controllers
- **Source Protection**: Confidential code removed from final image
- **Volume Isolation**: User data isolated in named volumes

This production environment ensures reliable, autonomous operation of the ERC drill system in competition environments while maintaining the flexibility needed for field debugging and monitoring.