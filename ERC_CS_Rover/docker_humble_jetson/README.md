# Docker Humble Jetson Environment

This directory contains the Docker containerization setup for running the EPFLXplore Rover system on **NVIDIA Jetson** embedded platforms using **ROS 2 Humble**. This production-ready environment is optimized for deployment on the actual rover hardware with ARM64 architecture support, hardware acceleration, and real-time performance capabilities.

## Overview

The Jetson Docker environment provides:
- **Production Deployment**: Optimized for NVIDIA Jetson Nano, Xavier NX, AGX Xavier, and Orin platforms
- **ARM64 Architecture**: Native ARM64 container support for Jetson hardware
- **Hardware Acceleration**: GPU access for computer vision and AI inference
- **Real-Time Performance**: Low-latency configuration for competition scenarios
- **Hardware Integration**: Direct access to cameras, sensors, motors, and communication devices
- **Competition Ready**: Autonomous launch capabilities with systemd service integration

## Files Description

### Core Files

#### `Dockerfile`
Production-optimized Docker build configuration:
- **Base Image**: `ghcr.io/epflxplore/docker_commons:humble-jetson` (ARM64 optimized)
- **ROS 2 Packages**: Navigation2 stack, OpenCV with CUDA support
- **Hardware Support**: 
  - Intel RealSense cameras (`pyrealsense2`)
  - Intel Movidius Neural Compute Stick (USB rules)
  - NVIDIA GPU acceleration
- **Python Dependencies**: 
  - Networking: `aiohttp`, `aiortc`, `aiohttp_cors`
  - System monitoring: `ping3`, `tornado` 
  - Database: `pymongo`, `bson`
  - Computer vision: `numpy`, `opencv`
- **Security**: Confidential source code removed from final image

#### `build.sh`
Jetson-specific image build script:
```bash
./build.sh
```
- Builds ARM64 architecture image
- Tags as `ghcr.io/epflxplore/rover:humble-jetson`
- Optimized build process for Jetson platforms

### Execution Scripts

#### `run.sh` (Interactive Development)
Interactive development and debugging script:
```bash
./run.sh
```

**Features**:
- **Interactive Shell**: Full bash access for development and debugging
- **X11 Forwarding**: GUI application support (RViz, debugging tools)
- **Hardware Access**: Complete `/dev` directory mounting
- **Jetson Monitoring**: `jtop` socket integration for system monitoring
- **Competition Photos**: Special volume mount for `/home/xplore/photos_competition`
- **Development Volumes**:
  - Source code: Live mounting for development
  - Home persistence: `rover_humble_jetson_home_volume`

#### `run_rover.sh` (Production Launch)
**Primary production script** for autonomous rover operation:
```bash
./run_rover.sh
```

**Key Features**:
- **Smart Container Management**: Checks if container is running, starts new or attaches to existing
- **Automatic Launch**: Directly executes `ros2 launch rover_pkg launch.py`
- **Non-Interactive Mode**: Optimized for systemd service integration
- **Fault Tolerance**: Handles container state transitions gracefully
- **Production Environment**: Minimal overhead for competition performance

**Container Logic**:
```bash
# If container not running -> Start new container with rover launch
# If container already running -> Attach and execute rover launch
```

### Management Scripts

#### `attach.sh`
Environment configuration for manual container access:
```bash
source attach.sh
```

**Configuration**:
- **RMW Setup**: CycloneDDS middleware for reliable communication
- **Python Environment**: ROS 2 package path configuration
- **X11 Variables**: Display forwarding setup
- **Workspace Sourcing**: Automatic ROS 2 environment loading

#### `stop_docker_rover.sh`
Safe rover container shutdown:
```bash
./stop_docker_rover.sh
```
- **Container Check**: Verifies container is running before stopping
- **Graceful Shutdown**: Uses `docker stop` for clean termination
- **State Management**: Ensures proper cleanup

#### `stop_docker_cameras.sh`
Camera-specific container management:
```bash
./stop_docker_cameras.sh
```
- **Target**: Stops `rover_humble_jetson_2` camera container
- **Specialized Function**: Handles separate camera system containers
- **Safety Check**: Validates container state before operation

## Jetson-Specific Optimizations

### Hardware Integration

#### GPU Acceleration
- **CUDA Support**: OpenCV compiled with CUDA for computer vision acceleration
- **Neural Processing**: Intel Movidius NCS support for AI inference
- **Memory Management**: Optimized for Jetson's unified memory architecture

#### System Monitoring
- **JTop Integration**: Real-time Jetson hardware monitoring
  - GPU utilization and memory usage
  - CPU frequency and temperature
  - Power consumption and thermal throttling
  - Fan speed and system health
- **Volume Mount**: `/run/jtop.sock` for system stats access

#### Device Access
- **Complete Hardware Access**: Full `/dev` directory mounting
- **USB Device Rules**: Pre-configured for competition hardware
- **Camera Integration**: Direct access to CSI and USB cameras
- **Serial Communication**: Motor controller and sensor interfaces

### Network Configuration

#### Host Networking
- **Performance**: Eliminates Docker network overhead
- **ROS 2 Discovery**: Direct multicast communication
- **Low Latency**: Critical for real-time rover control
- **Competition Network**: Seamless integration with field communication

#### Middleware Optimization
- **CycloneDDS**: High-performance DDS implementation
- **Real-Time Priorities**: Optimized for time-critical operations
- **Memory Efficiency**: Reduced overhead for embedded deployment

## Production Deployment

### Systemd Integration

The rover system integrates with systemd for autonomous startup:

```bash
# Service file: /etc/systemd/system/rovernode.service
[Unit]
Description=Rover Node Service
After=docker.service
Requires=docker.service

[Service]
ExecStart=/bin/bash /home/xplore/ERC_CS_Rover/docker_humble_jetson/run_rover.sh
ExecStop=/usr/bin/docker stop rover_humble_jetson

[Install]
WantedBy=default.target
```

### Automatic Startup Workflow

1. **System Boot**: Jetson powers on
2. **Docker Service**: SystemD starts Docker daemon
3. **Rover Service**: SystemD executes `run_rover.sh`
4. **Container Check**: Script determines container state
5. **Rover Launch**: Automatic execution of rover node
6. **Competition Ready**: Full system operational

### Competition Configuration

#### Volume Mounts for Competition
```bash
# Competition photos storage
/home/xplore/photos_competition -> /home/xplore/dev_ws/photos_competition

# Source code (live development)
$PARENT_DIR -> /home/xplore/dev_ws/src

# Persistent home directory
rover_humble_jetson_home_volume -> /home/xplore
```

#### Environment Variables
```bash
DISPLAY=unix$DISPLAY          # X11 forwarding
QT_X11_NO_MITSHM=1           # Qt compatibility
XAUTHORITY=$XAUTH            # X11 authentication
```

## Usage Workflows

### Development Workflow

1. **Build Jetson Image**:
   ```bash
   cd docker_humble_jetson
   ./build.sh
   ```

2. **Interactive Development**:
   ```bash
   ./run.sh
   # Full interactive shell with GUI support
   ```

3. **Inside Container**:
   ```bash
   colcon build
   source install/setup.bash
   ros2 launch rover_pkg launch.py
   ```

### Production Deployment

1. **Deploy to Jetson**:
   ```bash
   # Copy entire repository to Jetson
   scp -r ERC_CS_Rover xplore@jetson-ip:/home/xplore/
   ```

2. **Build Production Image**:
   ```bash
   ssh xplore@jetson-ip
   cd /home/xplore/ERC_CS_Rover/docker_humble_jetson
   ./build.sh
   ```

3. **Test Production Launch**:
   ```bash
   ./run_rover.sh
   # Should automatically launch rover system
   ```

4. **Enable Systemd Service**:
   ```bash
   sudo cp ../rovernode.service /etc/systemd/system/
   sudo systemctl enable rovernode.service
   sudo systemctl start rovernode.service
   ```

### Debugging and Monitoring

#### System Monitoring
```bash
# Monitor Jetson performance
sudo jtop

# Check container status
docker ps
docker logs rover_humble_jetson

# Monitor ROS 2 system
./run.sh
# Inside container:
ros2 node list
ros2 topic list
ros2 topic echo /Rover/RoverState
```

#### Hardware Debugging
```bash
# Check camera devices
ls /dev/video*

# Test RealSense cameras
realsense-viewer

# Monitor system resources
htop
nvidia-smi  # If available on Jetson
```

## Performance Optimization

### Memory Management
- **Swap Configuration**: Optimize swap for competition scenarios
- **Memory Limits**: Container memory limits for stability
- **Garbage Collection**: Python GC tuning for real-time performance

### CPU Optimization
- **Process Priorities**: Real-time scheduling for critical processes
- **CPU Affinity**: Bind processes to specific cores
- **Frequency Scaling**: Performance governor during competition

### Network Performance
- **Buffer Sizes**: Optimized network buffer configuration
- **Quality of Service**: ROS 2 QoS profiles for reliable communication
- **Bandwidth Management**: Prioritize critical data streams

## Troubleshooting

### Container Issues
```bash
# Container won't start
sudo systemctl status docker
sudo systemctl restart docker

# Permission issues
sudo chown -R xplore:xplore /home/xplore/dev_ws

# Clean restart
./stop_docker_rover.sh
docker system prune -f
./run_rover.sh
```

### Hardware Issues
```bash
# Camera not detected
ls /dev/video*
lsusb | grep Intel  # For RealSense cameras

# USB device permissions
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Performance Issues
```bash
# Check system load
sudo jtop
top

# Monitor temperatures
cat /sys/devices/virtual/thermal/thermal_zone*/temp

# Check for thermal throttling
dmesg | grep -i thermal
```

### Network Communication
```bash
# ROS 2 discovery issues
export ROS_DOMAIN_ID=0
ros2 daemon stop
ros2 daemon start

# Test inter-node communication
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener
```

## Hardware Requirements

### Jetson Platform Support
- **Jetson Nano**: 4GB RAM minimum (Developer Kit or Production Module)
- **Jetson Xavier NX**: 8GB RAM recommended for full functionality
- **Jetson AGX Xavier**: Optimal performance for computer vision tasks
- **Jetson Orin**: Latest platform with enhanced AI capabilities

### Storage Requirements
- **SD Card/eMMC**: Minimum 32GB, 64GB+ recommended
- **Docker Images**: ~8GB for full rover environment
- **Workspace**: 4GB for source code and build artifacts
- **Logs and Data**: 2GB for competition data storage

### Connected Hardware
- **Cameras**: Intel RealSense D435i, CSI cameras
- **Motors**: CAN bus motor controllers
- **Sensors**: IMU, GPS, environmental sensors
- **Communication**: WiFi, radio modules
- **Power Management**: Smart power distribution

This Jetson Docker environment provides a robust, production-ready platform for autonomous rover operations while maintaining the flexibility needed for development and competition scenarios.