# Docker Humble Jetson Environment# Docker Humble Jetson Environment# Docker Humble Jetson Environment



This directory contains the Docker containerization setup for running the EPFLXplore Rover system on **NVIDIA Jetson** embedded platforms using **ROS 2 Humble**. This production-ready environment is optimized for deployment on the actual rover hardware with ARM64 architecture support, hardware acceleration, and real-time performance capabilities.



## OverviewThis directory contains the Docker containerization setup for running the EPFLXplore Rover system on **NVIDIA Jetson** embedded platforms using **ROS 2 Humble**. This production-ready environment is optimized for deployment on the actual rover hardware with ARM64 architecture support, hardware acceleration, and real-time performance capabilities.This directory contains the Docker containerization setup for running the EPFLXplore Rover system on **NVIDIA Jetson** embedded platforms using **ROS 2 Humble**. This production-ready environment is optimized for deployment on the actual rover hardware with ARM64 architecture support, hardware acceleration, and real-time performance capabilities.



The Jetson Docker environment provides:

- **Production Deployment**: Optimized for NVIDIA Jetson Nano, Xavier NX, AGX Xavier, and Orin platforms

- **ARM64 Architecture**: Native ARM64 container support for Jetson hardware## Overview## Overview

- **Hardware Acceleration**: GPU access for computer vision and AI inference

- **Real-Time Performance**: Low-latency configuration for competition scenarios

- **Hardware Integration**: Direct access to cameras, sensors, motors, and communication devices

- **Competition Ready**: Autonomous launch capabilities with systemd service integrationThe Jetson Docker environment provides:The Jetson Docker environment provides:



## Files Description- **Production Deployment**: Optimized for NVIDIA Jetson Nano, Xavier NX, AGX Xavier, and Orin platforms- **Production Deployment**: Optimized for NVIDIA Jetson Nano, Xavier NX, AGX Xavier, and Orin platforms



### Core Files- **ARM64 Architecture**: Native ARM64 container support for Jetson hardware- **ARM64 Architecture**: Native ARM64 container support for Jetson hardware



#### `Dockerfile`- **Hardware Acceleration**: GPU access for computer vision and AI inference- **Hardware Acceleration**: GPU access for computer vision and AI inference

Production-optimized Docker build configuration:

- **Base Image**: `ghcr.io/epflxplore/docker_commons:humble-jetson` (ARM64 optimized)- **Real-Time Performance**: Low-latency configuration for competition scenarios- **Real-Time Performance**: Low-latency configuration for competition scenarios

- **ROS 2 Packages**: Navigation2 stack, OpenCV with CUDA support

- **Hardware Support**: - **Hardware Integration**: Direct access to cameras, sensors, motors, and communication devices- **Hardware Integration**: Direct access to cameras, sensors, motors, and communication devices

  - Intel RealSense cameras (`pyrealsense2`)

  - Intel Movidius Neural Compute Stick (USB rules)- **Competition Ready**: Autonomous launch capabilities with systemd service integration- **Competition Ready**: Autonomous launch capabilities with systemd service integration

  - NVIDIA GPU acceleration

- **Python Dependencies**: 

  - Networking: `aiohttp`, `aiortc`, `aiohttp_cors`

  - System monitoring: `ping3`, `tornado` ## Files Description## Files Description

  - Database: `pymongo`, `bson`

  - Computer vision: `numpy`, `opencv`

- **Security**: Confidential source code removed from final image

### Core Files### Core Files

#### `Makefile`

Comprehensive production deployment and development system with all functionality built-in:

```bash

make help       # Show all available commands#### `Dockerfile`#### `Dockerfile`

make build      # Build the Jetson Docker image

make run        # Run interactive containerProduction-optimized Docker build configuration:Production-optimized Docker build configuration:

make run-rover  # Run rover launch script (production)

make attach     # Attach to running container- **Base Image**: `ghcr.io/epflxplore/docker_commons:humble-jetson` (ARM64 optimized)- **Base Image**: `ghcr.io/epflxplore/docker_commons:humble-jetson` (ARM64 optimized)

```

- **ROS 2 Packages**: Navigation2 stack, OpenCV with CUDA support- **ROS 2 Packages**: Navigation2 stack, OpenCV with CUDA support

All environment setup and configuration is handled automatically within the Makefile targets - no external scripts required.

- **Hardware Support**: - **Hardware Support**: 

## Usage Workflows

  - Intel RealSense cameras (`pyrealsense2`)  - Intel RealSense cameras (`pyrealsense2`)

### Quick Start

  - Intel Movidius Neural Compute Stick (USB rules)  - Intel Movidius Neural Compute Stick (USB rules)

1. **View Available Commands**:

   ```bash  - NVIDIA GPU acceleration  - NVIDIA GPU acceleration

   make help

   ```- **Python Dependencies**: - **Python Dependencies**: 



2. **Build the Jetson Image**:  - Networking: `aiohttp`, `aiortc`, `aiohttp_cors`  - Networking: `aiohttp`, `aiortc`, `aiohttp_cors`

   ```bash

   make build  - System monitoring: `ping3`, `tornado`   - System monitoring: `ping3`, `tornado` 

   ```

  - Database: `pymongo`, `bson`  - Database: `pymongo`, `bson`

3. **Start Development Container**:

   ```bash  - Computer vision: `numpy`, `opencv`  - Computer vision: `numpy`, `opencv`

   # Interactive development

   make run- **Security**: Confidential source code removed from final image- **Security**: Confidential source code removed from final image

   

   # Production rover launch

   make run-rover

   ```#### `Makefile`#### `build.sh`



4. **Attach to Running Container**:Comprehensive production deployment and development system:Jetson-specific image build script:

   ```bash

   make attach```bash```bash

   ```

make help       # Show all available commands./build.sh

### Available Make Commands

make build      # Build the Jetson Docker image```

- **`make build`**: Build the Rover Jetson Docker image

- **`make run`**: Run interactive container for development and debuggingmake run        # Run interactive container- Builds ARM64 architecture image

- **`make run-rover`**: Run rover launch script (starts rover autonomously or attaches if running)

- **`make attach`**: Attach to running container with proper environment setupmake run-rover  # Run rover launch script (production)- Tags as `ghcr.io/epflxplore/rover:humble-jetson`

- **`make stop`**: Stop the main rover container

- **`make stop-rover`**: Alias for stop (stop rover container)make attach     # Attach to running container- Optimized build process for Jetson platforms

- **`make stop-cameras`**: Stop the camera container specifically

- **`make clean`**: Remove containers, volumes, and images (with confirmation)```

- **`make status`**: Show status of containers, volumes, and images

- **`make docker-check`**: Verify Docker is running### Execution Scripts

- **`make x11-setup`**: Setup X11 forwarding manually

#### `attach_env.sh`

### Production Deployment Workflow

Environment setup script for container execution with proper ROS 2 configuration#### `run.sh` (Interactive Development)

1. **Build for Production**:

   ```bashInteractive development and debugging script:

   make build

   ```## Usage Workflows```bash



2. **Run Rover in Production Mode**:./run.sh

   ```bash

   make run-rover### Quick Start```

   ```

   This will:

   - Start a new container if none is running

   - Attach to existing container if already running1. **View Available Commands**:**Features**:

   - Automatically launch the rover navigation system

   - Configure all environment variables and paths inline   ```bash- **Interactive Shell**: Full bash access for development and debugging



3. **Monitor and Debug**:   make help- **X11 Forwarding**: GUI application support (RViz, debugging tools)

   ```bash

   # Check status   ```- **Hardware Access**: Complete `/dev` directory mounting

   make status

   - **Jetson Monitoring**: `jtop` socket integration for system monitoring

   # Attach additional terminal for debugging

   make attach2. **Build the Jetson Image**:- **Competition Photos**: Special volume mount for `/home/xplore/photos_competition`

   ```

   ```bash- **Development Volumes**:

### Development Workflow

   make build  - Source code: Live mounting for development

1. **Interactive Development**:

   ```bash   ```  - Home persistence: `rover_humble_jetson_home_volume`

   make run

   ```



2. **Inside Container Development**:3. **Start Development Container**:#### `run_rover.sh` (Production Launch)

   ```bash

   # Build your packages   ```bash**Primary production script** for autonomous rover operation:

   colcon build

      # Interactive development```bash

   # Source the workspace

   source install/setup.bash   make run./run_rover.sh

   

   # Test individual components   ```

   ros2 launch rover_pkg launch.py

   ```   # Production rover launch



### Container Management   make run-rover**Key Features**:



- **Start Production System**: `make run-rover`   ```- **Smart Container Management**: Checks if container is running, starts new or attaches to existing

- **Stop Everything**: `make stop` or `make stop-rover`

- **Stop Only Cameras**: `make stop-cameras`- **Automatic Launch**: Directly executes `ros2 launch rover_pkg launch.py`

- **Clean Reset**: `make clean` (removes all data)

4. **Attach to Running Container**:- **Non-Interactive Mode**: Optimized for systemd service integration

## Environment Configuration

   ```bash- **Fault Tolerance**: Handles container state transitions gracefully

### ROS 2 Configuration

- **Distribution**: ROS 2 Humble Hawksbill   make attach- **Production Environment**: Minimal overhead for competition performance

- **RMW Implementation**: CycloneDDS (rmw_cyclonedds_cpp)

- **Domain ID**: Default (0) - configurable via `ROS_DOMAIN_ID`   ```

- **Architecture**: ARM64 native for Jetson hardware

**Container Logic**:

### Automatic Environment Setup

The Makefile automatically configures all necessary environment variables:### Available Make Commands```bash

- **RMW_IMPLEMENTATION**: `rmw_cyclonedds_cpp`

- **DISPLAY**: X11 display forwarding# If container not running -> Start new container with rover launch

- **QT_X11_NO_MITSHM**: X11 compatibility

- **PYTHONPATH**: Complete ROS 2 Python package paths- **`make build`**: Build the Rover Jetson Docker image# If container already running -> Attach and execute rover launch

- **ROS 2 Sourcing**: Workspace setup and sourcing

- **`make run`**: Run interactive container for development and debugging```

### Volume Mounts Explained

- **`make run-rover`**: Run rover launch script (starts rover autonomously or attaches if running)

| Host Path | Container Path | Purpose |

|-----------|----------------|---------|- **`make attach`**: Attach to running container with proper environment setup### Management Scripts

| `../` (parent directory) | `/home/xplore/dev_ws/src` | Live source code editing |

| `/home/xplore/photos_competition` | `/home/xplore/dev_ws/photos_competition` | Competition photo storage |- **`make stop`**: Stop the main rover container

| `rover_humble_jetson_home_volume` | `/home/xplore` | Persistent home directory |

| `/dev` | `/dev` | Hardware device access |- **`make stop-rover`**: Alias for stop (stop rover container)#### `attach.sh`

| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 socket for GUI apps |

| `/run/jtop.sock` | `/run/jtop.sock` | Jetson stats monitoring |- **`make stop-cameras`**: Stop the camera container specificallyEnvironment configuration for manual container access:



### Hardware Access- **`make clean`**: Remove containers, volumes, and images (with confirmation)```bash

- **Privileged Mode**: Full hardware access for motor control and sensors

- **Host Networking**: Direct network interface access for real-time communication- **`make status`**: Show status of containers, volumes, and imagessource attach.sh

- **Device Mounting**: Complete `/dev` access for cameras, sensors, and peripherals

- **GPU Access**: NVIDIA GPU acceleration for computer vision workloads- **`make docker-check`**: Verify Docker is running```



## Troubleshooting- **`make x11-setup`**: Setup X11 forwarding manually



### Container Issues**Configuration**:

```bash

# Check if rover is running### Production Deployment Workflow- **RMW Setup**: CycloneDDS middleware for reliable communication

make status

- **Python Environment**: ROS 2 package path configuration

# Stop and restart if stuck

make stop1. **Build for Production**:- **X11 Variables**: Display forwarding setup

make run-rover

```   ```bash- **Workspace Sourcing**: Automatic ROS 2 environment loading



### Hardware Access Problems   make build

```bash

# Verify device permissions (run on host)   ```#### `stop_docker_rover.sh`

ls -la /dev/video*

ls -la /dev/ttyUSB*Safe rover container shutdown:



# The container runs in privileged mode to access hardware2. **Run Rover in Production Mode**:```bash

# If issues persist, check host device permissions

```   ```bash./stop_docker_rover.sh



### ROS 2 Communication Issues   make run-rover```

```bash

# Inside container, verify configuration   ```- **Container Check**: Verifies container is running before stopping

echo $RMW_IMPLEMENTATION

# Should output: rmw_cyclonedds_cpp   This will:- **Graceful Shutdown**: Uses `docker stop` for clean termination



# Test ROS 2 nodes   - Start a new container if none is running- **State Management**: Ensures proper cleanup

ros2 node list

ros2 topic list   - Attach to existing container if already running

```

   - Automatically launch the rover navigation system#### `stop_docker_cameras.sh`

### Performance Issues

```bash   - Configure all environment variables and pathsCamera-specific container management:

# Check Jetson stats (inside container)

jtop```bash



# Monitor container resources3. **Monitor and Debug**:./stop_docker_cameras.sh

docker stats rover_humble_jetson

```   ```bash```



### X11 Display Issues   # Check status- **Target**: Stops `rover_humble_jetson_2` camera container

```bash

# Setup X11 manually if needed   make status- **Specialized Function**: Handles separate camera system containers

make x11-setup

   - **Safety Check**: Validates container state before operation

# Or restart container

make stop   # Attach additional terminal for debugging

make run

```   make attach## Jetson-Specific Optimizations



## Development Tips   ```



### Competition Mode### Hardware Integration

- Use `make run-rover` for autonomous operation

- Container will auto-restart the rover launch script if it exits### Development Workflow

- Monitor with `make status` and `make attach`

#### GPU Acceleration

### Development Mode

- Use `make run` for interactive development1. **Interactive Development**:- **CUDA Support**: OpenCV compiled with CUDA for computer vision acceleration

- Full access to build system and debugging tools

- Hot-reload Python code, rebuild C++ as needed   ```bash- **Neural Processing**: Intel Movidius NCS support for AI inference



### Performance Optimization   make run- **Memory Management**: Optimized for Jetson's unified memory architecture

- The image is optimized for ARM64 architecture

- GPU acceleration is available for computer vision tasks   ```

- Real-time performance configuration for competition scenarios

#### System Monitoring

### Container States

- **Development**: Interactive shell with full development tools2. **Inside Container Development**:- **JTop Integration**: Real-time Jetson hardware monitoring

- **Production**: Autonomous rover operation with automatic restart

- **Monitoring**: Non-intrusive attachment for debugging   ```bash  - GPU utilization and memory usage



## Migration from Shell Scripts   # Build your packages  - CPU frequency and temperature



This Makefile replaces the following shell scripts with **zero external dependencies**:   colcon build  - Power consumption and thermal throttling

- `build.sh` → `make build`

- `run.sh` → `make run`     - Fan speed and system health

- `run_rover.sh` → `make run-rover`

- `attach.sh` → `make attach` (environment setup now inline)   # Source the workspace- **Volume Mount**: `/run/jtop.sock` for system stats access

- `stop_docker_rover.sh` → `make stop-rover`

- `stop_docker_cameras.sh` → `make stop-cameras`   source install/setup.bash



All functionality has been preserved and enhanced with:   #### Device Access

- **Zero External Scripts**: Everything is self-contained in the Makefile

- **Better error handling** and status reporting   # Test individual components- **Complete Hardware Access**: Full `/dev` directory mounting

- **Unified interface** across development and production

- **Automatic environment setup** and validation   ros2 launch rover_pkg launch.py- **USB Device Rules**: Pre-configured for competition hardware

- **Container lifecycle management**

- **Inline environment configuration** (no external scripts needed)   ```- **Camera Integration**: Direct access to CSI and USB cameras

- **Serial Communication**: Motor controller and sensor interfaces

### Container Management

### Network Configuration

- **Start Production System**: `make run-rover`

- **Stop Everything**: `make stop` or `make stop-rover`#### Host Networking

- **Stop Only Cameras**: `make stop-cameras`- **Performance**: Eliminates Docker network overhead

- **Clean Reset**: `make clean` (removes all data)- **ROS 2 Discovery**: Direct multicast communication

- **Low Latency**: Critical for real-time rover control

## Environment Configuration- **Competition Network**: Seamless integration with field communication



### ROS 2 Configuration#### Middleware Optimization

- **Distribution**: ROS 2 Humble Hawksbill- **CycloneDDS**: High-performance DDS implementation

- **RMW Implementation**: CycloneDDS (rmw_cyclonedds_cpp)- **Real-Time Priorities**: Optimized for time-critical operations

- **Domain ID**: Default (0) - configurable via `ROS_DOMAIN_ID`- **Memory Efficiency**: Reduced overhead for embedded deployment

- **Architecture**: ARM64 native for Jetson hardware

## Production Deployment

### Volume Mounts Explained

### Systemd Integration

| Host Path | Container Path | Purpose |

|-----------|----------------|---------|The rover system integrates with systemd for autonomous startup:

| `../` (parent directory) | `/home/xplore/dev_ws/src` | Live source code editing |

| `/home/xplore/photos_competition` | `/home/xplore/dev_ws/photos_competition` | Competition photo storage |```bash

| `rover_humble_jetson_home_volume` | `/home/xplore` | Persistent home directory |# Service file: /etc/systemd/system/rovernode.service

| `/dev` | `/dev` | Hardware device access |[Unit]

| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 socket for GUI apps |Description=Rover Node Service

| `/run/jtop.sock` | `/run/jtop.sock` | Jetson stats monitoring |After=docker.service

Requires=docker.service

### Hardware Access

- **Privileged Mode**: Full hardware access for motor control and sensors[Service]

- **Host Networking**: Direct network interface access for real-time communicationExecStart=/bin/bash /home/xplore/ERC_CS_Rover/docker_humble_jetson/run_rover.sh

- **Device Mounting**: Complete `/dev` access for cameras, sensors, and peripheralsExecStop=/usr/bin/docker stop rover_humble_jetson

- **GPU Access**: NVIDIA GPU acceleration for computer vision workloads

[Install]

## TroubleshootingWantedBy=default.target

```

### Container Issues

```bash### Automatic Startup Workflow

# Check if rover is running

make status1. **System Boot**: Jetson powers on

2. **Docker Service**: SystemD starts Docker daemon

# Stop and restart if stuck3. **Rover Service**: SystemD executes `run_rover.sh`

make stop4. **Container Check**: Script determines container state

make run-rover5. **Rover Launch**: Automatic execution of rover node

```6. **Competition Ready**: Full system operational



### Hardware Access Problems### Competition Configuration

```bash

# Verify device permissions (run on host)#### Volume Mounts for Competition

ls -la /dev/video*```bash

ls -la /dev/ttyUSB*# Competition photos storage

/home/xplore/photos_competition -> /home/xplore/dev_ws/photos_competition

# The container runs in privileged mode to access hardware

# If issues persist, check host device permissions# Source code (live development)

```$PARENT_DIR -> /home/xplore/dev_ws/src



### ROS 2 Communication Issues# Persistent home directory

```bashrover_humble_jetson_home_volume -> /home/xplore

# Inside container, verify configuration```

echo $RMW_IMPLEMENTATION

# Should output: rmw_cyclonedds_cpp#### Environment Variables

```bash

# Test ROS 2 nodesDISPLAY=unix$DISPLAY          # X11 forwarding

ros2 node listQT_X11_NO_MITSHM=1           # Qt compatibility

ros2 topic listXAUTHORITY=$XAUTH            # X11 authentication

``````



### Performance Issues## Usage Workflows

```bash

# Check Jetson stats (inside container)### Development Workflow

jtop

1. **Build Jetson Image**:

# Monitor container resources   ```bash

docker stats rover_humble_jetson   cd docker_humble_jetson

```   ./build.sh

   ```

### X11 Display Issues

```bash2. **Interactive Development**:

# Setup X11 manually if needed   ```bash

make x11-setup   ./run.sh

   # Full interactive shell with GUI support

# Or restart container   ```

make stop

make run3. **Inside Container**:

```   ```bash

   colcon build

## Development Tips   source install/setup.bash

   ros2 launch rover_pkg launch.py

### Competition Mode   ```

- Use `make run-rover` for autonomous operation

- Container will auto-restart the rover launch script if it exits### Production Deployment

- Monitor with `make status` and `make attach`

1. **Deploy to Jetson**:

### Development Mode   ```bash

- Use `make run` for interactive development   # Copy entire repository to Jetson

- Full access to build system and debugging tools   scp -r ERC_CS_Rover xplore@jetson-ip:/home/xplore/

- Hot-reload Python code, rebuild C++ as needed   ```



### Performance Optimization2. **Build Production Image**:

- The image is optimized for ARM64 architecture   ```bash

- GPU acceleration is available for computer vision tasks   ssh xplore@jetson-ip

- Real-time performance configuration for competition scenarios   cd /home/xplore/ERC_CS_Rover/docker_humble_jetson

   ./build.sh

### Container States   ```

- **Development**: Interactive shell with full development tools

- **Production**: Autonomous rover operation with automatic restart3. **Test Production Launch**:

- **Monitoring**: Non-intrusive attachment for debugging   ```bash

   ./run_rover.sh

## Migration from Shell Scripts   # Should automatically launch rover system

   ```

This Makefile replaces the following shell scripts:

- `build.sh` → `make build`4. **Enable Systemd Service**:

- `run.sh` → `make run`   ```bash

- `run_rover.sh` → `make run-rover`   sudo cp ../rovernode.service /etc/systemd/system/

- `attach.sh` → `make attach`   sudo systemctl enable rovernode.service

- `stop_docker_rover.sh` → `make stop-rover`   sudo systemctl start rovernode.service

- `stop_docker_cameras.sh` → `make stop-cameras`   ```



All functionality has been preserved and enhanced with:### Debugging and Monitoring

- Better error handling and status reporting

- Unified interface across development and production#### System Monitoring

- Automatic environment setup and validation```bash

- Container lifecycle management# Monitor Jetson performance
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