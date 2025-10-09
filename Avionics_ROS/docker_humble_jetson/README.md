# Docker Humble Jetson - Avionics Production Environment

## Overview

The `docker_humble_jetson` folder contains the Docker configuration for the **production deployment environment** of the Avionics ROS2 system on NVIDIA Jetson platforms. This environment is specifically optimized for ARM64 architecture and is designed to run on the rover's embedded Jetson computer with hardware acceleration and real-time performance requirements.

## Purpose

This Docker environment serves as the **production runtime** for the avionics subsystem:
- **Production Deployment**: Run avionics code on the rover's Jetson platform
- **ARM64 Optimization**: Native ARM64 execution with CUDA/GPU acceleration
- **Real-time Operations**: Low-latency sensor data processing and hardware control
- **Hardware Integration**: Direct access to Jetson's GPIO, serial ports, and peripherals
- **Service Management**: Automated startup and lifecycle management of avionics services

## Architecture

### Base Image
- **Base**: `dustynv/ros:humble-desktop-l4t-r36.2.0`
- **Platform**: NVIDIA Jetson (ARM64/aarch64)
- **L4T Version**: Linux for Tegra R36.2.0 (JetPack 6.0)
- **ROS2 Distribution**: Humble Hawksbill
- **CUDA Support**: Native NVIDIA GPU acceleration
- **OS**: Ubuntu 22.04 LTS (Jammy) for ARM64

### Key Components
- **ROS2 Humble**: Full desktop installation optimized for Jetson
- **CycloneDDS**: High-performance DDS middleware for real-time communication
- **NVIDIA Libraries**: CUDA, cuDNN, TensorRT for GPU acceleration
- **Hardware Libraries**: libserialport, GStreamer for multimedia processing
- **Python Libraries**: `numpy`, `pymodbus`, `pyserial`, `minimalmodbus` for sensor communication

## File Structure

```
docker_humble_jetson/
├── Dockerfile              # Docker image definition for Jetson
├── build.sh               # Image build script
├── run.sh                 # Interactive container execution
├── attach.sh              # Environment setup script
├── start_elec_stack.sh    # Production service startup
├── stop_elec_stack.sh     # Production service shutdown
└── README.md              # This documentation
```

## Scripts Description

### 1. `Dockerfile`
**Purpose**: Defines the production Docker image for Jetson platforms

**Key Features**:
- Based on dustynv's optimized Jetson ROS2 image
- Includes NVIDIA L4T (Linux for Tegra) drivers
- ARM64-native compilation and execution
- Hardware acceleration libraries (CUDA, GStreamer)
- Production-ready Python environment
- Optimized for memory and CPU constraints

**Jetson-Specific Optimizations**:
- Native ARM64 Python packages
- CUDA-accelerated libraries
- Memory-efficient package selection
- Hardware-specific drivers

### 2. `build.sh`
**Purpose**: Builds the Jetson-optimized Docker image

```bash
./build.sh
```

**What it does**:
- Builds image with tag `ghcr.io/epflxplore/elec:humble-jetson`
- Uses ARM64-specific base image
- Optimizes for Jetson hardware capabilities

### 3. `run.sh`
**Purpose**: Launches an interactive development container

```bash
./run.sh
```

**Features**:
- **Interactive Mode**: For development and debugging
- **Hardware Access**: Full device mounting (`/dev:/dev`)
- **GPU Access**: NVIDIA GPU passthrough
- **Network**: Host networking for ROS2 communication
- **GUI Support**: X11 forwarding for visualization tools
- **Volume Persistence**: Home directory and source code mounting

**Use Case**: Development, testing, and manual debugging on Jetson

### 4. `attach.sh`
**Purpose**: Sets up the ROS2 environment inside containers

```bash
source attach.sh
```

**Configuration**:
- **DDS Middleware**: CycloneDDS for optimal Jetson performance
- **Display**: X11 display configuration for GUI applications
- **Python Environment**: Custom package paths for avionics modules
- **ROS2 Workspace**: Sources the built workspace

### 5. `start_elec_stack.sh`
**Purpose**: Production service startup with container lifecycle management

```bash
./start_elec_stack.sh
```

**Advanced Features**:
- **Container State Management**: Checks if container is already running
- **Automatic Recovery**: Restarts container if needed
- **Production Launch**: Directly launches avionics system
- **X11 Configuration**: Automated GUI setup
- **Service Mode**: Runs in non-interactive mode for production

**Production Workflow**:
1. Checks existing container status
2. Sets up X11 authentication if needed
3. Starts new container OR attaches to existing one
4. Automatically launches `ros2 launch avionics_nexus launch.py`

### 6. `stop_elec_stack.sh`
**Purpose**: Graceful shutdown of production services

```bash
./stop_elec_stack.sh
```

**Features**:
- **Safe Shutdown**: Checks container status before stopping
- **Graceful Stop**: Uses Docker's graceful stop mechanism
- **Resource Cleanup**: Ensures proper cleanup of resources

## Production Deployment Workflow

### 1. Initial Setup on Jetson
```bash
# Build the production image
./build.sh

# Start the avionics stack
./start_elec_stack.sh
```

### 2. Service Management
```bash
# Check if services are running
docker ps | grep elec_humble_jetson

# Stop services
./stop_elec_stack.sh

# Restart services
./stop_elec_stack.sh
./start_elec_stack.sh
```

### 3. Development and Debugging
```bash
# Launch interactive container for debugging
./run.sh

# Inside container - manual launch
source src/docker_humble_jetson/attach.sh
cd ..
colcon build
ros2 launch avionics_nexus launch.py
```

### 4. Monitoring and Logs
```bash
# View container logs
docker logs elec_humble_jetson

# Attach to running container
docker exec -it elec_humble_jetson bash

# Monitor ROS2 topics
docker exec -it elec_humble_jetson bash -c "source install/setup.bash; ros2 topic list"
```

## Hardware Integration

### Jetson-Specific Features
- **GPIO Access**: Direct access to Jetson GPIO pins
- **Serial Interfaces**: Multiple UART/USB serial ports for sensors
- **I2C/SPI**: Hardware bus access for sensor communication
- **Camera Interfaces**: CSI and USB camera support
- **GPU Acceleration**: CUDA-accelerated processing for sensor data

### Device Mapping
- **BMS Interface**: `/dev/ttyBMS` - Battery Management System
- **4-in-1 Sensor**: `/dev/tty4in1` - Environmental sensors
- **LED Controller**: `/dev/ttyESP32_LED` - LED control ESP32
- **Avionics MCU**: `/dev/ttyESP32_Avionics` - Main avionics microcontroller

### Performance Optimizations
- **Native ARM64**: No emulation overhead
- **CUDA Acceleration**: GPU-accelerated sensor processing
- **Memory Management**: Optimized for Jetson's memory constraints
- **Real-time Scheduling**: Low-latency operation for critical sensors

## System Integration

### Service Integration
The Jetson environment integrates with systemd services for automatic startup:

```bash
# Example systemd service (not included but recommended)
[Unit]
Description=Xplore Avionics Stack
After=docker.service
Requires=docker.service

[Service]
Type=forking
ExecStart=/path/to/start_elec_stack.sh
ExecStop=/path/to/stop_elec_stack.sh
Restart=always

[Install]
WantedBy=multi-user.target
```

### Network Configuration
- **Host Networking**: Direct access to Jetson's network interfaces
- **ROS2 Discovery**: Automatic discovery of other ROS2 nodes on network
- **DDS Configuration**: Optimized for local and remote communication

### Resource Management
- **Memory Limits**: Configured for Jetson's available RAM
- **CPU Affinity**: Can be configured for specific CPU cores
- **GPU Memory**: Managed GPU memory allocation for CUDA operations

## Troubleshooting

### Common Jetson Issues

#### Container Won't Start
```bash
# Check Docker service
sudo systemctl status docker

# Check image availability
docker images | grep humble-jetson

# Check available resources
free -h
df -h
```

#### Hardware Access Issues
```bash
# Check device permissions
ls -la /dev/tty*
sudo usermod -a -G dialout $USER

# Check USB devices
lsusb
dmesg | grep tty
```

#### Performance Issues
```bash
# Monitor system resources
htop
nvidia-smi  # Check GPU usage
iotop       # Check I/O usage

# Check Jetson clocks
sudo jetson_clocks --show
sudo jetson_clocks  # Enable max performance
```

#### ROS2 Communication Issues
```bash
# Check DDS configuration
echo $RMW_IMPLEMENTATION

# Test ROS2 discovery
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

### Performance Tuning

#### Jetson Performance Modes
```bash
# Check current power mode
sudo nvpmodel -q

# Set maximum performance mode
sudo nvpmodel -m 0
sudo jetson_clocks
```

#### Container Optimization
```bash
# Add CPU/memory limits to docker run
--cpus="3.0" --memory="6g"

# Set CPU affinity
--cpuset-cpus="0-3"
```

## Security Considerations

### Production Security
- **Privileged Access**: Required for hardware access but monitored
- **Network Security**: Host networking requires firewall configuration
- **Source Code Protection**: Code removed from final image
- **Container Isolation**: Proper user permissions and resource limits

### Hardening Recommendations
- Use specific user accounts instead of privileged mode when possible
- Implement container resource limits
- Regular security updates of base image
- Network segmentation for production deployments

## Maintenance and Updates

### Regular Maintenance
```bash
# Update base image
docker pull dustynv/ros:humble-desktop-l4t-r36.2.0
./build.sh

# Clean up old containers and images
docker system prune -a

# Update Python packages
docker exec -it elec_humble_jetson pip install --upgrade numpy pymodbus pyserial
```

### Backup and Recovery
```bash
# Backup persistent volume
docker run --rm -v elec_humble_jetson_home_volume:/data -v $(pwd):/backup ubuntu tar czf /backup/jetson_home_backup.tar.gz -C /data .

# Restore from backup
docker run --rm -v elec_humble_jetson_home_volume:/data -v $(pwd):/backup ubuntu tar xzf /backup/jetson_home_backup.tar.gz -C /data
```

### Monitoring and Logging
- **Container Health**: Regular health checks and monitoring
- **Resource Usage**: Monitor CPU, memory, and GPU utilization
- **Log Management**: Implement log rotation and centralized logging
- **Performance Metrics**: Track sensor data latency and throughput

## Integration with Rover Systems

### Communication with Other Subsystems
- **Control Station**: ROS2 topics for remote control and monitoring
- **Navigation**: Sensor data sharing for localization and mapping
- **Science**: Environmental sensor data for scientific measurements
- **HD**: Camera control and data streaming coordination

### Data Flow
1. **Sensor Data Collection**: Real-time sensor data from BMS, environmental sensors
2. **Processing**: Local processing with GPU acceleration when needed
3. **Distribution**: ROS2 topic publishing to other rover subsystems
4. **Command Execution**: Receive and execute commands from control station

This production environment ensures reliable, high-performance operation of the avionics subsystem on the rover's Jetson platform while providing the tools necessary for development, debugging, and maintenance.