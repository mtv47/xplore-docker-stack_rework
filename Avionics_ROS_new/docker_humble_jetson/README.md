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
├── Makefile               # Build and deployment commands (replaces shell scripts)
└── README.md              # This documentation
```

## Makefile Commands

All functionality is now consolidated into a comprehensive Makefile optimized for Jetson deployment. Use `make help` to see all available commands.

### Core Commands

**Building and Running**:
- `make build` - Build the Avionics ROS Jetson Docker image
- `make run` - Run interactive container for development
- `make start-stack` - Start avionics stack automatically (production mode)
- `make attach` - Attach to running container with full ROS environment
- `make stop` - Stop the container

**Production Management**:
- `make launch-stack` - Launch avionics stack in running container
- `make stop-stack` - Stop the avionics stack (alias for stop)
- `make logs` - Show container logs

**Development**:
- `make dev-shell` - Run development shell without auto-launch
- `make hardware-check` - Check Jetson hardware and GPU availability
- `make status` - Show containers, volumes, and images status

**Maintenance**:
- `make clean` - Remove containers, volumes, and images
- `make docker-check` - Verify Docker is running
- `make x11-setup` - Setup X11 forwarding for GUI applications

### Key Features

**Dockerfile**:
- Optimized for NVIDIA Jetson ARM64 platforms with CUDA support

**Key Features**:
- Based on dustynv's optimized Jetson ROS2 image
- Includes NVIDIA L4T (Linux for Tegra) drivers
- ARM64-native compilation and execution
- Hardware acceleration libraries (CUDA, GStreamer)
**Container Features**:
- Production-ready Python environment
- Optimized for memory and CPU constraints
- Automated service lifecycle management
- Comprehensive hardware integration

**Jetson-Specific Optimizations**:
- Native ARM64 Python packages
- CUDA-accelerated libraries
- Memory-efficient package selection
- Hardware-specific drivers

## Production Deployment Workflow

### 1. Initial Setup on Jetson
```bash
# Build the production image
make build

# Start the avionics stack (production mode)
make start-stack
```

### 2. Service Management
```bash
# Check service status
make status

# Stop services
make stop

# Restart services
make stop
make start-stack

# View logs
make logs
```

### 3. Development and Debugging
```bash
# Launch interactive container for debugging
make run

# Or use development shell without auto-launch
make dev-shell

# Attach to running container
make attach

# Launch stack in running container
make launch-stack
```

### 4. Hardware and System Checks
```bash
# Check Jetson hardware status
make hardware-check

# Verify Docker environment
make docker-check
```
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
# Example systemd service (avionics.service)
[Unit]
Description=Xplore Avionics Stack
After=docker.service
Requires=docker.service

[Service]
Type=forking
WorkingDirectory=/path/to/Avionics_ROS/docker_humble_jetson
ExecStart=/usr/bin/make start-stack
ExecStop=/usr/bin/make stop
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Setup Instructions**:
```bash
# Install the service
sudo cp avionics.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable avionics.service

# Control the service
sudo systemctl start avionics
sudo systemctl status avionics
sudo systemctl stop avionics
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
make build

# Clean up old containers and images
make clean

# Update Python packages (in running container)
make attach
pip install --upgrade numpy pymodbus pyserial
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