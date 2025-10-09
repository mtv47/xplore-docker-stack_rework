# Docker Humble Jetson Environment

This directory contains the Docker configuration for running the ERC Navigation stack on NVIDIA Jetson platforms in a production-ready ROS2 Humble environment.

## Overview

The `docker_humble_jetson` environment is designed for:
- **Production Deployment**: Real robot operation on NVIDIA Jetson hardware
- **Hardware Integration**: Full sensor integration including LiDAR, cameras, and IMU
- **Performance Optimization**: ARM64 and CUDA optimizations for embedded systems
- **Competition Ready**: Robust configuration for European Rover Challenge deployment

## Quick Start

To see all available commands:
```bash
make help
```

Common workflows:
```bash
# Build the Navigation image
make build

# Check Jetson hardware and requirements
make hardware-check

# Run Navigation container with GPU acceleration
make run

# In another terminal, attach to the running container
make attach

# Launch wheels control system
make wheels-control

# Check status
make status

# Stop the container
make stop
```

## Configuration

All container management is handled through the comprehensive **Makefile** which provides:

### Core Commands
- `make build` - Build the Navigation Jetson image
- `make build-quick` - Build using cache (faster)
- `make run` - Launch Navigation container with full hardware access
- `make attach` - Connect to running container with ROS2 environment
- `make stop` - Stop all Navigation containers
- `make status` - Show container, volume, and image status

### Specialized Features
- `make wheels-control` - Launch wheels control stack (auto-manages containers)
- `make nav-test` - Run Navigation stack tests
- `make launch-nav` - Launch full Navigation stack
- `make wheels-status` - Check wheels control system status

### Development & Testing
- `make dev-shell` - Start development shell without auto-setup
- `make performance-check` - Monitor GPU and system performance
- `make config-check` - Validate configuration files and paths

### Hardware & Diagnostics
- `make hardware-check` - Verify Jetson platform and Navigation requirements
- `make gpu-check` - Check GPU availability and Jetson tools
- `make jetson-info` - Detailed Jetson platform information
- `make x11-setup` - Configure X11 forwarding for GUI applications

### Maintenance
- `make clean` - Remove containers, volumes, and images
- `make logs` - Show container logs

## Files Description

— **Dockerfile** → Defines the production navigation container for Jetson platforms
  * Base Image: Extends `ghcr.io/epflxplore/docker_commons:humble-jetson`
  * RealSense Integration: Builds Intel RealSense SDK from source with Python bindings
  * CUDA Support: Enables CUDA acceleration for RealSense and point cloud processing
  * Navigation Stack: Installs complete Nav2 framework with Jetson optimizations
  * SLAM Libraries: Includes GTSAM, LIORF dependencies, and mapping tools
  * Python Environment: Custom pyrealsense2 installation with proper path configuration
  * Hardware Access: USB rules for OAK-D cameras and sensor connectivity
  * Cleanup: Removes source code from final image for security and size optimization

— **Makefile** → Comprehensive container management system
  * Build Management: Image building with caching options and network configuration
  * Hardware Integration: Full Jetson, GPU, and sensor support with JTop monitoring
  * X11 Configuration: Advanced display forwarding setup for remote operation
  * Navigation Tools: Wheels control, stack testing, and performance monitoring
  * Development Support: Multiple container access methods and debugging commands
  * Configuration Validation: Hardware checking, config file validation, platform verification
  * Maintenance: Cleanup, status reporting, and log access
— **cyclonedds.xml** → Advanced DDS configuration for multi-interface networking
  * Network Interfaces: Configures both Ethernet interfaces (enP8p1s0, enx0a76f8b03d57)
  * Multicast Support: Enables multicast for ROS2 node discovery
  * Domain Configuration: Sets up DDS domain for isolated communication
  * Interface Binding: Handles both wired and USB-Ethernet connections

## Prerequisites

— **Hardware Requirements**
  * Platform: NVIDIA Jetson Nano, TX2, Xavier NX, or AGX Xavier/Orin
  * Memory: Minimum 8GB RAM, 16GB recommended for full navigation stack
  * Storage: 64GB+ SD card or eMMC, SSD recommended for performance
  * GPU: Integrated CUDA-capable GPU with sufficient memory
  * Sensors: Ouster LiDAR, Intel RealSense cameras, IMU, motor controllers
  * Network: Ethernet ports for LiDAR and external communication

— **Software Requirements**
  * JetPack: Latest NVIDIA JetPack SDK with Docker support
  * Docker Engine: Version 20.10+ with NVIDIA Container Runtime
  * NVIDIA Docker: GPU acceleration support for containers
  * System Permissions: User access to docker group and hardware devices

## Installation & Setup

— **Step 1: Build the Jetson Image**
```bash
cd docker_humble_jetson
./build.sh
```
  * Base Configuration: Pulls Jetson-optimized ROS2 Humble base image
  * RealSense Build: Compiles Intel RealSense SDK with CUDA and Python support
  * Navigation Stack: Installs Nav2 with ARM64 and CUDA optimizations
  * Dependencies: Builds GTSAM, LIORF, and other SLAM libraries from source
## Installation & Setup

— **Step 1: Build the Navigation Image**
```bash
make build
```
  * Jetson Optimization: Compiles RealSense SDK with ARM64 and CUDA support
  * Navigation Dependencies: Installs Nav2, SLAM tools, and sensor drivers
  * Python Integration: Configures pyrealsense2 with proper library paths

— **Step 2: Verify Hardware**
```bash
make hardware-check
```
  * Platform Detection: Confirms Jetson platform and capabilities
  * GPU Validation: Checks NVIDIA GPU and CUDA accessibility
  * Sensor Verification: Validates sensor connectivity and permissions
  * Configuration: Checks DDS config and volume mounts

— **Step 3: Launch Navigation Container**
```bash
# Full navigation stack
make run

# Wheel control only
make wheels-control
```
  * Hardware Initialization: Enables all sensors and actuators
  * Network Configuration: Sets up multi-interface DDS communication
  * Environment Setup: Configures CUDA, Python paths, and ROS2 workspace
  * Security: Establishes proper file permissions and user access

— **Step 4: Access Development Environment**
```bash
make attach
```
  * Environment Configuration: Sets up complete ROS2 development environment
  * Multi-session Support: Enables concurrent debugging and monitoring
  * Hardware Access: Provides access to all connected sensors and actuators

## Production Features

— **Sensor Integration**
  * Intel RealSense: Full D400/L500 series support with Python bindings
  * Ouster LiDAR: Optimized drivers with CUDA point cloud processing
  * OAK-D Cameras: Depth AI integration for stereo vision
  * IMU Integration: Madgwick filtering and sensor fusion
  * Serial Devices: Motor controllers and auxiliary sensors

— **Performance Optimizations**
  * ARM64 Compilation: Native ARM64 builds for optimal performance
  * CUDA Acceleration: GPU-accelerated point cloud and image processing
  * Memory Management: Optimized for limited Jetson memory resources
  * Real-time Processing: Low-latency sensor data processing
  * Power Efficiency: Thermal and power management considerations

— **Networking & Communication**
  * CycloneDDS: High-performance DDS middleware for ROS2
  * Multi-interface: Supports both Ethernet and USB-Ethernet adapters
  * LiDAR Networking: Dedicated network configuration for sensor communication
  * Remote Access: X11 forwarding for remote visualization and debugging

## Usage Examples

— **Full Navigation Stack Deployment**
```bash
# 1. Start navigation container
make run

# 2. Launch complete autonomous navigation (inside container)
# 2. Launch navigation stack (inside container)
make launch-nav

# Or manually:
# ros2 launch path_planning autonomous_stack.launch.py

# 3. Monitor system performance (in another terminal)
make attach
# jtop  # Jetson system monitor
# ros2 topic hz /scan  # LiDAR data rate
```
  * Autonomous Operation: Complete navigation pipeline from sensors to motors
  * Real-time Monitoring: System performance and sensor data validation
  * Emergency Handling: Safe shutdown and error recovery mechanisms

— **Manual Control Operation**
```bash
# Launch wheel control interface
make wheels-control

# Or using existing container
make attach
# ros2 launch wheels_control manual_stack.launch.py
```
  * Teleoperation: Direct manual control for testing and emergency situations
  * Sensor Monitoring: Real-time sensor data display during manual operation
  * Safety Systems: Integrated safety limits and emergency stops

— **Development and Testing**
```bash
# Access development environment
make attach

# Run navigation tests
make nav-test

# Check wheels control status
make wheels-status

# Monitor performance
make performance-check

# Inside container - development tools
# ros2 run rqt_graph rqt_graph
# ros2 topic echo /nav2/global_costmap/costmap
# rviz2  # Visualization (if display available)
```
```

## Hardware Configuration

— **Sensor Calibration**
  * Camera Intrinsics: Intel RealSense factory calibration with custom parameters
  * LiDAR-Camera: Extrinsic calibration between sensors
  * IMU-Base: Inertial measurement unit to robot base frame transforms
  * Wheel Odometry: Encoder calibration and kinematic parameters

— **Network Setup**
  * LiDAR Interface: Static IP configuration (10.5.5.1/24)
  * DDS Configuration: Multi-interface networking with CycloneDDS
  * Remote Access: SSH and VNC setup for remote operation
  * Firewall Rules: Proper port configuration for ROS2 communication

— **Power Management**
  * Performance Modes: Jetson clock configuration for different power profiles
  * Thermal Management: Temperature monitoring and throttling protection
  * Battery Integration: Power monitoring and low-battery handling
  * Graceful Shutdown: Proper system shutdown procedures

## Advanced Configuration

— **CUDA Optimization**
  * Memory Allocation: GPU memory management for point cloud processing
  * Kernel Optimization: Custom CUDA kernels for specific algorithms
  * Memory Bandwidth: Optimized data transfer between CPU and GPU
  * Driver Compatibility: NVIDIA driver version requirements

— **Real-time Performance**
  * CPU Isolation: Core isolation for real-time processes
  * Memory Locking: Preventing memory swapping for critical processes
  * Interrupt Handling: Optimized interrupt routing for sensors
  * Scheduler Configuration: Real-time scheduling for navigation components

## Troubleshooting

— **Hardware Issues**
```bash
# Check Jetson system status
jtop
sudo nvpmodel -q  # Power mode
sudo jetson_clocks  # Performance mode

# Verify sensor connections
lsusb  # USB devices (cameras)
ip addr show  # Network interfaces (LiDAR)
```
  * Symptoms: Sensor data missing or system performance issues
  * Causes: Hardware failures, thermal throttling, or power limitations
  * Solutions: Hardware verification, thermal management, power optimization

— **RealSense Camera Issues**
```bash
# Test RealSense functionality
python3 -c "import pyrealsense2 as rs; print('RealSense OK')"
rs-enumerate-devices  # List connected cameras

# Debug camera permissions
ls -l /dev/video*
groups $USER  # Check user groups
```
  * Symptoms: Camera initialization failures or missing frames
  * Causes: USB permissions, power limitations, or driver issues
  * Solutions: USB rules verification, power supply check, driver updates

— **Network Communication Issues**
```bash
# Test LiDAR connectivity
ping os-122140001125.local
nmap -p 7502 os-122140001125.local

# Debug ROS2 communication
ros2 node list
ros2 topic list
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
  * Symptoms: Missing ROS2 nodes or topics, LiDAR connection failures
  * Causes: Network configuration, DDS middleware, or firewall issues
  * Solutions: Network interface verification, DDS configuration, port access

— **Performance Issues**
```bash
# Monitor system resources
htop
nvidia-smi  # GPU usage
iostat -x 1  # Storage performance

# Check thermal throttling
cat /sys/devices/virtual/thermal/thermal_zone*/temp
```
  * Symptoms: Slow navigation response or system lag
  * Causes: Resource limitations, thermal throttling, or inefficient algorithms
  * Solutions: Performance tuning, cooling improvements, algorithm optimization

## Competition Deployment

— **Pre-Competition Checklist**
  * Hardware Verification: All sensors functional and calibrated
  * Software Testing: Navigation algorithms validated in test environment
  * Performance Validation: Real-time performance under competition conditions
  * Backup Systems: Redundant hardware and software configurations
  * Safety Systems: Emergency stops and fail-safe mechanisms

— **Competition Operation**
  * Autonomous Mode: Fully autonomous navigation for competition tasks
  * Manual Override: Quick switch to manual control when needed
  * Monitoring Tools: Real-time system health and performance monitoring
  * Data Logging: Comprehensive logging for post-competition analysis

## Security Considerations

⚠️ **Important Security Notes**:
- Container operates in privileged mode for hardware access
- Full device access granted for sensor and actuator control
- Network host mode exposes all network interfaces
- Python path modifications may affect system security
- Designed for controlled competition environments only

## Related Documentation

- [Main ERC_NAV README](../README.md)
- [Nav2 Documentation](https://navigation.ros.org/)
- [NVIDIA Jetson Developer Guide](https://developer.nvidia.com/embedded-computing)
- [Intel RealSense Documentation](https://dev.intelrealsense.com/)
- [CycloneDDS Configuration Guide](https://github.com/eclipse-cyclonedx/cyclonedx)

## Support

For Jetson-specific deployment issues:
1. Verify hardware connections and power supply
2. Check Jetson system configuration and performance modes
3. Validate sensor functionality and calibration
4. Monitor system resources and thermal performance
5. Contact EPFLXplore navigation team for competition support