# ERC_HD Docker Jetson Environment

This directory contains the Docker configuration for running the **ERC Handling Device (ERC_HD)** system on **NVIDIA Jetson** embedded platforms using **ROS2 Humble**. This is the production deployment environment optimized for ARM64 architecture and NVIDIA GPU acceleration.

## 🐳 Overview

The Jetson Docker environment provides a complete, containerized production platform for the robotic arm system running on NVIDIA Jetson Xavier NX/AGX hardware. It includes optimized dependencies, real-time configurations, and hardware-specific integrations for embedded deployment in field conditions.

## 📁 Files Description

### `Dockerfile` → Defines the production-ready HD environment for ARM64 Jetson platforms
- **Base Image**: Extends `ghcr.io/epflxplore/docker_commons:humble-jetson-nx-hd`
- **Architecture**: Optimized for ARM64 (aarch64) processors
- **ROS2 Integration**: MoveIt, hardware interfaces, and Jetson-specific packages
- **AI/ML Stack**: PyTorch 2.8.0, TorchVision, Segment Anything, MobileSAM
- **Computer Vision**: OpenCV 4.7.0.72, Intel RealSense, DepthAI support
- **Hardware Libraries**: CUDA integration, Tegra GPU drivers, USB device support
- **Jetson Tools**: jetson-stats for system monitoring and performance tuning
- **NVIDIA Drivers**: Full GPU capabilities with proper udev rules

### `run.sh` → Interactive development launcher with full Jetson hardware access
- **NVIDIA Runtime**: Full GPU acceleration with `--runtime=nvidia --gpus all`
- **Hardware Integration**: Complete device access including USB, video, and input devices
- **Jetson-Specific Mounts**:
  - Tegra libraries: `/usr/lib/aarch64-linux-gnu/tegra` (read-only)
  - CUDA runtime: `/usr/local/cuda` (read-only)
  - JTop monitoring: `/run/jtop.sock`
- **Network**: Host networking for ROS2 multi-robot communication
- **DDS Configuration**: Custom CycloneDDS XML for network interface binding
- **Volume Persistence**: Jetson-optimized home directory and photo storage

### `run_hd_stack.sh` → Production launcher for autonomous HD system operation
- **Auto-Launch**: Directly starts the complete HD stack (`ros2 launch hd_launch new.launch.py`)
- **Container Management**: Smart container reuse - attaches if running, creates if stopped
- **Production Mode**: Non-interactive execution for autonomous operation
- **Resource Optimization**: GPU acceleration with video/render group access
- **Monitoring Integration**: JTop socket for real-time performance monitoring
- **Photo Storage**: Dedicated volume for competition photo capture

### `run_motors.sh` → Dedicated EtherCAT motor control launcher
- **Motor Control**: Isolated container for `ros2 run ethercat_device_configurator motor_control`
- **Root Access**: Runs as root for EtherCAT real-time privileges
- **Hardware Priority**: Direct hardware access for real-time motor communication
- **Safety Isolation**: Separate container prevents motor control interference
- **Network Integration**: Host networking for ROS2 motor command distribution

### `stop_hd_stack.sh` → Safe system shutdown utility
- **Container Management**: Gracefully stops running HD stack containers
- **Status Check**: Verifies container state before attempting shutdown
- **Clean Shutdown**: Ensures proper ROS2 node termination and resource cleanup

### `build.sh` → Production image builder with Jetson optimizations
- **Target Image**: `ghcr.io/epflxplore/hd:humble-jetson-test`
- **Build Context**: Uses parent directory for complete source access
- **Progress Output**: Detailed build logging for troubleshooting
- **Optimization**: ARM64-specific compilation flags and dependencies

### `attach.sh` → Production environment configuration for ROS2 and hardware
- **DDS Middleware**: CycloneDDS with custom network interface configuration
- **Display**: X11 forwarding for remote monitoring and debugging
- **Python Path**: Optimized package paths for custom messages and ROS2 integration
- **ROS2 Setup**: Sources workspace with proper environment variables

### `cyclonedx.xml` → Network-specific DDS configuration for multi-robot communication
- **Network Interface**: Binds to `enP8p1s0` (Jetson Ethernet interface)
- **Multicast**: Configured for SPDP (Simple Participant Discovery Protocol)
- **Domain**: Uses ROS2 Domain ID 0 for system-wide communication
- **Optimization**: Reduces network overhead for embedded deployment

## 🛠️ Jetson-Specific Features

### Hardware Acceleration
- **NVIDIA GPU**: Full CUDA support with optimized drivers
- **Tegra Integration**: Native ARM64 GPU libraries and runtime
- **Video Processing**: Hardware-accelerated computer vision pipelines
- **AI Inference**: Optimized PyTorch and TensorRT integration

### Real-Time Capabilities
- **EtherCAT Support**: Real-time industrial communication protocols
- **Motor Control**: Microsecond-precision joint control
- **Priority Scheduling**: Container privilege escalation for real-time tasks
- **Hardware Interrupts**: Direct access to USB and serial interfaces

### Power Management
- **Thermal Monitoring**: Integration with jetson-stats for temperature control
- **Performance Modes**: Support for Jetson power/performance profiles
- **Resource Optimization**: ARM64-optimized libraries and minimal overhead

## 🚀 Usage

### Prerequisites

1. **NVIDIA Jetson Xavier NX/AGX** with JetPack 5.x
2. **Docker Engine** with NVIDIA container runtime
3. **EtherCAT Hardware** connected via USB/Ethernet
4. **Network Configuration** matching cyclonedx.xml settings

### Production Deployment

#### Complete HD System Launch
```bash
cd /path/to/ERC_HD/docker_humble_jetson
./run_hd_stack.sh
```

#### Motor Control Only
```bash
./run_motors.sh
```

#### Development/Debug Mode
```bash
./run.sh
```

#### System Shutdown
```bash
./stop_hd_stack.sh
```

### Build Custom Image
```bash
./build.sh
```

## 🔧 Configuration

### Network Setup
The system requires proper network interface configuration:
```bash
# Check available interfaces
ip addr show

# Update cyclonedx.xml if using different interface
# Default: enP8p1s0 (Jetson Ethernet)
```

### Performance Tuning
```bash
# Inside container - monitor system performance
jtop

# Set Jetson to maximum performance
sudo jetson_clocks
```

### EtherCAT Configuration
```bash
# Verify EtherCAT device detection
lsusb | grep -i ether

# Check motor control node
ros2 run ethercat_device_configurator motor_control
```

## 🎯 Production Workflows

### Autonomous Operation Sequence
1. **System Boot**: Jetson powers up and loads Docker environment
2. **Stack Launch**: `run_hd_stack.sh` starts complete HD system
3. **Motor Initialization**: EtherCAT motors configure and calibrate
4. **Perception Startup**: Cameras and AI models initialize
5. **ROS2 Integration**: All nodes connect via CycloneDDS
6. **Ready State**: System awaits commands from Control Station

### Competition Deployment
1. **Pre-Launch**: Verify all hardware connections and network
2. **Stack Launch**: Single command system activation
3. **Performance Monitor**: JTop monitoring for thermal/power management
4. **Task Execution**: Autonomous robotic arm operations
5. **Data Capture**: Photos and telemetry saved to persistent volumes
6. **Safe Shutdown**: Graceful system termination

## 🐛 Troubleshooting

### GPU Access Issues
```bash
# Verify NVIDIA runtime
docker run --rm --runtime=nvidia --gpus all nvidia/l4t-base:r35.1.0 nvidia-smi

# Check container GPU access
nvidia-docker run --rm ghcr.io/epflxplore/hd:humble-jetson-test nvidia-smi
```

### EtherCAT Motor Problems
```bash
# Check USB permissions
ls -la /dev/ttyUSB*

# Verify EtherCAT master
sudo dmesg | grep -i ether

# Run motor control with root privileges
./run_motors.sh
```

### Network Communication Issues
```bash
# Test ROS2 discovery
ros2 node list
ros2 topic list

# Check DDS configuration
echo $CYCLONEDDS_URI
cat /home/xplore/cyclonedx.xml
```

### Performance Issues
```bash
# Monitor system resources
jtop
htop

# Check thermal throttling
cat /sys/class/thermal/thermal_zone*/temp

# Verify power mode
sudo nvpmodel -q
```

## 📋 Development Tips

### Container Management
- Use `run_hd_stack.sh` for production deployment
- Use `run.sh` for development and debugging
- Always use `stop_hd_stack.sh` for clean shutdown

### Performance Optimization
- Monitor with `jtop` for real-time system metrics
- Use `jetson_clocks` for maximum performance mode
- Optimize AI models for Jetson architecture

### Debugging
- Attach to running containers: `docker exec -it hd_humble bash`
- Check logs: `docker logs hd_humble`
- Monitor ROS2: `ros2 run rqt_console rqt_console`

## 🔗 Integration

This Jetson environment integrates with:
- **ERC_CS_ControlStation**: Remote command and telemetry
- **ERC_NAV**: Multi-robot coordination via DDS
- **Hardware Systems**: EtherCAT motors, cameras, sensors
- **Competition Infrastructure**: Autonomous task execution

---

**Note**: This environment is optimized for production deployment on NVIDIA Jetson platforms. For development and testing on x86_64 systems, use the `docker_humble_desktop` configuration.