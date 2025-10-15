# ERC_HD Docker Jetson Environment# ERC_HD Docker Jetson Environment



This directory contains the Docker configuration for running the **ERC Handling Device (ERC_HD)** system on **NVIDIA Jetson** embedded platforms using **ROS 2 Humble**. This is the production deployment environment optimized for ARM64 architecture, NVIDIA GPU acceleration, and real-time robotic arm control.This directory contains the Docker configuration for running the **ERC Handling Device (ERC_HD)** system on **NVIDIA Jetson** embedded platforms using **ROS2 Humble**. This is the production deployment environment optimized for ARM64 architecture and NVIDIA GPU acceleration.



## 🐳 Overview## 🐳 Overview



The Jetson Docker environment provides a complete, containerized production platform for the robotic arm system running on NVIDIA Jetson Xavier NX/AGX hardware. It includes optimized dependencies, real-time configurations, and hardware-specific integrations for embedded deployment in field conditions.The Jetson Docker environment provides a complete, containerized production platform for the robotic arm system running on NVIDIA Jetson Xavier NX/AGX hardware. It includes optimized dependencies, real-time configurations, and hardware-specific integrations for embedded deployment in field conditions.



## 📁 Files Description## 📁 Files Description



### Core Files### `Dockerfile` → Defines the production-ready HD environment for ARM64 Jetson platforms

- **Base Image**: Extends `ghcr.io/epflxplore/docker_commons:humble-jetson-nx-hd`

#### `Dockerfile`- **Architecture**: Optimized for ARM64 (aarch64) processors

Defines the production-ready HD environment for ARM64 Jetson platforms:- **ROS2 Integration**: MoveIt, hardware interfaces, and Jetson-specific packages

- **Base Image**: Extends `ghcr.io/epflxplore/docker_commons:humble-jetson-nx-hd`- **AI/ML Stack**: PyTorch 2.8.0, TorchVision, Segment Anything, MobileSAM

- **Architecture**: Optimized for ARM64 (aarch64) processors- **Computer Vision**: OpenCV 4.7.0.72, Intel RealSense, DepthAI support

- **ROS 2 Integration**: MoveIt, hardware interfaces, and Jetson-specific packages- **Hardware Libraries**: CUDA integration, Tegra GPU drivers, USB device support

- **AI/ML Stack**: PyTorch 2.8.0, TorchVision, Segment Anything, MobileSAM- **Jetson Tools**: jetson-stats for system monitoring and performance tuning

- **Computer Vision**: OpenCV 4.7.0.72, Intel RealSense, DepthAI support- **NVIDIA Drivers**: Full GPU capabilities with proper udev rules

- **Hardware Libraries**: CUDA integration, Tegra GPU drivers, USB device support

- **Jetson Tools**: jetson-stats for system monitoring and performance tuning### `run.sh` → Interactive development launcher with full Jetson hardware access

- **NVIDIA Drivers**: Full GPU capabilities with proper udev rules- **NVIDIA Runtime**: Full GPU acceleration with `--runtime=nvidia --gpus all`

- **Hardware Integration**: Complete device access including USB, video, and input devices

#### `Makefile`- **Jetson-Specific Mounts**:

Comprehensive production deployment and development system with all functionality built-in:  - Tegra libraries: `/usr/lib/aarch64-linux-gnu/tegra` (read-only)

```bash  - CUDA runtime: `/usr/local/cuda` (read-only)

make help           # Show all available commands  - JTop monitoring: `/run/jtop.sock`

make build          # Build the HD Jetson Docker image- **Network**: Host networking for ROS2 multi-robot communication

make run            # Run interactive container- **DDS Configuration**: Custom CycloneDDS XML for network interface binding

make run-hd-stack   # Run HD launch stack (production)- **Volume Persistence**: Jetson-optimized home directory and photo storage

make run-motors     # Run motor control container

make attach         # Attach to running container### `run_hd_stack.sh` → Production launcher for autonomous HD system operation

make jetson-check   # Check Jetson hardware availability- **Auto-Launch**: Directly starts the complete HD stack (`ros2 launch hd_launch new.launch.py`)

```- **Container Management**: Smart container reuse - attaches if running, creates if stopped

- **Production Mode**: Non-interactive execution for autonomous operation

All environment setup, hardware configuration, and Jetson-specific optimizations are handled automatically within the Makefile targets - no external scripts required.- **Resource Optimization**: GPU acceleration with video/render group access

- **Monitoring Integration**: JTop socket for real-time performance monitoring

#### `cyclonedds.xml`- **Photo Storage**: Dedicated volume for competition photo capture

Custom CycloneDDS configuration file for optimized ROS 2 communication on Jetson hardware with network interface binding and performance tuning.

### `run_motors.sh` → Dedicated EtherCAT motor control launcher

## 🛠️ Usage Workflows- **Motor Control**: Isolated container for `ros2 run ethercat_device_configurator motor_control`

- **Root Access**: Runs as root for EtherCAT real-time privileges

### Quick Start- **Hardware Priority**: Direct hardware access for real-time motor communication

- **Safety Isolation**: Separate container prevents motor control interference

1. **Check Jetson Environment**:- **Network Integration**: Host networking for ROS2 motor command distribution

   ```bash

   make jetson-check### `stop_hd_stack.sh` → Safe system shutdown utility

   ```- **Container Management**: Gracefully stops running HD stack containers

- **Status Check**: Verifies container state before attempting shutdown

2. **Build the HD Jetson Image**:- **Clean Shutdown**: Ensures proper ROS2 node termination and resource cleanup

   ```bash

   make build### `build.sh` → Production image builder with Jetson optimizations

   ```- **Target Image**: `ghcr.io/epflxplore/hd:humble-jetson-test`

- **Build Context**: Uses parent directory for complete source access

3. **Start HD Jetson System**:- **Progress Output**: Detailed build logging for troubleshooting

   ```bash- **Optimization**: ARM64-specific compilation flags and dependencies

   # Interactive development

   make run### `attach.sh` → Production environment configuration for ROS2 and hardware

   - **DDS Middleware**: CycloneDDS with custom network interface configuration

   # Production HD stack- **Display**: X11 forwarding for remote monitoring and debugging

   make run-hd-stack- **Python Path**: Optimized package paths for custom messages and ROS2 integration

   - **ROS2 Setup**: Sources workspace with proper environment variables

   # Motor control only

   make run-motors### `cyclonedx.xml` → Network-specific DDS configuration for multi-robot communication

   ```- **Network Interface**: Binds to `enP8p1s0` (Jetson Ethernet interface)

- **Multicast**: Configured for SPDP (Simple Participant Discovery Protocol)

4. **Monitor and Debug**:- **Domain**: Uses ROS2 Domain ID 0 for system-wide communication

   ```bash- **Optimization**: Reduces network overhead for embedded deployment

   make attach    # Attach additional terminal

   make status    # Check container status## 🛠️ Jetson-Specific Features

   ```

### Hardware Acceleration

### Available Make Commands- **NVIDIA GPU**: Full CUDA support with optimized drivers

- **Tegra Integration**: Native ARM64 GPU libraries and runtime

- **`make build`**: Build the HD Jetson Docker image with ARM64 optimizations- **Video Processing**: Hardware-accelerated computer vision pipelines

- **`make run`**: Run interactive container for development and debugging- **AI Inference**: Optimized PyTorch and TensorRT integration

- **`make run-hd-stack`**: Run HD launch stack (starts autonomously or attaches if running)

- **`make run-motors`**: Run dedicated motor control container with EtherCAT access### Real-Time Capabilities

- **`make attach`**: Attach to running container with proper environment setup- **EtherCAT Support**: Real-time industrial communication protocols

- **`make stop`**: Stop the main HD Jetson container- **Motor Control**: Microsecond-precision joint control

- **`make stop-hd-stack`**: Alias for stop (stop HD stack container)- **Priority Scheduling**: Container privilege escalation for real-time tasks

- **`make stop-motors`**: Stop the motors container specifically- **Hardware Interrupts**: Direct access to USB and serial interfaces

- **`make clean`**: Remove containers, volumes, and images (with confirmation)

- **`make status`**: Show status of containers, volumes, and images### Power Management

- **`make docker-check`**: Verify Docker is running- **Thermal Monitoring**: Integration with jetson-stats for temperature control

- **`make jetson-check`**: Check Jetson hardware and NVIDIA runtime availability- **Performance Modes**: Support for Jetson power/performance profiles

- **`make x11-setup`**: Setup X11 forwarding manually- **Resource Optimization**: ARM64-optimized libraries and minimal overhead



### Production Deployment Workflow## 🚀 Usage



1. **Verify Jetson Environment**:### Prerequisites

   ```bash

   make jetson-check1. **NVIDIA Jetson Xavier NX/AGX** with JetPack 5.x

   ```2. **Docker Engine** with NVIDIA container runtime

   Should show:3. **EtherCAT Hardware** connected via USB/Ethernet

   - ✓ Tegra libraries detected4. **Network Configuration** matching cyclonedx.xml settings

   - ✓ CUDA installation detected

   - ✓ jtop socket available### Production Deployment

   - ✓ Docker NVIDIA runtime detected

#### Complete HD System Launch

2. **Deploy HD Stack**:```bash

   ```bashcd /path/to/ERC_HD/docker_humble_jetson

   make run-hd-stack./run_hd_stack.sh

   ``````

   This will:

   - Start a new container if none is running#### Motor Control Only

   - Attach to existing container if already running```bash

   - Automatically launch the complete HD stack./run_motors.sh

   - Configure all Jetson-specific hardware access```



3. **Run Motor Control** (if needed separately):#### Development/Debug Mode

   ```bash```bash

   make run-motors./run.sh

   ``````

   For dedicated EtherCAT motor control with root privileges.

#### System Shutdown

4. **Monitor System**:```bash

   ```bash./stop_hd_stack.sh

   # Check container status```

   make status

   ### Build Custom Image

   # Attach for debugging```bash

   make attach./build.sh

   ```

   # Inside container, monitor Jetson performance

   jtop## 🔧 Configuration

   ```

### Network Setup

### Development WorkflowThe system requires proper network interface configuration:

```bash

1. **Interactive Development**:# Check available interfaces

   ```baship addr show

   make run

   ```# Update cyclonedx.xml if using different interface

# Default: enP8p1s0 (Jetson Ethernet)

2. **Inside Container Development**:```

   ```bash

   # Build the workspace### Performance Tuning

   colcon build```bash

   # Inside container - monitor system performance

   # Source the environmentjtop

   source install/setup.bash

   # Set Jetson to maximum performance

   # Launch individual componentssudo jetson_clocks

   ros2 launch hd_launch new.launch.py```

   

   # Test motor control### EtherCAT Configuration

   ros2 run ethercat_device_configurator motor_control```bash

   # Verify EtherCAT device detection

   # Monitor system performancelsusb | grep -i ether

   jtop

   ```# Check motor control node

ros2 run ethercat_device_configurator motor_control

3. **Production Testing**:```

   ```bash

   # Test full stack## 🎯 Production Workflows

   make run-hd-stack

   ### Autonomous Operation Sequence

   # In another terminal, monitor1. **System Boot**: Jetson powers up and loads Docker environment

   make attach2. **Stack Launch**: `run_hd_stack.sh` starts complete HD system

   ```3. **Motor Initialization**: EtherCAT motors configure and calibrate

4. **Perception Startup**: Cameras and AI models initialize

## 🔧 Environment Configuration5. **ROS2 Integration**: All nodes connect via CycloneDDS

6. **Ready State**: System awaits commands from Control Station

### ROS 2 Configuration

- **Distribution**: ROS 2 Humble Hawksbill### Competition Deployment

- **RMW Implementation**: CycloneDDS (rmw_cyclonedds_cpp)1. **Pre-Launch**: Verify all hardware connections and network

- **DDS Configuration**: Custom `cyclonedds.xml` for Jetson networking2. **Stack Launch**: Single command system activation

- **Domain ID**: Default (0) - configurable via `ROS_DOMAIN_ID`3. **Performance Monitor**: JTop monitoring for thermal/power management

- **Architecture**: ARM64 native for Jetson hardware4. **Task Execution**: Autonomous robotic arm operations

5. **Data Capture**: Photos and telemetry saved to persistent volumes

### Automatic Environment Setup6. **Safe Shutdown**: Graceful system termination

The Makefile automatically configures all necessary environment variables and hardware access:

- **RMW_IMPLEMENTATION**: `rmw_cyclonedds_cpp`## 🐛 Troubleshooting

- **DISPLAY**: X11 display forwarding

- **QT_X11_NO_MITSHM**: X11 compatibility### GPU Access Issues

- **PYTHONPATH**: Complete ROS 2 Python package paths```bash

- **CYCLONEDDS_URI**: Custom DDS configuration file# Verify NVIDIA runtime

- **GPU Access**: NVIDIA runtime with all GPU devicesdocker run --rm --runtime=nvidia --gpus all nvidia/l4t-base:r35.1.0 nvidia-smi

- **Hardware Groups**: video, render, input group access

# Check container GPU access

### Jetson-Specific Hardware Integrationnvidia-docker run --rm ghcr.io/epflxplore/hd:humble-jetson-test nvidia-smi

```

#### GPU and CUDA Access

- **NVIDIA Runtime**: `--runtime=nvidia --gpus all`### EtherCAT Motor Problems

- **Tegra Libraries**: `/usr/lib/aarch64-linux-gnu/tegra` (read-only)```bash

- **CUDA Runtime**: `/usr/local/cuda` (read-only)# Check USB permissions

- **Hardware Groups**: video, render, input for GPU accessls -la /dev/ttyUSB*



#### Real-Time Hardware Access# Verify EtherCAT master

- **Privileged Mode**: Full hardware access for EtherCAT and motor controlsudo dmesg | grep -i ether

- **Host Networking**: Direct network interface access

- **PID Namespace**: `--pid=host` for real-time performance# Run motor control with root privileges

- **IPC Namespace**: `--ipc=host` for shared memory access./run_motors.sh

- **USB Device Access**: `--device /dev/bus/usb` for USB devices```



#### Jetson Monitoring and Optimization### Network Communication Issues

- **JTop Integration**: `/run/jtop.sock` for real-time performance monitoring```bash

- **System Statistics**: Full access to Jetson power and thermal management# Test ROS2 discovery

- **Performance Tuning**: Optimized for real-time robotic controlros2 node list

ros2 topic list

### Volume Mounts Explained

# Check DDS configuration

| Host Path | Container Path | Purpose |echo $CYCLONEDDS_URI

|-----------|----------------|---------|cat /home/xplore/cyclonedx.xml

| `../` (parent directory) | `/home/xplore/dev_ws/src` | Live source code editing |```

| `/home/xplore-hd/Documents/photos_competition` | `/home/xplore/dev_ws/photos_competition` | Competition photo storage |

| `hd_humble_jetson_home_volume` | `/home/xplore` | Persistent home directory |### Performance Issues

| `/dev` | `/dev` | Complete hardware device access |```bash

| `/run/jtop.sock` | `/run/jtop.sock` | Jetson system monitoring |# Monitor system resources

| `/usr/lib/aarch64-linux-gnu/tegra` | `/usr/lib/aarch64-linux-gnu/tegra` | Tegra GPU libraries (ro) |jtop

| `/usr/local/cuda` | `/usr/local/cuda` | CUDA runtime libraries (ro) |htop

| `cyclonedds.xml` | `/home/xplore/cyclonedds.xml` | DDS configuration (ro) |

# Check thermal throttling

## 🔍 Troubleshootingcat /sys/class/thermal/thermal_zone*/temp



### Jetson Hardware Issues# Verify power mode

```bashsudo nvpmodel -q

# Check Jetson environment```

make jetson-check

## 📋 Development Tips

# Verify NVIDIA runtime

docker info | grep nvidia### Container Management

- Use `run_hd_stack.sh` for production deployment

# Check GPU access- Use `run.sh` for development and debugging

nvidia-smi- Always use `stop_hd_stack.sh` for clean shutdown



# Monitor Jetson performance### Performance Optimization

sudo jtop- Monitor with `jtop` for real-time system metrics

```- Use `jetson_clocks` for maximum performance mode

- Optimize AI models for Jetson architecture

### Container Issues

```bash### Debugging

# Check container status- Attach to running containers: `docker exec -it hd_humble bash`

make status- Check logs: `docker logs hd_humble`

- Monitor ROS2: `ros2 run rqt_console rqt_console`

# Stop and restart if needed

make stop## 🔗 Integration

make run-hd-stack

```This Jetson environment integrates with:

- **ERC_CS_ControlStation**: Remote command and telemetry

### EtherCAT Motor Control Issues- **ERC_NAV**: Multi-robot coordination via DDS

```bash- **Hardware Systems**: EtherCAT motors, cameras, sensors

# Run dedicated motor container- **Competition Infrastructure**: Autonomous task execution

make run-motors

---

# Check EtherCAT permissions (on host)

sudo dmesg | grep -i ethercat**Note**: This environment is optimized for production deployment on NVIDIA Jetson platforms. For development and testing on x86_64 systems, use the `docker_humble_desktop` configuration.

# Verify motor container is running as root
docker exec -it hd_humble_motors whoami
```

### ROS 2 Communication Issues
```bash
# Inside container, verify DDS configuration
echo $CYCLONEDDS_URI
cat /home/xplore/cyclonedds.xml

# Test ROS 2 communication
ros2 node list
ros2 topic list
ros2 topic echo /joint_states
```

### Performance Issues
```bash
# Monitor Jetson performance
make attach
jtop

# Check GPU utilization
nvidia-smi -l 1

# Monitor container resources
docker stats hd_humble
```

## 🚀 Development Tips

### Production Mode
- Use `make run-hd-stack` for autonomous operation
- Container automatically restarts HD stack if it exits
- Monitor with `make status` and `make attach`

### Development Mode
- Use `make run` for interactive development
- Full access to build system and debugging tools
- Hot-reload Python code, rebuild C++ as needed

### Performance Optimization
- Image optimized for ARM64 Jetson architecture
- GPU acceleration available for computer vision and AI tasks
- Real-time configuration for robotic control loops
- Custom DDS configuration for network optimization

### Hardware Integration
- **EtherCAT Support**: Real-time motor control with dedicated container
- **GPU Acceleration**: CUDA-enabled computer vision and AI inference
- **Jetson Monitoring**: Full jtop integration for performance tuning
- **USB Device Access**: Complete hardware peripheral support

## 📋 Migration from Shell Scripts

This Makefile replaces the following shell scripts with **zero external dependencies**:
- `build.sh` → `make build`
- `run.sh` → `make run` (interactive development)
- `run_hd_stack.sh` → `make run-hd-stack` (production stack)
- `run_motors.sh` → `make run-motors` (motor control)
- `attach.sh` → `make attach` (environment setup now inline)
- `stop_hd_stack.sh` → `make stop-hd-stack`

All functionality has been preserved and enhanced with:
- **Zero External Scripts**: Everything is self-contained in the Makefile
- **Jetson Hardware Detection**: Automatic checking of Jetson-specific resources
- **Better Error Handling**: Comprehensive status reporting and validation
- **Unified Interface**: Consistent commands across development and production
- **Automatic Environment Setup**: No manual configuration required
- **Enhanced Hardware Integration**: Built-in Jetson optimization and monitoring

## 🏗️ System Requirements

### Hardware Requirements
- **NVIDIA Jetson**: Xavier NX, AGX Xavier, or Orin platform
- **Memory**: 8GB RAM minimum (16GB recommended for AI workloads)
- **Storage**: 32GB available space for images and volumes
- **USB Ports**: For EtherCAT adapter and peripheral devices

### Software Requirements
- **JetPack**: Latest JetPack SDK with CUDA support
- **Docker**: Docker Engine with NVIDIA runtime support
- **NVIDIA Runtime**: `nvidia-docker2` package installed
- **EtherCAT**: EtherCAT master drivers for motor control
- **X11**: X11 server for GUI applications (if needed)

### Jetson-Specific Setup
- **Power Mode**: Set to maximum performance mode
- **Fan Control**: Ensure adequate cooling for sustained operation
- **Storage**: Use high-speed SD card or NVMe SSD for optimal performance
- **Network**: Configure network interfaces for DDS communication