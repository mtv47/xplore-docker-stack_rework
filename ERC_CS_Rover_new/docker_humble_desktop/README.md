# Docker Humble Desktop Environment# Docker Humble Desktop Environment



This directory contains the Docker containerization setup for running the EPFLXplore Rover system on desktop/development environments using **ROS 2 Humble**. This containerized environment allows for consistent development, testing, and simulation across different host systems.This directory contains the Docker containerization setup for running the EPFLXplore Rover system on desktop/development environments using **ROS 2 Humble**. This containerized environment allows for consistent development, testing, and simulation across different host systems.



## Overview## Overview



The Docker environment provides:The Docker environment provides:

- **Isolated Development Environment**: Consistent ROS 2 Humble setup across different machines- **Isolated Development Environment**: Consistent ROS 2 Humble setup across different machines

- **GUI Application Support**: X11 forwarding for RViz, Gazebo, and other graphical tools- **GUI Application Support**: X11 forwarding for RViz, Gazebo, and other graphical tools

- **Hardware Access**: Direct access to USB devices, cameras, and other peripherals- **Hardware Access**: Direct access to USB devices, cameras, and other peripherals

- **Cross-Platform Compatibility**: Works on Linux desktop systems with X11- **Cross-Platform Compatibility**: Works on Linux desktop systems with X11

- **Development Volume Mounting**: Live code editing with automatic synchronization- **Development Volume Mounting**: Live code editing with automatic synchronization



## Files Description## Files Description



### Core Files### Core Files



#### `Dockerfile`#### `Dockerfile`

Multi-stage Docker build configuration that:Multi-stage Docker build configuration that:

- **Base Image**: `ghcr.io/epflxplore/docker_commons:humble-desktop`- **Base Image**: `ghcr.io/epflxplore/docker_commons:humble-desktop`

- **Dependencies**: Installs ROS 2 Navigation stack, OpenCV, multimedia codecs- **Dependencies**: Installs ROS 2 Navigation stack, OpenCV, multimedia codecs

- **Python Packages**: Includes networking libraries (aiohttp, aiortc), database support (pymongo), and system monitoring- **Python Packages**: Includes networking libraries (aiohttp, aiortc), database support (pymongo), and system monitoring

- **USB Rules**: Intel Movidius Neural Compute Stick support- **USB Rules**: Intel Movidius Neural Compute Stick support

- **Security**: Removes confidential source code from final image- **Security**: Removes confidential source code from final image



#### `Makefile`#### `Makefile`

Comprehensive build and execution management system that replaces all previous shell scripts:Comprehensive build and execution management system that replaces all previous shell scripts:

```bash```bash

make help    # Show all available commandsmake help    # Show all available commands

make build   # Build the Docker imagemake build   # Build the Docker image

make run     # Run interactive containermake run     # Run interactive container

make attach  # Attach to running containermake attach  # Attach to running container

``````



## Usage Workflows## Usage Workflows



### Quick Start### Quick Start



1. **View Available Commands**:1. **View Available Commands**:

   ```bash   ```bash

   make help   make help

   ```   ```



2. **Build the Development Image**:2. **Build the Development Image**:

   ```bash   ```bash

   make build   make build

   ```   ```



3. **Start Development Container**:3. **Start Development Container**:

   ```bash   ```bash

   # For Linux   # For Linux

   make run   make run

      

   # For Mac with Control Station network   # For Mac with Control Station network

   make run-mac   make run-mac

      

   # For camera testing   # For camera testing

   make run-cameras   make run-cameras

   ```   ```



4. **Attach to Running Container**:4. **Attach to Running Container**:

   ```bash   ```bash

   make attach   make attach

   ```   ```



### Available Make Commands### Available Make Commands



- **`make build`**: Build the Rover Desktop Docker image with no cache- **`make build`**: Build the Rover Desktop Docker image with no cache

- **`make run`**: Run interactive container with full X11 forwarding and hardware access- **`make run`**: Run interactive container with full X11 forwarding and hardware access

- **`make run-mac`**: Run for macOS with custom network for Control Station integration- **`make run-mac`**: Run for macOS with custom network for Control Station integration

- **`make run-cameras`**: Run camera service node automatically- **`make run-cameras`**: Run camera service node automatically

- **`make attach`**: Attach to running container with proper environment setup- **`make attach`**: Attach to running container with proper environment setup

- **`make stop`**: Stop the running container- **`make stop`**: Stop the running container

- **`make clean`**: Remove containers, volumes, and images (with confirmation)- **`make clean`**: Remove containers, volumes, and images (with confirmation)

- **`make status`**: Show status of containers, volumes, and images- **`make status`**: Show status of containers, volumes, and images

- **`make docker-check`**: Verify Docker is running- **`make docker-check`**: Verify Docker is running

- **`make x11-setup`**: Setup X11 forwarding manually- **`make x11-setup`**: Setup X11 forwarding manually



### Development Workflow### Development Workflow



1. **Build the Development Image**:1. **Build the Development Image**:

   ```bash   ```bash

   make build   make build

   ```   ```



2. **Start Development Container**:2. **Start Development Container**:

   ```bash   ./run.sh

   make run   

   ```   # For macOS

   ./run_mac.sh

3. **Develop Inside Container**:   ```

   ```bash

   # Build your packages3. **Inside the Container**:

   colcon build   ```bash

      # Build the workspace

   # Source the workspace   colcon build

   source install/setup.bash   

      # Source the environment

   # Run the rover node   source install/setup.bash

   ros2 launch rover_pkg launch.py   

   ```   # Run the rover node

   ros2 launch rover_pkg launch.py

### Camera Development Workflow   ```



1. **Build the Image** (if not already built):### Camera Development Workflow

   ```bash

   make build1. **Build the Image** (if not already built):

   ```   ```bash

   ./build.sh

2. **Run Camera Testing**:   ```

   ```bash

   make run-cameras2. **Run Camera Testing**:

   ```   ```bash

   This automatically builds and starts the camera node for testing.   ./run_cameras.sh

   ```

### Multi-Container Setup (with Control Station)   This automatically builds and starts the camera node for testing.



For integrated development with the Control Station:### Multi-Container Setup (with Control Station)



1. **Start Control Station Network**:For integrated development with the Control Station:

   ```bash

   # In ERC_CS_ControlStation/docker_humble_desktop1. **Start Control Station Network**:

   docker-compose up   ```bash

   ```   # In ERC_CS_ControlStation/docker_humble_desktop

   docker-compose up

2. **Connect Rover to CS Network**:   ```

   ```bash

   # Use run-mac command which connects to the CS network2. **Connect Rover to CS Network**:

   make run-mac   ```bash

   ```   # Use run_mac.sh which connects to the CS network

   ./run_mac.sh

## Environment Configuration   ```



### ROS 2 Configuration## Environment Configuration

- **Distribution**: ROS 2 Humble Hawksbill

- **RMW Implementation**: CycloneDDS (rmw_cyclonedds_cpp)### ROS 2 Configuration

- **Domain ID**: Default (0) - can be overridden with `ROS_DOMAIN_ID`- **Distribution**: ROS 2 Humble Hawksbill

- **Network Discovery**: Multicast enabled for cross-container communication- **RMW Implementation**: CycloneDDS (rmw_cyclonedds_cpp)

- **Domain ID**: Default (0) - can be overridden with `ROS_DOMAIN_ID`

### Volume Mounts Explained- **Network Discovery**: Multicast enabled for cross-container communication



| Host Path | Container Path | Purpose |### Volume Mounts Explained

|-----------|----------------|---------|

| `../` (parent directory) | `/home/xplore/dev_ws/src` | Live source code editing || Host Path | Container Path | Purpose |

| `rover_humble_desktop_home_volume` | `/home/xplore` | Persistent home directory ||-----------|----------------|---------|

| `/dev` | `/dev` | Hardware device access || `../` (parent directory) | `/home/xplore/dev_ws/src` | Live source code editing |

| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 socket for GUI apps || `rover_humble_desktop_home_volume` | `/home/xplore` | Persistent home directory |

| `/run/jtop.sock` | `/run/jtop.sock` | Jetson stats monitoring || `/dev` | `/dev` | Hardware device access |

| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 socket for GUI apps |

### Network Modes| `/run/jtop.sock` | `/run/jtop.sock` | Jetson stats monitoring |



- **Host Networking** (`make run`): Direct access to host network interfaces### Network Modes

- **Custom Network** (`make run-mac`): Integration with Control Station Docker network

- **Privileged Mode**: Required for hardware access and system-level operations- **Host Networking** (`run.sh`): Direct access to host network interfaces

- **Custom Network** (`run_mac.sh`): Integration with Control Station Docker network

## Troubleshooting- **Privileged Mode**: Required for hardware access and system-level operations



### X11 Display Issues## Troubleshooting

```bash

# Setup X11 manually if needed### X11 Display Issues

make x11-setup```bash

# Remove corrupted Xauthority file

# Or restart the containersudo rm -rf /tmp/.docker.xauth

make stop

make run# Restart the container

```./run.sh

```

### Permission Issues

```bash### Permission Issues

# The container automatically fixes ownership on startup```bash

# If issues persist, check Docker daemon permissions# The container automatically fixes ownership on startup

make docker-check# If issues persist, run as root temporarily

```sudo ./run.sh

```

### ROS 2 Communication Issues

```bash### ROS 2 Communication Issues

# Inside container, check RMW configuration```bash

echo $RMW_IMPLEMENTATION# Inside container, check RMW configuration

# Should output: rmw_cyclonedds_cppecho $RMW_IMPLEMENTATION

# Should output: rmw_cyclonedds_cpp

# Test ROS 2 discovery

ros2 node list# Test ROS 2 discovery

ros2 topic listros2 node list

```ros2 topic list

```

### Container Already Running

```bash### Container Already Running

# Stop existing container```bash

make stop# Stop existing container

docker stop rover_humble_desktop

# Check status

make status# Or use --rm flag (already included in scripts)

``````



## Development Tips## Development Tips



### Hot Reloading### Hot Reloading

- Python files are automatically updated due to volume mounting- Python files are automatically updated due to volume mounting

- C++ files require `colcon build` inside the container- C++ files require `colcon build` inside the container

- Use `colcon build --packages-select <package_name>` for faster incremental builds- Use `colcon build --packages-select <package_name>` for faster incremental builds



### Container Management### Debugging

- Use `make status` to check container and volume status```bash

- Use `make clean` to completely reset the environment (will ask for confirmation)# Access running container

- Use `make attach` to attach additional terminals to a running containerdocker exec -it rover_humble_desktop /bin/bash



### Environment Variables# Check container logs

All environment variables are automatically configured by the Makefile:docker logs rover_humble_desktop

- `XAUTH`: X11 authority file path

- `PARENT_DIR`: Source code directory# Monitor resource usage

- `DISPLAY`: X11 display for GUI applicationsdocker stats rover_humble_desktop

```

## Migration from Shell Scripts

### Integration Testing

This Makefile replaces the following shell scripts:- Use `run_mac.sh` to connect to Control Station network

- `build.sh` → `make build`- Test bidirectional communication between Rover and CS containers

- `run.sh` → `make run`- Monitor network traffic with ROS 2 introspection tools

- `run_mac.sh` → `make run-mac`

- `run_cameras.sh` → `make run-cameras`## Hardware Requirements

- `attach.sh` → `make attach`

### Host System

All functionality has been preserved and enhanced with additional features like status checking, cleanup, and better error handling.- **OS**: Ubuntu 20.04+ or macOS with XQuartz
- **Docker**: Version 20.10+
- **RAM**: Minimum 4GB available for container
- **Storage**: 10GB for image and volumes

### Connected Hardware
- USB cameras (Intel RealSense recommended)
- Gamepad controllers
- Serial devices (for motor controllers)
- Network interfaces (for rover communication)

This containerized environment ensures consistent development experience while providing full access to the rover's hardware capabilities and GUI tools necessary for development and testing.