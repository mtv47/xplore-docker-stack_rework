# Docker Humble GLIM Environment

This directory contains the Docker configuration for running the GLIM (Graph-based LiDAR-Inertial Mapping) SLAM system in a ROS2 Humble environment, specifically optimized for NVIDIA Jetson platforms.

## Overview

The `docker_humble_glim` environment is designed for:
- **Advanced SLAM**: Graph-based LiDAR-Inertial simultaneous localization and mapping
- **GPU Acceleration**: CUDA-optimized GLIM implementation for real-time performance
- **Jetson Optimization**: Specifically configured for NVIDIA Jetson hardware platforms
- **Research & Development**: Cutting-edge SLAM algorithms for robotics applications

## Quick Start

To see all available commands:
```bash
make help
```

Common workflows:
```bash
# Build the GLIM image
make build

# Check Jetson hardware and GLIM requirements
make hardware-check

# Run GLIM container with GPU acceleration
make run

# In another terminal, attach to the running container
make attach

# Check status
make status

# Stop the container
make stop
```

## Configuration

All container management is handled through the comprehensive **Makefile** which provides:

### Core Commands
- `make build` - Build the GLIM Jetson image with fresh cache
- `make build-quick` - Build using cache (faster)
- `make run` - Launch GLIM container with full GPU and hardware access
- `make attach` - Connect to running container with ROS2 environment
- `make stop` - Stop the running container
- `make status` - Show container, volume, and image status

### Development & Testing
- `make dev-shell` - Start development shell without auto-setup
- `make glim-test` - Run GLIM SLAM tests
- `make launch-glim` - Launch GLIM SLAM in running container
- `make performance-check` - Monitor GPU and system performance

### Hardware & Diagnostics
- `make hardware-check` - Verify Jetson platform and GLIM requirements
- `make gpu-check` - Check GPU availability and CUDA libraries
- `make jetson-info` - Detailed Jetson platform information
- `make x11-setup` - Configure X11 forwarding for GUI applications

### Maintenance
- `make clean` - Remove containers, volumes, and images
- `make logs` - Show container logs

## Files Description

— **Dockerfile.glim** → Defines the specialized GLIM SLAM container image
  * Base Image: Uses `dustynv/ros:humble-desktop-l4t-r36.2.0` for Jetson compatibility
  * ROS2 Configuration: Updates ROS2 signing keys and repositories for latest packages
  * OpenCV Cleanup: Removes conflicting OpenCV installations to prevent version issues
  * Koide3 PPA: Adds specialized repository for GLIM and GTSAM-points packages
  * GLIM Installation: Installs `ros-humble-glim-ros-cuda12.6` with CUDA 12.6 support
  * Dependencies: Includes GTSAM, Iridescence, Boost, and other SLAM libraries
  * User Setup: Creates non-root user with sudo privileges for development

— **Makefile** → Comprehensive container management system
  * Build Management: Image building with caching options
  * Hardware Integration: Full Jetson and GPU support with CUDA/Tegra libraries
  * X11 Configuration: Advanced display forwarding setup
  * Development Tools: Multiple container access methods and testing commands
  * Diagnostics: Hardware checking, performance monitoring, platform information
  * Maintenance: Cleanup, status reporting, and log access

## Prerequisites

— **Hardware Requirements**
  * Platform: NVIDIA Jetson Nano, TX2, Xavier NX, or AGX Xavier/Orin
  * GPU: CUDA-capable NVIDIA GPU (integrated or discrete)
  * Memory: Minimum 8GB RAM, 16GB recommended for large-scale mapping
  * Storage: At least 20GB free space for SLAM data and maps
  * Sensors: LiDAR (Ouster, Velodyne) and IMU for SLAM functionality

— **Software Requirements**
  * Docker Engine: Version 20.10+ with NVIDIA Container Runtime
  * NVIDIA Docker: GPU support for CUDA applications
  * JetPack: Latest JetPack SDK for Jetson platforms (if applicable)
  * X11 Server: For SLAM visualization tools (RViz, custom viewers)

## Installation & Setup

— **Step 1: Build the GLIM Image**
```bash
make build
```
  * Base Setup: Pulls Jetson-optimized ROS2 Humble base image
  * GLIM Installation: Installs latest GLIM-ROS with CUDA 12.6 acceleration
  * Dependencies: Adds GTSAM-points, Iridescence visualization, and SLAM tools
  * Configuration: Sets up CycloneDDS middleware and NVIDIA environment

— **Step 2: Verify Hardware**
```bash
make hardware-check
```
  * Platform Detection: Verifies Jetson platform and capabilities
  * GPU Validation: Checks NVIDIA GPU and CUDA library access
  * Libraries: Confirms Tegra and CUDA library availability
  * Permissions: Validates hardware group memberships

— **Step 3: Launch the GLIM Container**
```bash
make run
```
  * Hardware Access: Enables full GPU, sensor, and device connectivity
  * Performance: Configures shared namespaces for optimal SLAM performance
  * Environment: Automatically sets CUDA paths and ROS2 environment
  * Workspace: Mounts navigation source code for development

— **Step 4: Access Development Environment**
```bash
make attach
```
  * Environment Setup: Automatically configures ROS2 and workspace
  * Multi-terminal: Supports multiple concurrent development sessions
  * Debugging: Full access to SLAM visualization and debugging tools

## GLIM SLAM Features

— **Graph-based Optimization**
  * Factor Graphs: Uses GTSAM for robust pose graph optimization
  * Loop Closure: Automatic loop detection and global map correction
  * Multi-sensor Fusion: Combines LiDAR, IMU, and odometry data
  * Incremental Updates: Real-time map updates with sliding window optimization

— **CUDA Acceleration**
  * GPU Processing: CUDA-accelerated point cloud processing and feature extraction
  * Memory Management: Efficient GPU memory allocation for large point clouds
  * Parallel Computing: Multi-threaded processing for real-time performance
  * Hardware Optimization: Jetson-specific optimizations for embedded deployment

— **Advanced Mapping**
  * Submapping: Hierarchical mapping for large-scale environments
  * Dynamic Objects: Robust handling of moving objects in the environment
  * Map Representation: Efficient sparse map representation for storage
  * Global Consistency: Maintains global map consistency through optimization

## Usage Examples

— **Basic SLAM Operation**
```bash
# 1. Start GLIM container
make run

# 2. Launch GLIM SLAM (inside container)
make launch-glim

# Or manually:
# ros2 launch glim_starter glim_starter.launch.py

# 3. Start sensor drivers (in another terminal)
make attach
# ros2 launch ros2_ouster driver_launch.py

# 4. Monitor SLAM performance
# ros2 topic echo /glim_rosnode/pose_corrected
```
  * Real-time Mapping: Processes LiDAR and IMU data for live mapping
  * Pose Estimation: Provides high-accuracy 6DOF pose estimation
  * Map Building: Creates detailed 3D maps of the environment

— **Development and Testing**
```bash
# Run GLIM tests
make glim-test

# Check performance metrics
make performance-check

# Monitor hardware status
make hardware-check

# Show container logs
make logs
```

— **Integration with Navigation Stack**
```bash
# Launch GLIM alongside navigation (inside container)
ros2 launch path_planning autonomous_stack.launch.py use_glim:=true

# Monitor SLAM-Navigation integration
ros2 run rqt_tf_tree rqt_tf_tree
```
  * Transform Integration: Provides localization data to navigation stack
  * Frame Management: Manages coordinate frames between SLAM and navigation
  * Covariance Tuning: Adjusts uncertainty estimates for sensor fusion

— **Map Saving and Loading**
```bash
# Save current SLAM map
ros2 service call /glim_rosnode/save_map std_srvs/srv/Empty

# Load existing map for localization
ros2 param set /glim_rosnode map_path "/path/to/saved/map"
```

## Advanced Configuration

— **GLIM Parameters**
  * Config Path: `/home/xplore/dev_ws/src/localization/lidar/glim_starter/glim_config/config`
  * Sensor Calibration: IMU-LiDAR extrinsic calibration parameters
  * Optimization Settings: Factor graph optimization parameters
  * Memory Management: Point cloud processing and storage settings

— **CUDA Settings**
  * CUDA Version: Configured for CUDA 12.6 compatibility
  * GPU Memory: Automatic GPU memory management for point cloud processing
  * Compute Capability: Optimized for Jetson and desktop GPU architectures
  * Driver Compatibility: Requires NVIDIA drivers 525+ for CUDA 12.6

## Performance Optimization

— **Jetson-Specific Optimizations**
  * Memory Usage: Optimized for limited system memory on embedded platforms
  * CPU-GPU Balance: Efficient workload distribution between ARM CPU and GPU
  * Power Management: Considers thermal and power constraints
  * Real-time Processing: Maintains real-time performance for navigation

— **SLAM Tuning**
  * Keyframe Selection: Intelligent keyframe selection for map efficiency
  * Loop Closure Frequency: Configurable loop closure detection parameters
  * Optimization Frequency: Adjustable graph optimization intervals
  * Point Cloud Downsampling: Configurable voxel grid filtering

## Troubleshooting

— **CUDA Issues**
```bash
# Verify CUDA availability
nvidia-smi
/usr/local/cuda/bin/nvcc --version

# Check GPU memory usage
nvidia-smi -l 1
```
  * Symptoms: SLAM performance degradation or CUDA errors
  * Causes: Insufficient GPU memory or driver compatibility issues
  * Solutions: Adjust point cloud processing parameters or update drivers

— **SLAM Initialization Failures**
```bash
# Check sensor data streams
ros2 topic hz /ouster/points
ros2 topic hz /imu/data

# Verify transform tree
ros2 run tf2_tools view_frames
```
  * Symptoms: SLAM not initializing or losing tracking
  * Causes: Missing sensor data or incorrect calibration
  * Solutions: Verify sensor connections and calibration parameters

— **Memory Issues**
```bash
# Monitor system memory usage
free -h
sudo jtop  # On Jetson systems

# Adjust GLIM memory parameters
# Edit glim_config/config.json
```
  * Symptoms: System crashes or out-of-memory errors
  * Causes: Large point clouds or memory leaks
  * Solutions: Reduce point cloud density or increase system memory

## Integration Notes

— **Navigation Stack Integration**
  * Transform Publishing: GLIM publishes to separate TF tree to avoid conflicts
  * Odometry Republishing: Custom node republishes GLIM pose as navigation odometry
  * Covariance Management: Proper uncertainty propagation to navigation algorithms
  * Frame Coordination: Manages multiple coordinate frames (map, odom, base_link)

— **Sensor Compatibility**
  * LiDAR Support: Optimized for Ouster OS-series sensors
  * IMU Integration: Supports standard IMU interfaces with proper calibration
  * Camera Fusion: Optional visual-inertial integration capabilities
  * Multi-sensor Setup: Handles multiple sensor modalities simultaneously

## Security Considerations

⚠️ **Important Security Notes**:
- Container runs with extensive privileges for hardware access
- GPU and system resources are fully accessible
- Network namespaces are shared with host system
- Intended for controlled development and research environments only

## Related Documentation

- [GLIM Official Repository](https://github.com/koide3/glim)
- [GTSAM Documentation](https://gtsam.org/)
- [Main ERC_NAV README](../README.md)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [NVIDIA Jetson Developer Guide](https://developer.nvidia.com/embedded-computing)

## Support

For GLIM-specific issues:
1. Check GLIM configuration parameters
2. Verify CUDA and GPU setup
3. Monitor system resources and performance
4. Consult GLIM community documentation
5. Contact EPFLXplore navigation team for integration support