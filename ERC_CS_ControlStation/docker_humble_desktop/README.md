# Docker Humble Desktop - Control Station Containerization

This directory contains the Docker infrastructure for running the ERC Control Station in a containerized environment with ROS 2 Humble and desktop GUI support.

## Overview

The Docker setup provides a complete ROS 2 Humble environment with:
- ROSBridge server for web-ROS communication
- Node.js environment for React frontend
- X11 forwarding for GUI applications
- CycloneDDS middleware for ROS 2 communication
- Network configuration for rover communication

## Files Description

### Core Files

#### `Dockerfile`
Builds the control station image with:
- **Base Image**: `ghcr.io/epflxplore/docker_commons:humble-desktop`
- **ROS 2 Packages**: ROSBridge server, CycloneDDS middleware
- **Node.js**: Version 20.14.0 installed via NVM
- **Python Dependencies**: pymongo, tornado for ROSBridge
- **Network Tools**: net-tools, iputils-ping

#### `compose.yaml`
Docker Compose configuration defining:
- **Container Name**: `cs_humble_desktop`
- **Port Mapping**: 
  - `3000:3000` - React development server
  - `9090:9090` - ROSBridge WebSocket server
- **Volume Mounts**: X11 forwarding, source code, persistent home directory
- **Environment**: X11 display configuration

#### `cyclonedds.xml`
CycloneDDS configuration for ROS 2 communication:
- **Domain ID**: 0 (default ROS 2 domain)
- **Network Interface**: `eno2` (configurable)
- **Multicast**: Enabled
- **Peer Discovery**: Pre-configured rover IP addresses
  - `169.254.55.230`
  - `169.254.55.231`

### Execution Scripts

#### `build.sh`
Builds the Docker image:
```bash
docker build --progress=plain -t ghcr.io/epflxplore/cs:humble-desktop -f Dockerfile ..
```

#### `run_cs.sh` (Linux)
Runs the control station with X11 forwarding on Linux:
- Sets up X11 authentication
- Mounts display and audio devices
- Launches the control station automatically
- Uses direct Docker run command

#### `run_cs_mac.sh` (macOS)
Runs the control station on macOS:
- Similar X11 setup for macOS
- Uses Docker Compose instead of direct run
- Exports environment variables for compose

#### `run.sh`
Generic container runner:
- Sets up X11 forwarding
- Provides interactive shell access
- Does not auto-launch control station

#### `attach.sh`
Environment setup script for manual container access:
- Exports ROS 2 environment variables
- Sets CycloneDDS middleware
- Configures Python paths
- Sets up X11 forwarding variables

## Usage

### Quick Start

1. **Build the image:**
   ```bash
   ./build.sh
   ```

2. **Run on Linux:**
   ```bash
   ./run_cs.sh
   ```

3. **Run on macOS:**
   ```bash
   ./run_cs_mac.sh
   ```

### Advanced Usage

#### Interactive Development
For development with shell access:
```bash
./run.sh
```

Then inside the container:
```bash
source ./attach.sh
./launch.sh
```

#### Manual Container Access
If you need to attach to a running container:
```bash
docker exec -it cs_humble_desktop /bin/bash
source ./attach.sh
```

### Network Configuration

#### Rover Communication
Edit `cyclonedx.xml` to match your network setup:

```xml
<NetworkInterfaceAddress>your_interface</NetworkInterfaceAddress>
<Peers>
  <Peer address="rover_ip_1"/>
  <Peer address="rover_ip_2"/>
</Peers>
```

Common network interfaces:
- **Ethernet**: `eth0`, `eno1`, `eno2`
- **WiFi**: `wlan0`, `wlp3s0`
- **Docker**: `docker0`

#### Port Configuration
The default ports are:
- **3000**: React development server
- **9090**: ROSBridge WebSocket server

To change ports, modify `compose.yaml`:
```yaml
ports:
  - "new_frontend_port:3000"
  - "new_rosbridge_port:9090"
```

## Troubleshooting

### X11 Display Issues

If GUI applications don't display:

1. **Check X11 permissions:**
   ```bash
   xhost +local:docker
   ```

2. **Verify DISPLAY variable:**
   ```bash
   echo $DISPLAY
   ```

3. **Clean X11 auth file:**
   ```bash
   sudo rm -rf /tmp/.docker.xauth
   ```

### Container Access Issues

1. **Permission denied:**
   - Run scripts with sudo if needed
   - Check Docker daemon is running

2. **Volume mount issues:**
   - Ensure source directory exists
   - Check file permissions

### Network Connectivity

1. **ROSBridge connection fails:**
   - Verify port 9090 is not in use
   - Check firewall settings
   - Ensure ROSBridge server is running

2. **Rover communication issues:**
   - Verify IP addresses in `cyclonedds.xml`
   - Check network interface configuration
   - Test network connectivity with ping

### Node.js/npm Issues

1. **Package installation fails:**
   ```bash
   # Inside container
   cd /home/xplore/dev_ws/src/frontend
   npm install
   ```

2. **Version conflicts:**
   ```bash
   # Check Node.js version
   node --version  # Should be 20.14.0
   ```

## Development Workflow

### Local Development
For rapid development, you can:

1. Run container in background:
   ```bash
   docker compose up -d
   ```

2. Attach for development:
   ```bash
   docker exec -it cs_humble_desktop /bin/bash
   ```

3. Make changes to mounted source code
4. Restart services as needed

### Building for Production
The image can be used for production deployments:
```bash
docker build -t cs:production .
docker run -d --name cs_prod cs:production
```

## Volume Management

### Persistent Data
The setup uses a named volume for user data:
```bash
# List volumes
docker volume ls

# Inspect volume
docker volume inspect cs_humble_desktop_home_volume

# Backup volume
docker run --rm -v cs_humble_desktop_home_volume:/data -v $(pwd):/backup ubuntu tar czf /backup/backup.tar.gz /data
```

### Source Code Mounting
Source code is mounted from the parent directory, allowing:
- Real-time code changes
- Persistent modifications
- Easy debugging

## Security Considerations

- Container runs with privileged access for hardware access
- X11 forwarding exposes display server
- Network host mode provides full network access
- Device mounting (`/dev`) gives hardware access

For production, consider:
- Running without privileged mode
- Using specific device mounts instead of `/dev`
- Implementing proper network isolation
- Using specific X11 authentication

## Environment Variables

Key environment variables set in the container:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedx_cpp  # ROS 2 middleware
NODE_VERSION=20.14.0                  # Node.js version
DISPLAY=unix$DISPLAY                  # X11 display
QT_X11_NO_MITSHM=1                   # Qt X11 compatibility
```

## Dependencies

### External Dependencies
- Docker Engine
- Docker Compose
- X11 server (for GUI)
- Network connectivity to rover

### Internal Dependencies
- EPFLXplore Docker Commons base image
- ROS 2 Humble packages
- Node.js ecosystem
- Python packages for ROSBridge