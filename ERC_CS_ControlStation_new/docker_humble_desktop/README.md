# Docker Humble Desktop - Control Station Containerization

This directory contains the Docker infrastructure for running the ERC Control Station in a containerized environment with ROS 2 Humble and desktop GUI support.

## Overview

The Docker setup provides a complete ROS 2 Humble environment with:
- ROSBridge server for web-ROS communication
- Node.js environment for React frontend
- X11 forwarding for GUI applications
- CycloneDDS middleware for ROS 2 communication
- Network configuration for rover communication

## Quick Start

To see all available commands:
```bash
make help
```

Common workflows:
```bash
# Build the Control Station image
make build

# Check configuration
make config-check

# Run Control Station container (interactive)
make run

# Run with launch script execution
make run-cs

# Run using Docker Compose (macOS optimized)
make run-mac

# In another terminal, attach to running container
make attach

# Check web interface status
make web-status

# Stop the container
make stop
```

## Configuration

All container management is handled through the comprehensive **Makefile** which provides:

### Core Commands
- `make build` - Build the Control Station image
- `make build-quick` - Build using cache (faster)
- `make run` - Run Control Station container with X11 forwarding
- `make run-cs` - Run Control Station with launch script execution
- `make run-mac` - Run using Docker Compose (macOS optimized)
- `make attach` - Connect to running container with ROS2 environment
- `make stop` - Stop all Control Station containers and services

### Docker Compose Operations
- `make compose-up` - Start services using Docker Compose (background mode)
- `make compose-down` - Stop Docker Compose services
- `make compose-logs` - Show Docker Compose logs
- `make compose-shell` - Access shell in Docker Compose service

### Development & Testing
- `make dev-shell` - Start development shell without auto-setup
- `make cs-test` - Run Control Station tests
- `make launch-cs` - Launch Control Station stack
- `make web-status` - Check web interface accessibility
- `make restart` - Restart all services

### Maintenance & Diagnostics
- `make status` - Show comprehensive status (containers, services, ports)
- `make config-check` - Validate configuration files and Docker setup
- `make logs` - Show container/service logs
- `make clean` - Remove containers, volumes, and images
- `make port-forward` - Show port forwarding setup for remote access

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

#### `Makefile`
Comprehensive container management system:
- **Build Management**: Image building with caching options
- **Multi-Mode Operation**: Interactive, launch script, and Docker Compose modes
- **Web Interface Support**: Port management and accessibility checking
- **Development Tools**: Testing, debugging, and configuration validation
- **X11 Configuration**: Advanced display forwarding setup
- **Service Management**: Both standalone and compose-based operations

## Usage

### Quick Start

1. **Build the image:**
   ```bash
   make build
   ```

2. **Run on Linux:**
   ```bash
   make run-cs
   ```

3. **Run on macOS:**
   ```bash
   make run-mac
   ```

4. **Check web interfaces:**
   ```bash
   make web-status
   ```

### Advanced Usage

#### Interactive Development
For development with shell access:
```bash
make run
```

Then inside the container:
```bash
./launch.sh
```

#### Docker Compose Mode
For background operation:
```bash
# Start services in background
make compose-up

# Check status
make status

# Access shell
make compose-shell

# Stop services
make compose-down
```

#### Manual Container Access
If you need to attach to a running container:
```bash
docker exec -it cs_humble_desktop /bin/bash
source ./attach.sh
```

```bash
make attach
```

### Web Interface Access

The Control Station provides two web interfaces:

1. **Main Interface (Port 3000)**: React frontend for control station UI
2. **ROSBridge (Port 9090)**: WebSocket server for ROS 2 communication

Access them at:
- http://localhost:3000 (Main Interface)
- http://localhost:9090 (ROSBridge)

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

### Remote Access

For remote access to the web interfaces:

```bash
# Show port forwarding setup
make port-forward

# SSH tunnel from remote machine
ssh -L 3000:localhost:3000 -L 9090:localhost:9090 user@control-station-host
```

## Troubleshooting

### X11 Display Issues

If GUI applications don't display:

1. **Clean X11 setup:**
   ```bash
   sudo rm -rf /tmp/.docker.xauth
   make x11-setup
   ```

2. **Check X11 permissions:**
   ```bash
   xhost +local:docker
   ```

3. **Verify DISPLAY variable:**
   ```bash
   echo $DISPLAY
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