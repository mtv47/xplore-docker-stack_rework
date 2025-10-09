# Control Station Docker Compose Setup

This directory contains the Docker Compose configuration for the ERC Control Station system using ROS 2 Humble.

## 🚀 Quick Start

### Prerequisites

- Docker (with Compose V2)
- X11 forwarding support (for GUI applications)
- `xauth` utility for X11 authentication

### Basic Usage

```bash
# Start the Control Station
make up

# Or use Docker Compose directly
docker compose up

# Stop the Control Station
make down

# Attach to running container with full environment
make attach

# Open basic shell in container
make shell
```

## 📁 Files Overview

### Core Docker Compose Files
- **`compose.yaml`** - Main Docker Compose configuration
- **`.env`** - Environment variables configuration
- **`Dockerfile`** - Container image definition
- **`.dockerignore`** - Build context optimization

### Scripts and Tools
- **`Makefile`** - Primary interface for all container operations

### Configuration
- **`cyclonedds.xml`** - CycloneDDS middleware configuration
- **`README.md`** - This documentation

## 🛠 Available Commands

### Using Makefile (Recommended)

The Makefile provides the primary interface for all container operations:

```bash
make help            # Show all available commands
make up              # Start Control Station
make down            # Stop Control Station
make restart         # Restart Control Station
make build           # Build the Docker image
make rebuild         # Rebuild and restart
make logs            # Show container logs
make shell           # Open shell in container
make attach          # Attach to running container with full ROS environment
make status          # Show container status
make clean           # Clean up containers and volumes
make health          # Check service health
make backup          # Backup Control Station volumes
make restore         # Restore Control Station volumes
```

### Command Details

- **`make up`**: Automatically sets up X11 forwarding and starts the Control Station
- **`make attach`**: Connects to running container with full ROS 2 environment configured
- **`make shell`**: Opens a basic shell without ROS environment setup
- **`make build`**: Builds the Docker image with no cache
- **`make rebuild`**: Complete rebuild and restart sequence

### Using Docker Compose Directly

For advanced usage:

```bash
# Basic operations
docker compose up                    # Start services
docker compose down                  # Stop services
docker compose build                 # Build images
docker compose logs -f cs            # Follow logs
```

## 🔧 Configuration

### Environment Variables

The `.env` file contains default configuration. Key variables:

```bash
# Display settings
DISPLAY=:0
XAUTH=/tmp/.docker.xauth

# Project paths
PARENT_DIR=..

# ROS Configuration
ROS_DISTRO=humble
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Network ports
CS_FRONTEND_PORT=3000
ROSBRIDGE_PORT=9090

# Container settings
CONTAINER_NAME=cs_humble_desktop
IMAGE_TAG=humble-desktop

# User settings
USER_ID=1000
GROUP_ID=1000
```

### Docker Compose Configuration

The `compose.yaml` file defines:

- **Network Mode**: Host networking for ROS communication
- **Volumes**: Source code, X11 sockets, device access
- **Environment**: ROS and display variables
- **Ports**: Web interface (3000) and ROSBridge (9090)
- **Health Checks**: Service availability monitoring

## 🌐 Networking

### Exposed Ports

- **Port 3000**: Control Station web interface
- **Port 9090**: ROSBridge WebSocket server

### ROS Communication

- Uses **CycloneDDS** as the RMW implementation
- Host networking mode for optimal ROS 2 communication
- CycloneDDS configuration in `cyclonedds.xml`

## 🖥 X11 Forwarding

X11 forwarding is automatically configured for GUI applications:

1. X authority file is created at `/tmp/.docker.xauth`
2. Display environment is passed to container
3. X11 sockets are mounted for GUI access

### Troubleshooting X11

If GUI applications don't work:

```bash
# Remove corrupted X authority file
sudo rm -rf /tmp/.docker.xauth

# Restart the Control Station
make restart
```

## 💾 Data Persistence

### Volumes

- **`cs_humble_desktop_home_volume`**: User home directory
- **`cs_humble_desktop_logs`**: ROS log files
- **Source code**: Mounted from parent directory

### Backup Important Data

```bash
# Create volume backup using make
make backup

# Restore volume backup using make
make restore

# Manual backup
docker run --rm -v cs_humble_desktop_home_volume:/data -v $(pwd):/backup alpine tar czf /backup/home_backup.tar.gz -C /data .

# Manual restore
docker run --rm -v cs_humble_desktop_home_volume:/data -v $(pwd):/backup alpine tar xzf /backup/home_backup.tar.gz -C /data
```

## 🔍 Monitoring and Debugging

### Check Service Status

```bash
make status
```

### View Logs

```bash
# Follow live logs
make logs

# View specific service logs
docker compose logs -f cs
```

### Container Shell Access

```bash
# Using make (with full ROS environment)
make attach

# Using make (basic shell)
make shell

# Direct Docker Compose
docker compose exec cs /bin/bash
```

### Health Checks

The service includes health checks that verify:
- Web interface availability on port 3000
- Container responsiveness

## 🚧 Development Workflow

### Building and Testing

```bash
# Build new image
make build

# Rebuild and restart
make rebuild

# Check service status
make status

# View logs
make logs

# Open shell for debugging
make shell

# Open shell with full ROS environment
make attach
```

### Code Changes

Source code is mounted from the parent directory, so changes are reflected immediately without rebuilding the container:

```bash
# Your code changes in the parent directory are automatically available
# in the container at /home/xplore/dev_ws/src

# If you need to rebuild ROS packages inside the container:
make attach
# Inside container:
cd /home/xplore/dev_ws
colcon build
source install/setup.bash
```

### Environment Customization

You can customize the environment by:

1. **Modifying `.env` file** for basic configuration
2. **Creating custom override files** for specific needs
3. **Using environment variables** at runtime

```bash
# Custom environment variables
DISPLAY=:1 make up

# Custom compose file
docker compose -f compose.yaml -f my-custom.yaml up
```

## 🐛 Troubleshooting

### Common Issues

1. **Container won't start**
   ```bash
   # Check Docker status
   docker info
   
   # View detailed logs
   make logs
   ```

2. **GUI applications not working**
   ```bash
   # Reset X11 authentication
   sudo rm -rf /tmp/.docker.xauth
   make restart
   ```

3. **Port conflicts**
   ```bash
   # Check port usage
   netstat -tulpn | grep -E "(3000|9090)"
   
   # Modify ports in .env file
   ```

4. **Permission issues**
   ```bash
   # Check volume permissions
   docker compose exec cs ls -la /home/xplore/
   ```

### Logs and Debugging

```bash
# Container logs
docker compose logs cs

# System logs
journalctl -u docker

# ROS logs (inside container)
make attach
tail -f ~/.ros/log/latest/*.log
```

## 🔄 Migration from Legacy Scripts

If migrating from old Docker run scripts:

1. **Environment**: Variables are now managed in `.env`
2. **X11 Setup**: Automated in Makefile targets
3. **Container Management**: Use `make` commands
4. **Networking**: Now uses host networking by default
5. **Multi-service**: Easy to extend with additional services

### Migration Guide

**Old Approach:**
```bash
# Old legacy scripts (removed)
./run.sh              # Enhanced launcher script
./attach.sh           # Container attachment script
./run_cs.sh           # Direct docker run
./run_cs_mac.sh       # macOS-specific docker compose
```

**New Approach:**
```bash
# New unified approach
make up               # Cross-platform Docker Compose with X11 setup
make attach           # Container attachment with full environment
docker compose up     # Direct Docker Compose
```

**Benefits of Migration:**
- ✅ Cross-platform compatibility
- ✅ Better error handling and logging
- ✅ Multi-environment support
- ✅ Easier container management
- ✅ Consistent development experience
- ✅ Integrated X11 setup and validation
- ✅ Single tool for all operations

## 📊 Performance Considerations

### Resource Management

- **Host Networking**: Optimal for ROS 2 communication
- **Volume Mounting**: Direct access to source code for fast development
- **Memory Usage**: Monitor container memory usage for large workspaces

### Resource Limits

Adjust in `compose.yaml`:

```yaml
services:
  cs:
    deploy:
      resources:
        limits:
          cpus: '2.0'
          memory: 4G
        reservations:
          cpus: '1.0'
          memory: 2G
```

### Performance Tips

1. **Use .dockerignore**: Reduce build context size
2. **Multi-stage builds**: Optimize image size (if needed)
3. **Volume caching**: Use tmpfs for temporary files
4. **Resource monitoring**: Use `docker stats cs_humble_desktop`

```bash
# Monitor resource usage
docker stats cs_humble_desktop

# Check volume usage
docker system df

# Clean up unused resources
make clean
# or
docker system prune -a
```

## 🔐 Security Notes

### Current Security Configuration

- Container runs in **privileged mode** for device access
- **X11 forwarding** requires careful authentication setup  
- **Host networking** provides unrestricted network access
- **Volume mounting** gives container access to host filesystem

### Production Security Recommendations

For production deployments, consider:

1. **Remove privileged mode** if device access isn't needed
2. **Use bridge networking** with specific port mapping instead of host networking
3. **Implement proper access controls** and user management
4. **Use secrets management** for sensitive configuration
5. **Regular security updates** of base images

```yaml
# Example production security configuration
services:
  cs:
    # Remove privileged mode
    # privileged: false
    
    # Use bridge networking
    # network_mode: bridge
    
    # Specific port mapping instead of host networking
    ports:
      - "3000:3000"
      - "9090:9090"
    
    # Security options
    security_opt:
      - no-new-privileges:true
      - seccomp:unconfined
    
    # User mapping
    user: "${USER_ID}:${GROUP_ID}"
```

### Security Best Practices

- **Regular Updates**: Keep Docker, images, and dependencies updated
- **Minimal Permissions**: Use least privilege principle
- **Network Segmentation**: Isolate container networks when possible
- **Monitoring**: Implement logging and monitoring for security events

## ❓ Frequently Asked Questions (FAQ)

### General Usage

**Q: How do I know if the Control Station is running?**
```bash
make status
# or
docker compose ps
```

**Q: How do I see what's happening inside the container?**
```bash
make logs        # View logs
make attach      # Open interactive shell with ROS environment
make shell       # Open basic shell
```

**Q: How do I restart just the Control Station service?**
```bash
make restart
# or
docker compose restart cs
```

### Troubleshooting

**Q: Container fails to start with "port already in use" error?**
```bash
# Check what's using the ports
netstat -tulpn | grep -E "(3000|9090)"
# or
lsof -i :3000
lsof -i :9090

# Stop conflicting services or change ports in .env file
```

**Q: GUI applications don't appear (X11 issues)?**
```bash
# Reset X11 authentication
sudo rm -rf /tmp/.docker.xauth
make restart

# Check DISPLAY variable
echo $DISPLAY

# Test X11 forwarding (install x11-apps if needed)
make shell
xeyes  # Should open a window if X11 forwarding works
```

**Q: How do I rebuild everything from scratch?**
```bash
# Stop everything and clean up
make down
docker compose down --volumes --remove-orphans

# Rebuild from scratch
make rebuild
```

**Q: How do I access the web interface?**

Open your browser and go to:
- **Control Station UI**: http://localhost:3000
- **ROSBridge WebSocket**: ws://localhost:9090

### Development

**Q: How do I develop with live code changes?**

Source code is automatically mounted from the parent directory. Changes are reflected immediately:
```bash
# Make changes in your host editor
# Changes appear instantly in the container at /home/xplore/dev_ws/src

# If you need to rebuild ROS packages:
make attach
cd /home/xplore/dev_ws
colcon build --symlink-install
source install/setup.bash
```

**Q: How do I add additional ROS packages or dependencies?**

1. **For system packages**: Modify the `Dockerfile`
2. **For ROS packages**: Add them to your workspace and rebuild
3. **For Python packages**: Either modify `Dockerfile` or install at runtime

**Q: How do I run multiple instances?**

Modify the `.env` file to use different ports and container names:
```bash
# Copy and modify environment
cp .env .env.instance2
# Edit .env.instance2 with different ports and container name

# Start with custom environment
docker compose --env-file .env.instance2 up
```