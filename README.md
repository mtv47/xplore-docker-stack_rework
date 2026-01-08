# Docker Rework Repository

This repository contains multiple versions of Docker configurations for various ROS-based projects from the EPFL Xplore Association, along with different GitHub Actions CI/CD workflows.

## Repository Structure

### Project Versions

Each project exists in three versions, representing progressive improvements to the Docker setup:

#### **Base Version** (no suffix)
- Original configuration using standalone Docker
- Traditional bash scripts (`build.sh`, `run.sh`, `attach.sh`)
- Standard Dockerfile structure (multiple RUN commands)

#### **_new Version**
- Migrated to **Docker Compose** with enhanced configurations
- Bash scripts replaced with comprehensive **Makefile** for better task management
- Same Dockerfile layer structure as base version (not optimized)
- Makefile includes color-coded output and helpful descriptions

#### **_new_v2 Version**
- Based on `_new` version (Docker Compose + Makefile)
- **Optimized Dockerfile layers** using multi-command RUN statements (for better caching and smaller image sizes)
- Adds cache cleanup commands (`rm -rf /var/lib/apt/lists/*`, `--no-cache-dir` for pip)
- Same enhanced compose.yaml and Makefile as `_new` version
- Improved build performance and smaller image sizes

### Projects

The following projects are available in all three versions:

- **Avionics_ROS** - Avionics ROS workspace
- **docker_commons** - Common Docker configurations
- **ERC_CS_ControlStation** - Control Station for ERC Competition
- **ERC_CS_Rover** - Rover software for ERC Competition
- **ERC_HD** - HD (High Definition) subsystem
- **ERC_NAV** - Navigation subsystem
- **ERC_SC_Drill** - Science (Drill) subsystem

Each project typically contains:
- `docker_humble_desktop/` - Desktop development environment
- `docker_humble_jetson/` - Jetson deployment environment (where applicable)
- Additional specialized environments (e.g., `docker_humble_glim/` for NAV)

### GitHub Actions Workflows

Three versions of CI/CD workflows for multi-architecture Docker image builds:

#### **Github_Action**
- **Simple cross-compilation approach**
- Single build job using QEMU emulation
- Builds both AMD64 and ARM64 in one job
- Creates multi-arch image directly
- ✅ Simple setup
- ⚠️ Doesn't currently work

#### **Github_Action_arm**
- **Native architecture builds**
- Separate jobs for AMD64 and ARM64
- ARM64 job runs on GitHub-hosted `ubuntu-24.04-arm64` runners
- Third job creates multi-arch manifest
- ✅ Faster builds (no emulation)
- ⚠️ Requires GitHub ARM64 runners (additional cost)

#### **Github_Action_selfhosted**
- **Self-hosted infrastructure**
- Same structure as `Github_Action_arm`
- ARM64 job runs on self-hosted runner
- ✅ Faster builds, cost control
- ⚠️ Requires managing your own ARM64 hardware

### Workflows Available

Each GitHub Actions folder contains CI/CD workflows for:
- `erc_cs_controlstation_docker_ci.yml`
- `erc_cs_rover_docker_ci.yml`
- `erc_hd_docker_ci.yml`
- `erc_nav_docker_ci.yml`
- `erc_sc_drill_docker_ci.yml`

## Migration Path

```
Base Version          →    _new Version           →    _new_v2 Version
├─ Docker                  ├─ Docker Compose            ├─ Docker Compose
├─ Bash scripts            ├─ Makefile                  ├─ Makefile
└─ Standard Dockerfile     └─ Standard Dockerfile       └─ Optimized Dockerfile
```

## Image Size Comparison

The `size-desktop.txt` and `size-jetson.txt` files contain image size comparisons between different versions.

## Getting Started

1. Choose your project and version
2. Copy the files to the project directory that you want to set up (projects are on seperate Github repos)
3. For base version: Use `./build.sh`, `./run.sh`, `./attach.sh`
4. For _new and _new_v2: Use `make build`, `make run`, `make attach` (or see Makefile for available targets)