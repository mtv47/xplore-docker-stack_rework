# ERC Navigation Jetson Docker Makefile
# Provides convenient commands for managing the Navigation Jetson containers

.PHONY: help build run attach stop clean compose-check x11-setup gpu-check wheels-control

# Default target
.DEFAULT_GOAL := help

# Colors for output
BLUE = \033[0;34m
GREEN = \033[0;32m
YELLOW = \033[1;33m
RED = \033[0;31m
NC = \033[0m # No Color

# Configuration
XAUTH ?= /tmp/.docker.xauth
IMAGE_NAME = ghcr.io/epflxplore/nav:humble-jetson
CONTAINER_NAME = nav_humble_jetson
USERNAME = xplore
PARENT_DIR := $(abspath $(CURDIR)/..)
CYCLONEDDS_CONFIG = $(HOME)/Documents/ERC_NAV/docker_humble_jetson/cyclonedx.xml
PHOTOS_DIR = /home/xplore-nav/Documents/photos_competition

# Export environment variables
export XAUTH
export PARENT_DIR
export DISPLAY

help: ## Show this help message
	@echo "$(BLUE)ERC Navigation Jetson Docker Commands$(NC)"
	@echo ""
	@echo "$(GREEN)Available commands:$(NC)"
	@awk 'BEGIN {FS = ":.*##"} /^[a-zA-Z_-]+:.*##/ { printf "  $(YELLOW)%-18s$(NC) %s\n", $$1, $$2 }' $(MAKEFILE_LIST)
	@echo ""
	@echo "$(BLUE)Examples:$(NC)"
	@echo "  make build           # Build the Navigation Jetson image"
	@echo "  make run             # Run Navigation container with GPU"
	@echo "  make attach          # Attach to running container"
	@echo "  make wheels-control  # Launch wheels control stack"
	@echo "  make stop            # Stop the container"
	@echo ""
	@echo "$(YELLOW)Note:$(NC) Optimized for NVIDIA Jetson platforms with ROS2 Humble"

compose-check: ## Check if Docker Compose is available
	@if ! docker compose version > /dev/null 2>&1; then \
		echo "$(RED)[ERROR] Docker Compose is not available. Please install Docker Compose and try again.$(NC)"; \
		exit 1; \
	fi
	@if ! docker info > /dev/null 2>&1; then \
		echo "$(RED)[ERROR] Docker is not running. Please start Docker and try again.$(NC)"; \
		exit 1; \
	fi

x11-setup: ## Setup X11 forwarding for GUI applications (Jetson optimized)
	@echo "$(BLUE)[INFO] Setting up X11 forwarding for Navigation Jetson...$(NC)"
	@if [ ! -f "$(XAUTH)" ]; then \
		touch "$(XAUTH)"; \
	fi
	@echo "$(BLUE)[INFO] Populating Xauthority with SSH xauth cookie...$(NC)"
	@xauth nlist $$DISPLAY 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f "$(XAUTH)" nmerge - || true
	@chmod a+r "$(XAUTH)"
	@echo "$(BLUE)[INFO] Verifying file contents:$(NC)"
	@file $(XAUTH) || echo "$(YELLOW)[WARNING] Could not verify X authority file$(NC)"
	@echo "$(BLUE)[INFO] Permissions:$(NC)"
	@ls -FAlh $(XAUTH)

gpu-check: ## Check GPU availability and Jetson platform
	@echo "$(BLUE)[INFO] Checking Navigation Jetson hardware...$(NC)"
	@echo ""
	@echo "$(GREEN)GPU Information:$(NC)"
	@if command -v nvidia-smi > /dev/null 2>&1; then \
		nvidia-smi --query-gpu=name,memory.total,memory.used --format=csv,noheader,nounits 2>/dev/null || echo "$(YELLOW)NVIDIA GPU not available$(NC)"; \
	else \
		echo "$(YELLOW)nvidia-smi not available$(NC)"; \
	fi
	@echo ""
	@echo "$(GREEN)Jetson Platform:$(NC)"
	@if [ -f /etc/nv_tegra_release ]; then \
		cat /etc/nv_tegra_release; \
	else \
		echo "$(YELLOW)Not running on Jetson platform$(NC)"; \
	fi
	@echo ""
	@echo "$(GREEN)JTop Socket:$(NC)"
	@if [ -S /run/jtop.sock ]; then \
		echo "✅ JTop socket available: /run/jtop.sock"; \
	else \
		echo "$(YELLOW)⚠️  JTop socket not found$(NC)"; \
	fi

build: compose-check ## Build the Navigation Jetson Docker image
	@echo "$(BLUE)[INFO] Building Navigation Jetson image...$(NC)"
	docker build --pull --no-cache --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

build-quick: compose-check ## Build the Navigation Jetson Docker image (using cache)
	@echo "$(BLUE)[INFO] Building Navigation Jetson image (using cache)...$(NC)"
	docker build --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

run: compose-check x11-setup gpu-check ## Run Navigation Jetson container with GPU acceleration
	@echo "$(BLUE)[INFO] Starting Navigation Jetson container...$(NC)"
	docker compose --profile default up -d nav-jetson

attach: compose-check ## Attach to running Navigation Jetson container
	@echo "$(BLUE)[INFO] Attaching to Navigation Jetson container...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)❌ Navigation Jetson container '$(CONTAINER_NAME)' is not running$(NC)"; \
		echo ""; \
		echo "To start the Navigation Jetson container, run:"; \
		echo "  make run"; \
		echo ""; \
		exit 1; \
	fi
	@echo "🔗 Attaching to Navigation Jetson container..."
	@echo ""
	docker exec -it $(CONTAINER_NAME) /bin/bash -c " \
		source install/setup.bash; \
		echo '🚀 Welcome to Navigation Jetson container!'; \
		echo 'Environment configured with:'; \
		echo '  - ROS 2 Humble'; \
		echo '  - CycloneDDS middleware'; \
		echo '  - X11 forwarding'; \
		echo '  - GPU acceleration'; \
		echo '  - Navigation stack'; \
		echo '  - Wheels control'; \
		echo ''; \
		/bin/bash"

wheels-control: compose-check x11-setup ## Launch wheels control stack
	@echo "$(BLUE)[INFO] Launching wheels control stack...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(GREEN)[INFO] Using existing container$(NC)"; \
		docker exec -it $(CONTAINER_NAME) /bin/bash -c " \
			sudo chown -R $(USERNAME):$(USERNAME) /home/$(USERNAME); \
			source src/docker_humble_jetson/attach.sh; \
			ros2 launch wheels_control manual_stack.launch.py pub_urdf:=True"; \
	else \
		echo "$(YELLOW)[INFO] Start the container first using 'make run'$(NC)"; \
	fi

stop: compose-check ## Stop the Navigation Jetson container
	@echo "$(BLUE)[INFO] Stopping Navigation Jetson container...$(NC)"
	docker stop $(CONTAINER_NAME) >/dev/null 2>&1 || true;
	@echo "$(GREEN)[SUCCESS] Container stopped$(NC)"; 

create_volume: compose-check ## Create persistent Docker volume for Navigation Jetson
	@echo "$(BLUE)[INFO] Creating persistent volume for Navigation Jetson...$(NC)"
	@if ! docker volume ls --format "{{.Name}}" | grep -q "^nav_humble_jetson_home_volume$$"; then \
		docker volume create nav_humble_jetson_home_volume; \
		echo "$(GREEN)[SUCCESS] Volume created: nav_humble_jetson_home_volume$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Volume 'nav_humble_jetson_home_volume' already exists$(NC)"; \
	fi

remove_volume: compose-check ## Remove persistent Docker volume for Navigation Jetson
	@echo "$(BLUE)[INFO] Removing persistent volume for Navigation Jetson...$(NC)"
	@if docker volume ls --format "{{.Name}}" | grep -q "^nav_humble_jetson_home_volume$$"; then \
		docker volume rm nav_humble_jetson_home_volume; \
		echo "$(GREEN)[SUCCESS] Volume 'nav_humble_jetson_home_volume' removed$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Volume 'nav_humble_jetson_home_volume' not found$(NC)"; \
	fi
remove_container: compose-check ## Remove the Navigation Jetson container
	@echo "$(BLUE)[INFO] Removing Navigation Jetson container...$(NC)"
	@if docker ps -a --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker rm -f $(CONTAINER_NAME); \
		echo "$(GREEN)[SUCCESS] Container '$(CONTAINER_NAME)' removed$(NC)";
	else \
		echo "$(YELLOW)[INFO] Container '$(CONTAINER_NAME)' not found$(NC)"; \
	fi

status: compose-check ## Show status of Navigation Jetson containers and volumes
	@echo "$(BLUE)[INFO] Navigation Jetson status:$(NC)"
	@echo ""
	@echo "$(GREEN)Containers:$(NC)"
	@docker ps -a --filter "name=$(CONTAINER_NAME)" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" || echo "No containers found"
	@echo ""
	@echo "$(GREEN)Volumes:$(NC)"
	@docker volume ls --filter "name=nav_humble_jetson" --format "table {{.Name}}\t{{.Driver}}" || echo "No volumes found"
	@echo ""
	@echo "$(GREEN)Images:$(NC)"
	@docker images $(IMAGE_NAME) --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}" || echo "No images found"

logs: compose-check ## Show logs from the Navigation Jetson container
	@echo "$(BLUE)[INFO] Showing Navigation Jetson container logs...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker compose logs -f; \
	else \
		echo "$(YELLOW)[INFO] Container is not running$(NC)"; \
	fi