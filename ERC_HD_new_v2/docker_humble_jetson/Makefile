# HD Jetson Docker Makefile
# Provides convenient commands for managing the HD (Heavy Duty) Jetson containers using Docker Compose

.PHONY: help build run run-hd-stack run-motors attach stop stop-hd-stack stop-motors clean docker-check x11-setup jetson-check status logs

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
IMAGE_NAME = ghcr.io/epflxplore/hd:humble-jetson-test
CONTAINER_NAME = hd_humble
MOTORS_CONTAINER_NAME = hd_humble_motors
USERNAME = xplore
PARENT_DIR = $(shell dirname $(PWD))
CYCLONEDDS_CONFIG = $(PWD)/cyclonedds.xml

# Export environment variables for Docker Compose
export XAUTH
export PARENT_DIR
export DISPLAY

help: ## Show this help message
	@echo "$(BLUE)HD Jetson Docker Commands$(NC)"
	@echo ""
	@echo "$(GREEN)Available commands:$(NC)"
	@awk 'BEGIN {FS = ":.*##"} /^[a-zA-Z_-]+:.*##/ { printf "  $(YELLOW)%-15s$(NC) %s\n", $$1, $$2 }' $(MAKEFILE_LIST)
	@echo ""
	@echo "$(BLUE)Examples:$(NC)"
	@echo "  make build       # Build the HD Jetson image"
	@echo "  make run         # Run interactive container"
	@echo "  make run-hd-stack # Run HD launch stack"
	@echo "  make run-motors  # Run motor control"
	@echo "  make attach      # Attach to running container"
	@echo ""

docker-check: ## Check if Docker is running
	@if ! docker info > /dev/null 2>&1; then \
		echo "$(RED)[ERROR] Docker is not running. Please start Docker and try again.$(NC)"; \
		exit 1; \
	fi

jetson-check: ## Check Jetson-specific requirements
	@echo "$(BLUE)[INFO] Checking Jetson environment...$(NC)"
	@if [ -d "/usr/lib/aarch64-linux-gnu/tegra" ]; then \
		echo "$(GREEN)✓ Tegra libraries detected$(NC)"; \
	else \
		echo "$(YELLOW)⚠ Tegra libraries not found (not running on Jetson)$(NC)"; \
	fi
	@if [ -d "/usr/local/cuda" ]; then \
		echo "$(GREEN)✓ CUDA installation detected$(NC)"; \
	else \
		echo "$(YELLOW)⚠ CUDA not found$(NC)"; \
	fi
	@if [ -S "/run/jtop.sock" ]; then \
		echo "$(GREEN)✓ jtop socket available$(NC)"; \
	else \
		echo "$(YELLOW)⚠ jtop socket not found$(NC)"; \
	fi
	@if docker info 2>/dev/null | grep -q nvidia; then \
		echo "$(GREEN)✓ Docker NVIDIA runtime detected$(NC)"; \
	else \
		echo "$(RED)✗ Docker NVIDIA runtime not detected$(NC)"; \
	fi

x11-setup: ## Setup X11 forwarding for GUI applications
	@echo "$(BLUE)[INFO] Preparing Xauthority data...$(NC)"
	@if [ ! -f "$(XAUTH)" ]; then \
		touch "$(XAUTH)"; \
	fi
	@xauth nlist $$DISPLAY | sed -e 's/^..../ffff/' | xauth -f "$(XAUTH)" nmerge -
	@chmod a+r "$(XAUTH)"
	@echo "$(GREEN)Done.$(NC)"
	@echo ""
	@echo "$(BLUE)Verifying file contents:$(NC)"
	@file $(XAUTH)
	@echo "$(YELLOW)--> It should say \"X11 Xauthority data\".$(NC)"
	@echo ""
	@echo "$(BLUE)Permissions:$(NC)"
	@ls -FAlh $(XAUTH)
	@echo ""

build: docker-check ## Build the HD Jetson Docker image (with cache refresh)
	@echo "$(BLUE)[INFO] Building HD Jetson image with fresh cache...$(NC)"
	docker build --pull --no-cache --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

build-quick: docker-check ## Build the HD Jetson Docker image (using cache)
	@echo "$(BLUE)[INFO] Building HD Jetson image (using cache)...$(NC)"
	docker build --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

run: docker-check x11-setup ## Run interactive HD Jetson container
	@echo "$(BLUE)[INFO] Starting HD Jetson container...$(NC)"
	docker compose --profile interactive up -d hd-jetson
	@echo "$(GREEN)[SUCCESS] HD Jetson container started$(NC)"
	@echo "$(YELLOW)Use 'make attach' to connect to the container$(NC)"

run-hd-stack: docker-check x11-setup ## Run HD launch stack
	@echo "$(BLUE)[INFO] Starting HD Stack...$(NC)"
	docker compose --profile stack up -d hd-jetson-stack
	@echo "$(GREEN)[SUCCESS] HD Stack started$(NC)"
	@echo "$(YELLOW)Use 'docker compose logs -f hd-jetson-stack' to view logs$(NC)"

run-motors: docker-check x11-setup ## Run motor control container
	@echo "$(BLUE)[INFO] Starting HD Motors container...$(NC)"
	docker compose --profile motors up -d hd-jetson-motors
	@echo "$(GREEN)[SUCCESS] HD Motors container started$(NC)"
	@echo "$(YELLOW)Use 'docker compose logs -f hd-jetson-motors' to view logs$(NC)"

attach: docker-check ## Attach to running HD Jetson container
	@echo "$(BLUE)[INFO] Attaching to HD Jetson container...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)❌ HD Jetson container '$(CONTAINER_NAME)' is not running$(NC)"; \
		echo ""; \
		echo "To start the HD Jetson, run:"; \
		echo "  make run          # Interactive container"; \
		echo "  make run-hd-stack # HD launch stack"; \
		echo ""; \
		exit 1; \
	fi
	@echo "🔗 Attaching to HD Jetson container..."
	@echo ""
	docker exec -it $(CONTAINER_NAME) /bin/bash -c " \
		export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
		export DISPLAY=unix\$$DISPLAY; \
		export QT_X11_NO_MITSHM=1; \
		export PYTHONPATH=/home/$(USERNAME)/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/$(USERNAME)/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
		source install/setup.bash; \
		echo '🚀 Welcome to HD Jetson container!'; \
		echo 'Environment configured with:'; \
		echo '  - ROS 2 Humble'; \
		echo '  - CycloneDDS middleware'; \
		echo '  - NVIDIA GPU acceleration'; \
		echo '  - Jetson hardware support'; \
		echo '  - EtherCAT capabilities'; \
		echo ''; \
		/bin/bash"

stop: docker-check ## Stop the main HD Jetson container
	@echo "$(BLUE)[INFO] Stopping HD Jetson container...$(NC)"
	docker compose down
	@echo "$(GREEN)[SUCCESS] Container(s) stopped$(NC)"

stop-hd-stack: docker-check ## Stop HD stack container
	@echo "$(BLUE)[INFO] Stopping HD Stack container...$(NC)"
	docker compose stop hd-jetson-stack
	@echo "$(GREEN)[SUCCESS] HD Stack container stopped$(NC)"

stop-motors: docker-check ## Stop the motors container
	@echo "$(BLUE)[INFO] Stopping HD Motors container...$(NC)"
	docker compose stop hd_humble
	@echo "$(GREEN)[SUCCESS] Motors container stopped$(NC)"

clean: docker-check ## Clean up containers, volumes, and images
	@echo "$(YELLOW)Cleaning up HD Jetson resources...$(NC)"
	@echo "This will remove containers, volumes, and images. Continue? [y/N]" && read ans && [ $${ans:-N} = y ]
	@docker compose down --volumes 2>/dev/null || true
	@if docker volume ls -q | grep -q "hd_humble_jetson_home_volume"; then \
		docker volume rm hd_humble_jetson_home_volume; \
	fi
	@if docker images -q $(IMAGE_NAME) | head -1 | xargs -r docker rmi; then \
		echo "$(GREEN)Cleanup complete!$(NC)"; \
	fi

status: docker-check ## Show status of HD Jetson containers and volumes
	@echo "$(BLUE)[INFO] HD Jetson status:$(NC)"
	@echo ""
	@echo "$(GREEN)Docker Compose Services:$(NC)"
	@docker compose ps 2>/dev/null || echo "No compose services found"
	@echo ""
	@echo "$(GREEN)Volumes:$(NC)"
	@docker volume ls --filter "name=hd_humble_jetson" --format "table {{.Name}}\t{{.Driver}}" || echo "No volumes found"
	@echo ""
	@echo "$(GREEN)Images:$(NC)"
	@docker images $(IMAGE_NAME) --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}" || echo "No images found"

logs: docker-check ## Show logs from HD Jetson containers
	@echo "$(BLUE)[INFO] Showing HD Jetson container logs...$(NC)"
	@echo "Available services: hd-jetson, hd-jetson-stack, hd-jetson-motors"
	@echo "Use: docker compose logs -f [service-name]"
	@docker compose logs --tail=50