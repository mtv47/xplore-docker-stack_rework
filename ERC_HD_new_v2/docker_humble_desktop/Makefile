# HD Desktop Docker Makefile
# Provides convenient commands for managing the HD (Heavy Duty) Desktop containers using Docker Compose

.PHONY: help build run run-no-gpu attach stop docker-check x11-setup status logs gpu-check, create_volume remove_volume remove_container

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
IMAGE_NAME = ghcr.io/epflxplore/hd:humble-desktop
CONTAINER_NAME = hd_humble_desktop
USERNAME = xplore
PARENT_DIR := $(abspath $(CURDIR)/..)

# Export environment variables for Docker Compose
export XAUTH
export PARENT_DIR
export DISPLAY

help: ## Show this help message
	@echo "$(BLUE)HD Desktop Docker Commands$(NC)"
	@echo ""
	@echo "$(GREEN)Available commands:$(NC)"
	@awk 'BEGIN {FS = ":.*##"} /^[a-zA-Z_-]+:.*##/ { printf "  $(YELLOW)%-15s$(NC) %s\n", $$1, $$2 }' $(MAKEFILE_LIST)
	@echo ""
	@echo "$(BLUE)Examples:$(NC)"
	@echo "  make run         # Run with GPU support"
	@echo "  make run-no-gpu  # Run without GPU support"
	@echo "  make attach      # Attach to running container"
	@echo "  make stop        # Stop the container"
	@echo ""

docker-check: ## Check if Docker is running
	@if ! docker info > /dev/null 2>&1; then \
		echo "$(RED)[ERROR] Docker is not running. Please start Docker and try again.$(NC)"; \
		exit 1; \
	fi

x11-setup: ## Setup X11 forwarding for GUI applications
	@echo "$(BLUE)[INFO] Preparing Xauthority data...$(NC)"
	@xauth_list=$$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/'); \
	if [ ! -f "$(XAUTH)" ]; then \
		if [ ! -z "$$xauth_list" ]; then \
			echo "$$xauth_list" | xauth -f "$(XAUTH)" nmerge -; \
		else \
			touch "$(XAUTH)"; \
		fi; \
		chmod a+r "$(XAUTH)"; \
	fi
	@echo "$(GREEN)Done.$(NC)"
	@echo ""
	@echo "$(BLUE)Verifying file contents:$(NC)"
	@file $(XAUTH)
	@echo "$(YELLOW)--> It should say \"X11 Xauthority data\".$(NC)"
	@echo ""
	@echo "$(BLUE)Permissions:$(NC)"
	@ls -FAlh $(XAUTH)
	@echo ""

build: docker-check ## Build the HD Desktop Docker image (with cache refresh)
	@echo "$(BLUE)[INFO] Building HD Desktop image with fresh cache...$(NC)"
	docker build --pull --no-cache --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

build-quick: docker-check ## Build the HD Desktop Docker image (using cache)
	@echo "$(BLUE)[INFO] Building HD Desktop image (using cache)...$(NC)"
	docker build --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

run: docker-check x11-setup ## Run HD Desktop container with GPU support
	@echo "$(BLUE)[INFO] Starting HD Desktop container with GPU support...$(NC)"
	docker compose --profile gpu up -d hd-gpu
	@echo "$(GREEN)[SUCCESS] HD Desktop started with GPU support$(NC)"
	@echo "$(YELLOW)Use 'make attach' to connect to the container$(NC)"

run-no-gpu: docker-check x11-setup ## Run HD Desktop container without GPU support
	@echo "$(BLUE)[INFO] Starting HD Desktop container without GPU support...$(NC)"
	docker compose --profile no-gpu up -d hd
	@echo "$(GREEN)[SUCCESS] HD Desktop started without GPU support$(NC)"
	@echo "$(YELLOW)Use 'make attach' to connect to the container$(NC)"

attach: docker-check ## Attach to running HD Desktop container
	@echo "$(BLUE)[INFO] Attaching to HD Desktop container...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)❌ HD Desktop container '$(CONTAINER_NAME)' is not running$(NC)"; \
		echo ""; \
		echo "To start the HD Desktop, run:"; \
		echo "  make run        # With GPU support"; \
		echo "  make run-no-gpu # Without GPU support"; \
		echo ""; \
		exit 1; \
	fi
	@echo "🔗 Attaching to HD Desktop container..."
	@echo ""
	docker exec -it $(CONTAINER_NAME) /bin/bash -c " \
		export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
		export DISPLAY=unix$$DISPLAY; \
		export QT_X11_NO_MITSHM=1; \
		export PYTHONPATH=/home/$(USERNAME)/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/$(USERNAME)/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
		source install/setup.bash; \
		echo '🚀 Welcome to HD Desktop container!'; \
		echo 'Environment configured with:'; \
		echo '  - ROS 2 Humble'; \
		echo '  - CycloneDDS middleware'; \
		echo '  - X11 forwarding'; \
		echo '  - Heavy Duty hardware support'; \
		echo ''; \
		/bin/bash"

stop: docker-check ## Stop the HD Desktop container
	@echo "$(BLUE)[INFO] Stopping HD Desktop container...$(NC)"
	docker stop $(CONTAINER_NAME) >/dev/null 2>&1 || true;
	@echo "$(GREEN)[SUCCESS] Container stopped$(NC)"

create_volume: docker-check ## Create the persistent home volume for HD Desktop
	@echo "$(BLUE)[INFO] Creating persistent home volume for HD Desktop...$(NC)"
	@if ! docker volume ls --format "{{.Name}}" | grep -q "^hd_humble_desktop_home_volume$$"; then \
		docker volume create hd_humble_desktop_home_volume; \
		echo "$(GREEN)[SUCCESS] Volume created: hd_humble_desktop_home_volume$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Volume already exists: hd_humble_desktop_home_volume$(NC)"; \
	fi

remove_volume: docker-check ## Remove the persistent home volume for HD Desktop
	@echo "$(BLUE)[INFO] Removing persistent home volume for HD Desktop...$(NC)"
	@if docker volume ls --format "{{.Name}}" | grep -q "^hd_humble_desktop_home_volume$$"; then \
		docker volume rm hd_humble_desktop_home_volume; \
		echo "$(GREEN)[SUCCESS] Volume removed: hd_humble_desktop_home_volume$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Volume does not exist: hd_humble_desktop_home_volume$(NC)"; \
	fi

remove_container: docker-check ## Remove the HD Desktop container
	@echo "$(BLUE)[INFO] Removing HD Desktop container...$(NC)"
	@if docker ps -a --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker rm -f $(CONTAINER_NAME); \
		echo "$(GREEN)[SUCCESS] Container removed: $(CONTAINER_NAME)$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Container does not exist: $(CONTAINER_NAME)$(NC)"; \
	fi

status: docker-check ## Show status of HD Desktop containers and volumes
	@echo "$(BLUE)[INFO] HD Desktop status:$(NC)"
	@echo ""
	@echo "$(GREEN)Docker Compose Services:$(NC)"
	@docker ps -a --filter "name=$(CONTAINER_NAME)" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" || echo "No containers found"
	@echo ""
	@echo "$(GREEN)Volumes:$(NC)"
	@docker volume ls --filter "name=hd_humble_desktop" --format "table {{.Name}}\t{{.Driver}}" || echo "No volumes found"
	@echo ""
	@echo "$(GREEN)Images:$(NC)"
	@docker images $(IMAGE_NAME) --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}" || echo "No images found"

logs: docker-check ## Show logs from the HD Desktop container
	@echo "$(BLUE)[INFO] Showing HD Desktop container logs...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker compose logs -f; \
	else \
		echo "$(YELLOW)[INFO] No containers are running$(NC)"; \
	fi

gpu-check: ## Check if NVIDIA GPU support is available
	@echo "$(BLUE)[INFO] Checking GPU support...$(NC)"
	@if command -v nvidia-smi > /dev/null 2>&1; then \
		echo "$(GREEN)✓ NVIDIA drivers detected$(NC)"; \
		nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader,nounits; \
	else \
		echo "$(YELLOW)⚠ NVIDIA drivers not found$(NC)"; \
	fi
	@if docker info 2>/dev/null | grep -q nvidia; then \
		echo "$(GREEN)✓ Docker NVIDIA runtime detected$(NC)"; \
	else \
		echo "$(YELLOW)⚠ Docker NVIDIA runtime not detected$(NC)"; \
		echo "$(YELLOW)  Use 'make run-no-gpu' for systems without GPU support$(NC)"; \
	fi