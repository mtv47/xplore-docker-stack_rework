# ERC Science & Core Drill Desktop Docker Compose Makefile
# Provides convenient commands for managing the SC Drill Desktop containers

.PHONY: help build run attach stop docker-check compose-check x11-setup gpu-check, \
	status logs create_volume remove_volume

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
IMAGE_NAME = ghcr.io/epflxplore/sc:humble-desktop
CONTAINER_NAME = sc_humble_desktop
USERNAME = xplore
PARENT_DIR := $(abspath $(CURDIR)/..)
COMPOSE_FILE = compose.yaml

# Export environment variables
export XAUTH
export PARENT_DIR
export DISPLAY

help: ## Show this help message
	@echo "$(BLUE)ERC Science & Core Drill Desktop Docker Compose Commands$(NC)"
	@echo ""
	@echo "$(GREEN)Available commands:$(NC)"
	@awk 'BEGIN {FS = ":.*##"} /^[a-zA-Z_-]+:.*##/ { printf "  $(YELLOW)%-15s$(NC) %s\n", $$1, $$2 }' $(MAKEFILE_LIST)
	@echo ""
	@echo "$(BLUE)Examples:$(NC)"
	@echo "  make build       # Build the SC Drill Desktop image"
	@echo "  make run         # Run interactive container"
	@echo "  make run-gpu     # Run with GPU acceleration"
	@echo "  make attach      # Attach to running container"
	@echo "  make stop        # Stop the container"
	@echo ""

docker-check: ## Check if Docker is running
	@if ! docker info > /dev/null 2>&1; then \
		echo "$(RED)[ERROR] Docker is not running. Please start Docker and try again.$(NC)"; \
		exit 1; \
	fi

compose-check: docker-check ## Check if Docker Compose is available
	@if ! docker compose version > /dev/null 2>&1; then \
		echo "$(RED)[ERROR] Docker Compose is not available. Please install Docker Compose and try again.$(NC)"; \
		exit 1; \
	fi

x11-setup: ## Setup X11 forwarding for GUI applications
	@echo "$(BLUE)[INFO] Preparing Xauthority data...$(NC)"
	@xauth_list=$$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/'); \
	if [ ! -f "$(XAUTH)" ]; then \
		if [ ! -z "$$xauth_list" ]; then \
			echo "$$xauth_list" | xauth -f "$(XAUTH)" nmerge -; \
			echo "$(GREEN)[SUCCESS] X authority data merged successfully$(NC)"; \
		else \
			touch "$(XAUTH)"; \
			echo "$(YELLOW)[WARNING] Created empty X authority file (no X session found)$(NC)"; \
		fi; \
		chmod a+r "$(XAUTH)"; \
	fi
	@echo "$(GREEN)[INFO] Done.$(NC)"
	@echo ""
	@echo "$(BLUE)[INFO] Verifying file contents:$(NC)"
	@file $(XAUTH) || echo "$(YELLOW)[WARNING] Could not verify X authority file$(NC)"
	@echo "--> It should say \"X11 Xauthority data\"."
	@echo ""
	@echo "$(BLUE)[INFO] Permissions:$(NC)"
	@ls -FAlh $(XAUTH)

gpu-check: ## Check GPU availability for potential GPU acceleration
	@echo "$(BLUE)[INFO] Checking GPU availability...$(NC)"
	@if command -v nvidia-smi > /dev/null 2>&1; then \
		echo "$(GREEN)[INFO] NVIDIA GPU detected:$(NC)"; \
		nvidia-smi --query-gpu=name,memory.total,memory.used --format=csv,noheader,nounits 2>/dev/null || echo "$(YELLOW)[WARNING] Could not query GPU details$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] No NVIDIA GPU detected$(NC)"; \
	fi

build: compose-check ## Build the SC Drill Desktop Docker image (with cache refresh)
	@echo "$(BLUE)[INFO] Building SC Drill Desktop image with fresh cache...$(NC)"
	docker build --pull --no-cache --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

build-quick: compose-check ## Build the SC Drill Desktop Docker image (using cache)
	@echo "$(BLUE)[INFO] Building SC Drill Desktop image (using cache)...$(NC)"
	docker build --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

run: compose-check x11-setup ## Run SC Drill Desktop container
	@echo "$(BLUE)[INFO] Starting SC Drill Desktop container...$(NC)"
	docker compose --profile default up -d sc-drill

run-gpu: compose-check x11-setup gpu-check ## Run SC Drill Desktop container with GPU acceleration
	@echo "$(BLUE)[INFO] Starting SC Drill Desktop container with GPU support...$(NC)"
	docker compose --profile gpu up -d sc-drill-gpu

attach: compose-check ## Attach to running SC Drill Desktop container
	@echo "$(BLUE)[INFO] Attaching to SC Drill Desktop container...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)❌ SC Drill Desktop container '$(CONTAINER_NAME)' is not running$(NC)"; \
		echo ""; \
		echo "To start the SC Drill Desktop, run:"; \
		echo "  make run      # Standard mode"; \
		echo "  make run-gpu  # With GPU acceleration"; \
		echo ""; \
		exit 1; \
	fi
	@echo "🔗 Attaching to SC Drill Desktop container..."
	@echo ""
	docker exec -it $(CONTAINER_NAME) /bin/bash -c " \
		export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
		export DISPLAY=unix\$$DISPLAY; \
		export QT_X11_NO_MITSHM=1; \
		export PYTHONPATH=/home/$(USERNAME)/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/$(USERNAME)/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
		source install/setup.bash; \
		echo '🚀 Welcome to SC Drill Desktop container!'; \
		echo 'Environment configured with:'; \
		echo '  - ROS 2 Humble'; \
		echo '  - CycloneDX middleware'; \
		echo '  - X11 forwarding'; \
		echo '  - Science & Core subsystem packages'; \
		echo '  - Full device access'; \
		echo ''; \
		/bin/bash"

stop: compose-check ## Stop the SC Drill Desktop container
	@echo "$(BLUE)[INFO] Stopping SC Drill Desktop container...$(NC)"
	docker stop $(CONTAINER_NAME) >/dev/null 2>&1 || true;
	@echo "$(GREEN)[SUCCESS] Container stopped$(NC)";

create_volume: compose-check ## Create the persistent home volume for SC Drill Desktop
	@echo "$(BLUE)[INFO] Creating persistent home volume for SC Drill Desktop...$(NC)"
	@if ! docker volume ls --format "{{.Name}}" | grep -q "^sc_humble_desktop_home_volume$$"; then \
		docker volume create sc_humble_desktop_home_volume; \
		echo "$(GREEN)[SUCCESS] Volume created: sc_humble_desktop_home_volume$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Volume already exists: sc_humble_desktop_home_volume$(NC)"; \
	fi

remove_volume: compose-check ## Remove the persistent home volume for SC Drill Desktop
	@echo "$(BLUE)[INFO] Removing persistent home volume for SC Drill Desktop...$(NC)"
	@if docker volume ls --format "{{.Name}}" | grep -q "^sc_humble_desktop_home_volume$$"; then \
		docker volume rm sc_humble_desktop_home_volume; \
		echo "$(GREEN)[SUCCESS] Volume removed: sc_humble_desktop_home_volume$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Volume does not exist: sc_humble_desktop_home_volume$(NC)"; \
	fi

remove_container: compose-check ## Remove the SC Drill container
	@echo "$(BLUE)[INFO] Removing SC Drill Desktop container...$(NC)"
	@if docker ps -a --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker rm -f $(CONTAINER_NAME); \
		echo "$(GREEN)[SUCCESS] Container removed: $(CONTAINER_NAME)$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Container does not exist: $(CONTAINER_NAME)$(NC)"; \
	fi

status: compose-check ## Show status of SC Drill Desktop containers and volumes
	@echo "$(BLUE)[INFO] SC Drill Desktop status:$(NC)"
	@echo ""
	@echo "$(GREEN)Containers:$(NC)"
	@docker compose -f $(COMPOSE_FILE) ps -a 2>/dev/null || echo "No containers found"
	@echo ""
	@echo "$(GREEN)Volumes:$(NC)"
	@docker volume ls --filter "name=sc_humble_desktop" --format "table {{.Name}}\t{{.Driver}}" || echo "No volumes found"
	@echo ""
	@echo "$(GREEN)Images:$(NC)"
	@docker images $(IMAGE_NAME) --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}" || echo "No images found"

logs: compose-check ## Show logs from the SC Drill Desktop container
	@echo "$(BLUE)[INFO] Showing SC Drill Desktop container logs...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker compose -f $(COMPOSE_FILE) logs -f sc-drill; \
	else \
		echo "$(YELLOW)[INFO] Container is not running$(NC)"; \
	fi