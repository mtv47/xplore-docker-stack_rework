# ERC Science & Core Drill Jetson Docker Compose Makefile
# Provides convenient commands for managing the SC Drill Jetson containers

.PHONY: help build build-quick run run-gpu attach stop clean docker-check compose-check x11-setup gpu-check run-drill

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
IMAGE_NAME = ghcr.io/epflxplore/sc:humble-jetson
CONTAINER_NAME = sc_humble_jetson
USERNAME = xplore
PARENT_DIR = $(shell dirname $(PWD))
PHOTOS_DIR = /home/xplore-hd/Documents/photos_competition
COMPOSE_FILE = compose.yaml

# Export environment variables
export XAUTH
export PARENT_DIR
export DISPLAY
export PHOTOS_DIR

help: ## Show this help message
	@echo "$(BLUE)ERC Science & Core Drill Jetson Docker Compose Commands$(NC)"
	@echo ""
	@echo "$(GREEN)Available commands:$(NC)"
	@awk 'BEGIN {FS = ":.*##"} /^[a-zA-Z_-]+:.*##/ { printf "  $(YELLOW)%-15s$(NC) %s\n", $$1, $$2 }' $(MAKEFILE_LIST)
	@echo ""
	@echo "$(BLUE)Examples:$(NC)"
	@echo "  make build        # Build the SC Drill Jetson image"
	@echo "  make run          # Run interactive container"
	@echo "  make run-drill    # Start drill stack automatically"
	@echo "  make attach       # Attach to running container"
	@echo "  make stop         # Stop the container"
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

build: compose-check ## Build the SC Drill Jetson Docker image (with cache refresh)
	@echo "$(BLUE)[INFO] Building SC Drill Jetson image with fresh cache...$(NC)"
	docker compose -f $(COMPOSE_FILE) build --pull --no-cache sc-drill

build-quick: compose-check ## Build the SC Drill Jetson Docker image (using cache)
	@echo "$(BLUE)[INFO] Building SC Drill Jetson image (using cache)...$(NC)"
	docker compose -f $(COMPOSE_FILE) build sc-drill

run: compose-check x11-setup ## Run SC Drill Jetson container (interactive)
	@echo "$(BLUE)[INFO] Starting SC Drill Jetson container...$(NC)"
	docker compose -f $(COMPOSE_FILE) --profile default up -it sc-drill

run-gpu: compose-check x11-setup gpu-check ## Run SC Drill Jetson container with GPU acceleration
	@echo "$(BLUE)[INFO] Starting SC Drill Jetson container with GPU support...$(NC)"
	docker compose -f $(COMPOSE_FILE) --profile gpu up -it sc-drill-gpu

run-drill: compose-check x11-setup ## Start SC Drill stack automatically (production mode)
	@echo "$(BLUE)[INFO] Starting SC Drill Jetson stack...$(NC)"
	@if [ "$$(docker inspect -f '{{.State.Running}}' $(CONTAINER_NAME) 2>/dev/null)" = "true" ]; then \
		echo "$(YELLOW)[INFO] Container $(CONTAINER_NAME) is already running. Launching drill stack...$(NC)"; \
		docker exec -it $(CONTAINER_NAME) /bin/bash -c " \
			sudo chown -R $(USERNAME):$(USERNAME) /home/$(USERNAME); \
			export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
			export DISPLAY=unix\$$DISPLAY; \
			export QT_X11_NO_MITSHM=1; \
			export PYTHONPATH=/home/$(USERNAME)/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/$(USERNAME)/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
			source install/setup.bash; \
			ros2 launch erc_drill drill.launch.xml"; \
	else \
		echo "$(BLUE)[INFO] Starting new container with drill stack...$(NC)"; \
		docker compose -f $(COMPOSE_FILE) --profile production up sc-drill-production; \
	fi

attach: compose-check ## Attach to running SC Drill Jetson container
	@echo "$(BLUE)[INFO] Attaching to SC Drill Jetson container...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)❌ SC Drill Jetson container '$(CONTAINER_NAME)' is not running$(NC)"; \
		echo ""; \
		echo "To start the SC Drill Jetson, run:"; \
		echo "  make run          # Interactive mode"; \
		echo "  make run-gpu      # With GPU acceleration"; \
		echo "  make run-drill    # Production mode with auto-launch"; \
		echo ""; \
		exit 1; \
	fi
	@echo "🔗 Attaching to SC Drill Jetson container..."
	@echo ""
	docker exec -it $(CONTAINER_NAME) /bin/bash -c " \
		export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
		export DISPLAY=unix\$$DISPLAY; \
		export QT_X11_NO_MITSHM=1; \
		export PYTHONPATH=/home/$(USERNAME)/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/$(USERNAME)/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
		source install/setup.bash; \
		echo '🚀 Welcome to SC Drill Jetson container!'; \
		echo 'Environment configured with:'; \
		echo '  - ROS 2 Humble'; \
		echo '  - CycloneDDS middleware'; \
		echo '  - X11 forwarding'; \
		echo '  - Jetson hardware optimization'; \
		echo '  - Drill hardware access'; \
		echo '  - Photos competition data'; \
		echo ''; \
		/bin/bash"

stop: compose-check ## Stop the SC Drill Jetson container
	@echo "$(BLUE)[INFO] Stopping SC Drill Jetson container...$(NC)"
	@if [ "$$(docker inspect -f '{{.State.Running}}' $(CONTAINER_NAME) 2>/dev/null)" = "true" ]; then \
		docker compose -f $(COMPOSE_FILE) down; \
		echo "$(GREEN)[SUCCESS] Container stopped$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Container is not running$(NC)"; \
	fi

clean: compose-check ## Clean up containers, volumes, and images
	@echo "$(YELLOW)Cleaning up SC Drill Jetson resources...$(NC)"
	@echo "This will remove containers, volumes, and images. Continue? [y/N]" && read ans && [ $${ans:-N} = y ]
	@docker compose -f $(COMPOSE_FILE) down -v --rmi all 2>/dev/null || true
	@if docker volume ls -q | grep -q "sc_humble_jetson_home_volume"; then \
		docker volume rm sc_humble_jetson_home_volume; \
	fi
	@echo "$(GREEN)Cleanup complete!$(NC)"

status: compose-check ## Show status of SC Drill Jetson containers and volumes
	@echo "$(BLUE)[INFO] SC Drill Jetson status:$(NC)"
	@echo ""
	@echo "$(GREEN)Containers:$(NC)"
	@docker compose -f $(COMPOSE_FILE) ps -a 2>/dev/null || echo "No containers found"
	@echo ""
	@echo "$(GREEN)Volumes:$(NC)"
	@docker volume ls --filter "name=sc_humble_jetson" --format "table {{.Name}}\t{{.Driver}}" || echo "No volumes found"
	@echo ""
	@echo "$(GREEN)Images:$(NC)"
	@docker images $(IMAGE_NAME) --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}" || echo "No images found"

logs: compose-check ## Show logs from the SC Drill Jetson container
	@echo "$(BLUE)[INFO] Showing SC Drill Jetson container logs...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker compose -f $(COMPOSE_FILE) logs -f sc-drill; \
	else \
		echo "$(YELLOW)[INFO] Container is not running$(NC)"; \
	fi