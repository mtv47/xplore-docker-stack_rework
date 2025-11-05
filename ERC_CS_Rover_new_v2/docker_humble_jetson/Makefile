# Rover Jetson Docker Makefile
# Provides convenient commands for managing the Rover Jetson containers using Docker Compose

.PHONY: help build build-quick run run-rover attach stop stop-rover stop-cameras clean docker-check x11-setup status logs

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
IMAGE_NAME = ghcr.io/epflxplore/rover:humble-jetson
CONTAINER_NAME = rover_humble_jetson
CAMERA_CONTAINER_NAME = rover_humble_jetson_2
USERNAME = xplore
PARENT_DIR = $(abspath $(CURDIR)/..)
JTOP_GID = $(shell getent group jtop | awk -F: '{print $$3}' 2>/dev/null || echo "Failed for JTOP_GID")

# Export environment variables for Docker Compose
export XAUTH
export PARENT_DIR
export DISPLAY
export JTOP_GID

help: ## Show this help message
	@echo "$(BLUE)Rover Jetson Docker Commands$(NC)"
	@echo ""
	@echo "$(GREEN)Available commands:$(NC)"
	@awk 'BEGIN {FS = ":.*##"} /^[a-zA-Z_-]+:.*##/ { printf "  $(YELLOW)%-15s$(NC) %s\n", $$1, $$2 }' $(MAKEFILE_LIST)
	@echo ""
	@echo "$(BLUE)Examples:$(NC)"
	@echo "  make build       # Build the Rover Jetson image (full rebuild)"
	@echo "  make build-quick # Build the Rover Jetson image (with cache)"
	@echo "  make run         # Run interactive container"
	@echo "  make run-rover   # Run rover launch script"
	@echo "  make attach      # Attach to running container"
	@echo "  make stop-rover  # Stop rover container"
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
	@echo "$(GREEN)[SUCCESS] Done.$(NC)"
	@echo ""
	@echo "$(BLUE)[INFO] Verifying file contents:$(NC)"
	@file $(XAUTH)
	@echo "--> It should say \"X11 Xauthority data\"."
	@echo ""
	@echo "$(BLUE)[INFO] Permissions:$(NC)"
	@ls -FAlh $(XAUTH)

build: docker-check ## Build the Rover Jetson Docker image (full rebuild, no cache)
	@echo "$(BLUE)[INFO] Building Rover Jetson image (full rebuild)...$(NC)"
	docker build --no-cache --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

build-quick: docker-check ## Build the Rover Jetson Docker image (with cache for faster builds)
	@echo "$(BLUE)[INFO] Building Rover Jetson image (quick build with cache)...$(NC)"
	docker build --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

run: docker-check x11-setup ## Run interactive Rover Jetson container
	@echo "$(BLUE)[INFO] Starting Rover Jetson container...$(NC)"
	docker compose --profile default up -d rover
	@echo "$(GREEN)[SUCCESS] Rover Jetson started$(NC)"
	@echo "$(YELLOW)Use 'make attach' to connect to the container$(NC)"

run-rover: docker-check x11-setup ## Run rover launch script (start or attach)
	@echo "$(BLUE)[INFO] Starting/Attaching to Rover...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(YELLOW)Container $(CONTAINER_NAME) is already running. Remove it to start a new one. $(NC)"; \
	else \
		echo "$(GREEN)Container $(CONTAINER_NAME) is not running. Starting a new container...$(NC)"; \
		docker compose --profile launch up rover-launch; \
	fi

attach: docker-check ## Attach to running Rover Jetson container
	@echo "$(BLUE)[INFO] Attaching to Rover Jetson container...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)❌ Rover Jetson container '$(CONTAINER_NAME)' is not running$(NC)"; \
		echo ""; \
		echo "To start the Rover Jetson, run:"; \
		echo "  make run"; \
		echo ""; \
		exit 1; \
	fi
	@echo "🔗 Attaching to Rover Jetson container..."
	@echo ""
	docker exec -it $(CONTAINER_NAME) /bin/bash -c " \
		export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
		export DISPLAY=unix\$$DISPLAY; \
		export QT_X11_NO_MITSHM=1; \
		export PYTHONPATH=/home/$(USERNAME)/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/$(USERNAME)/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
		source install/setup.bash; \
		echo '🚀 Welcome to Rover Jetson container!'; \
		echo 'Environment configured with:'; \
		echo '  - ROS 2 Humble'; \
		echo '  - CycloneDDS middleware'; \
		echo '  - X11 forwarding'; \
		echo '  - Jetson optimizations'; \
		echo ''; \
		/bin/bash"

stop: docker-check ## Stop the main Rover Jetson container
	@echo "$(BLUE)[INFO] Stopping Rover Jetson container...$(NC)"
	docker stop $(CONTAINER_NAME) >/dev/null 2>&1 || true;
	@echo "$(GREEN)[SUCCESS] Container stopped$(NC)"

stop-rover: stop ## Alias for stop (stop rover container)

stop-cameras: docker-check ## Stop the camera container
	@echo "$(BLUE)[INFO] Stopping camera container...$(NC)"
	docker stop $(CAMERA_CONTAINER_NAME) >/dev/null 2>&1 || true;
	@echo "$(GREEN)[SUCCESS] Container stopped$(NC)"

clean: docker-check ## Clean up containers, volumes, and images
	@echo "$(YELLOW)Cleaning up Rover Jetson resources...$(NC)"
	@echo "This will remove containers, volumes, and images. Continue? [y/N]" && read ans && [ $${ans:-N} = y ]
	@docker compose down --volumes 2>/dev/null || true
	@if docker volume ls -q | grep -q "rover_humble_jetson_home_volume"; then \
		docker volume rm rover_humble_jetson_home_volume; \
	fi
	@if docker images -q $(IMAGE_NAME) | head -1 | xargs -r docker rmi; then \
		echo "$(GREEN)Cleanup complete!$(NC)"; \
	fi

create_volume: docker-check ## Create the persistent home volume for Rover Jetson
	@echo "$(BLUE)[INFO] Creating persistent home volume for Rover Jetson...$(NC)"
	@if ! docker volume ls --format "{{.Name}}" | grep -q "^rover_humble_jetson_home_volume$$"; then \
		docker volume create rover_humble_jetson_home_volume; \
		echo "$(GREEN)[SUCCESS] Volume created: rover_humble_jetson_home_volume$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Volume already exists: rover_humble_jetson_home_volume$(NC)"; \
	fi

remove_volume: docker-check ## Remove the persistent home volume for Rover Jetson
	@echo "$(BLUE)[INFO] Removing persistent home volume for Rover Jetson...$(NC)"
	@if docker volume ls --format "{{.Name}}" | grep -q "^rover_humble_jetson_home_volume$$"; then \
		docker volume rm rover_humble_jetson_home_volume; \
		echo "$(GREEN)[SUCCESS] Volume removed: rover_humble_jetson_home_volume$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Volume does not exist: rover_humble_jetson_home_volume$(NC)"; \
	fi

status: docker-check ## Show status of Rover Jetson containers and volumes
	@echo "$(BLUE)[INFO] Rover Jetson status:$(NC)"
	@echo ""
	@echo "$(GREEN)Docker Compose Services:$(NC)"
	@docker ps -a --filter "name=$(CONTAINER_NAME)" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" || echo "No containers found"
	@echo ""
	@echo "$(GREEN)Volumes:$(NC)"
	@docker volume ls --filter "name=rover_humble_jetson" --format "table {{.Name}}\t{{.Driver}}" || echo "No volumes found"
	@echo ""
	@echo "$(GREEN)Images:$(NC)"
	@docker images $(IMAGE_NAME) --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}" || echo "No images found"

logs: docker-check ## Show logs from the Rover Jetson container
	@echo "$(BLUE)[INFO] Showing Rover Jetson container logs...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker compose logs -f; \
	else \
		echo "$(YELLOW)[INFO] No containers are running$(NC)"; \
	fi