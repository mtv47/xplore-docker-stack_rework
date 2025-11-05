# Rover Desktop Docker Makefile
# Provides convenient commands for managing the Rover Desktop containers using Docker Compose

.PHONY: help build build-quick run run-mac run-cameras attach stop docker-check x11-setup status logs, create_volume create_network remove_volume remove_network remove_container

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
IMAGE_NAME = ghcr.io/epflxplore/rover:humble-desktop
CONTAINER_NAME = rover_humble_desktop
USERNAME = xplore
PARENT_DIR = $(abspath $(CURDIR)/..)

# Export environment variables for Docker Compose
export XAUTH
export PARENT_DIR
export DISPLAY

help: ## Show this help message
	@echo "$(BLUE)Rover Desktop Docker Commands$(NC)"
	@echo ""
	@echo "$(GREEN)Available commands:$(NC)"
	@awk 'BEGIN {FS = ":.*##"} /^[a-zA-Z_-]+:.*##/ { printf "  $(YELLOW)%-15s$(NC) %s\n", $$1, $$2 }' $(MAKEFILE_LIST)
	@echo ""
	@echo "$(BLUE)Examples:$(NC)"
	@echo "  make build       # Build the Rover image (full rebuild)"
	@echo "  make build-quick # Build the Rover image (with cache)"
	@echo "  make run         # Run interactive container"
	@echo "  make run-mac     # Run for Mac with network"
	@echo "  make run-cameras # Run camera service"
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
	@echo "$(GREEN)[SUCCESS] Done.$(NC)"
	@echo ""
	@echo "$(BLUE)[INFO] Verifying file contents:$(NC)"
	@file $(XAUTH)
	@echo "--> It should say \"X11 Xauthority data\"."
	@echo ""
	@echo "$(BLUE)[INFO] Permissions:$(NC)"
	@ls -FAlh $(XAUTH)

build: docker-check ## Build the Rover Desktop Docker image (full rebuild, no cache)
	@echo "$(BLUE)[INFO] Building Rover Desktop image (full rebuild)...$(NC)"
	docker build --no-cache --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

build-quick: docker-check ## Build the Rover Desktop Docker image (with cache for faster builds)
	@echo "$(BLUE)[INFO] Building Rover Desktop image (quick build with cache)...$(NC)"
	docker build --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

run: docker-check x11-setup ## Run interactive Rover Desktop container
	@echo "$(BLUE)[INFO] Starting Rover Desktop container...$(NC)"
	docker compose --profile default up -d rover
	@echo "$(GREEN)[SUCCESS] Rover Desktop started$(NC)"
	@echo "$(YELLOW)Use 'make attach' to connect to the container$(NC)"

run-mac: docker-check x11-setup ## Run Rover Desktop container for Mac with network
	@echo "$(BLUE)[INFO] Starting Rover Desktop container for Mac...$(NC)"
	docker compose --profile mac up -d rover-mac
	@echo "$(GREEN)[SUCCESS] Rover Desktop started in Mac mode$(NC)"
	@echo "$(YELLOW)Use 'make attach' to connect to the container$(NC)"

run-cameras: docker-check x11-setup ## Run camera service
	@echo "$(BLUE)[INFO] Starting camera service...$(NC)"
	docker compose --profile cameras up -d rover-cameras
	@echo "$(GREEN)[SUCCESS] Camera service started$(NC)"
	@echo "$(YELLOW)Use 'make attach' to connect to the container$(NC)"

attach: docker-check ## Attach to running Rover Desktop container
	@echo "$(BLUE)[INFO] Attaching to Rover Desktop container...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)❌ Rover Desktop container '$(CONTAINER_NAME)' is not running$(NC)"; \
		echo ""; \
		echo "To start the Rover Desktop, run:"; \
		echo "  make run"; \
		echo ""; \
		exit 1; \
	fi
	@echo "🔗 Attaching to $$SERVICE Desktop container..."
	@echo ""
	docker exec -it $(CONTAINER_NAME) /bin/bash -c " \
		export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
		export DISPLAY=unix\$$DISPLAY; \
		export QT_X11_NO_MITSHM=1; \
		export PYTHONPATH=/home/$(USERNAME)/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/$(USERNAME)/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
		source install/setup.bash; \
		echo '🚀 Welcome to Rover Desktop container!'; \
		echo 'Environment configured with:'; \
		echo '  - ROS 2 Humble'; \
		echo '  - CycloneDDS middleware'; \
		echo '  - X11 forwarding'; \
		echo ''; \
		/bin/bash"

stop: docker-check ## Stop the Rover Desktop container
	@echo "$(BLUE)[INFO] Stopping Rover Desktop container...$(NC)"
	docker stop $(CONTAINER_NAME) >/dev/null 2>&1 || true;
	@echo "$(GREEN)[SUCCESS] Container stopped$(NC)"

create_volume: docker-check ## Create the persistent home volume for Rover Desktop
	@echo "$(BLUE)[INFO] Creating persistent home volume for Rover Desktop...$(NC)"
	@if ! docker volume ls --format "{{.Name}}" | grep -q "^rover_humble_desktop_home_volume$$"; then \
		docker volume create rover_humble_desktop_home_volume; \
		echo "$(GREEN)[SUCCESS] Volume created: rover_humble_desktop_home_volume$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Volume already exists: rover_humble_desktop_home_volume$(NC)"; \
	fi

create_network: docker-check ## Create the frontend network for Rover Desktop
	@echo "$(BLUE)[INFO] Creating frontend network for Rover Desktop...$(NC)"
	@if ! docker network ls --format "{{.Name}}" | grep -q "^docker_humble_desktop_cs_frontend_net$$"; then \
		docker network create docker_humble_desktop_cs_frontend_net; \
		echo "$(GREEN)[SUCCESS] Network created: docker_humble_desktop_cs_frontend_net$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Network already exists: docker_humble_desktop_cs_frontend_net$(NC)"; \
	fi

remove_volume: docker-check ## Remove the persistent home volume for Rover Desktop
	@echo "$(BLUE)[INFO] Removing persistent home volume for Rover Desktop...$(NC)"
	@if docker volume ls --format "{{.Name}}" | grep -q "^rover_humble_desktop_home_volume$$"; then \
		docker volume rm rover_humble_desktop_home_volume; \
		echo "$(GREEN)[SUCCESS] Volume removed: rover_humble_desktop_home_volume$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Volume does not exist: rover_humble_desktop_home_volume$(NC)"; \
	fi

remove_network: docker-check ## Remove the frontend network for Rover Desktop
	@echo "$(BLUE)[INFO] Removing frontend network for Rover Desktop...$(NC)"
	@if docker network ls --format "{{.Name}}" | grep -q "^docker_humble_desktop_cs_frontend_net$$"; then \
		docker network rm docker_humble_desktop_cs_frontend_net; \
		echo "$(GREEN)[SUCCESS] Network removed: docker_humble_desktop_cs_frontend_net$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Network does not exist: docker_humble_desktop_cs_frontend_net$(NC)"; \
	fi

remove_container: docker-check ## Remove the Rover Desktop container
	@echo "$(BLUE)[INFO] Removing Rover Desktop container...$(NC)"
	@if docker ps -a --format "{{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker rm -f $(CONTAINER_NAME); \
		echo "$(GREEN)[SUCCESS] Container removed: $(CONTAINER_NAME)$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Container does not exist: $(CONTAINER_NAME)$(NC)"; \
	fi

status: docker-check ## Show status of Rover Desktop containers and volumes
	@echo "$(BLUE)[INFO] Rover Desktop status:$(NC)"
	@echo ""
	@echo "$(GREEN)Docker Compose Services:$(NC)"
	@docker ps -a --filter "name=$(CONTAINER_NAME)" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" || echo "No containers found"
	@echo ""
	@echo "$(GREEN)Volumes:$(NC)"
	@docker volume ls --filter "name=rover_humble_desktop" --format "table {{.Name}}\t{{.Driver}}" || echo "No volumes found"
	@echo ""
	@echo "$(GREEN)Images:$(NC)"
	@docker images $(IMAGE_NAME) --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}" || echo "No images found"

logs: docker-check ## Show logs from the Rover Desktop container
	@echo "$(BLUE)[INFO] Showing Rover Desktop container logs...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker compose logs -f; \
	else \
		echo "$(YELLOW)[INFO] No containers are running$(NC)"; \
	fi