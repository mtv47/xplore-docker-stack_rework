# Avionics ROS Jetson Docker Makefile
# Provides convenient commands for managing the Avionics ROS Jetson containers

.PHONY: help build run attach stop docker-check x11-setup stop-stack launch-stack logs create_volume remove_volume status

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
IMAGE_NAME = ghcr.io/epflxplore/elec:humble-jetson
CONTAINER_NAME = elec_humble_jetson
USERNAME = xplore
PARENT_DIR = $(abspath $(CURDIR)/..)

# Export environment variables
export XAUTH
export PARENT_DIR
export DISPLAY

help: ## Show this help message
	@echo "$(BLUE)Avionics ROS Jetson Docker Commands$(NC)"
	@echo ""
	@echo "$(GREEN)Available commands:$(NC)"
	@awk 'BEGIN {FS = ":.*##"} /^[a-zA-Z_-]+:.*##/ { printf "  $(YELLOW)%-15s$(NC) %s\n", $$1, $$2 }' $(MAKEFILE_LIST)
	@echo ""
	@echo "$(BLUE)Examples:$(NC)"
	@echo "  make build            # Build the Avionics ROS Jetson image"
	@echo "  make run              # Run container (detached)"
	@echo "  make run-interactive  # Run container (interactive)"
	@echo "  make start-stack      # Start avionics stack automatically"
	@echo "  make attach           # Attach to running container"
	@echo "  make stop             # Stop the container"
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
	@echo ""

build: docker-check ## Build the Avionics ROS Jetson Docker image (verbose output)
	@echo "$(BLUE)[INFO] Building Avionics ROS Jetson image...$(NC)"
	docker build --pull --no-cache --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

build-quick: docker-check ## Build the Avionics ROS Jetson Docker image (fast, minimal output)
	@echo "$(BLUE)[INFO] Quick building Avionics ROS Jetson image...$(NC)"
	docker build --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

run: docker-check x11-setup ## Run Avionics ROS Jetson container (detached)
	@echo "$(BLUE)[INFO] Starting Avionics ROS Jetson container...$(NC)"
	docker compose up -d avionics-jetson
	@echo "$(GREEN)[SUCCESS] Container started. Use 'make attach' to connect.$(NC)"

attach: docker-check ## Attach to running Avionics ROS Jetson container
	@echo "$(BLUE)[INFO] Attaching to Avionics ROS Jetson container...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)❌ Avionics ROS Jetson container '$(CONTAINER_NAME)' is not running$(NC)"; \
		echo ""; \
		echo "To start the Avionics ROS Jetson, run:"; \
		echo "  make run          # Interactive mode"; \
		echo "  make start-stack  # Production mode with auto-launch"; \
		echo ""; \
		exit 1; \
	fi
	@echo "🔗 Attaching to Avionics ROS Jetson container..."
	@echo ""
	docker exec -it $(CONTAINER_NAME) /bin/bash -c " \
		export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
		export DISPLAY=unix$$DISPLAY; \
		export QT_X11_NO_MITSHM=1; \
		export PYTHONPATH=/home/$(USERNAME)/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/$(USERNAME)/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
		source install/setup.bash; \
		echo '🚀 Welcome to Avionics ROS Jetson container!'; \
		echo 'Environment configured with:'; \
		echo '  - ROS 2 Humble'; \
		echo '  - CycloneDDS middleware'; \
		echo '  - X11 forwarding'; \
		echo '  - Jetson hardware optimization'; \
		echo '  - Full device access'; \
		echo ''; \
		/bin/bash"

stop: docker-check ## Stop the Avionics ROS Jetson container
	@echo "$(BLUE)[INFO] Stopping Avionics ROS Jetson container...$(NC)"
	docker stop $(CONTAINER_NAME) >/dev/null 2>&1 || true;
	@echo "$(GREEN)[SUCCESS] Container stopped$(NC)";

stop-stack: stop ## Stop the Avionics ROS stack (alias for stop)

create_volume: docker-check ## Create the persistent home volume for Avionics ROS Jetson
	@echo "$(BLUE)[INFO] Creating persistent home volume for Avionics ROS Jetson...$(NC)"
	@if ! docker volume ls --format "{{.Name}}" | grep -q "^elec_humble_jetson_home_volume$$"; then \
		docker volume create elec_humble_jetson_home_volume; \
		echo "$(GREEN)[SUCCESS] Volume created: elec_humble_jetson_home_volume$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Volume already exists: elec_humble_jetson_home_volume$(NC)"; \
	fi

remove_volume: docker-check ## Remove the persistent home volume for Avionics ROS Jetson
	@echo "$(BLUE)[INFO] Removing persistent home volume for Avionics ROS Jetson...$(NC)"
	@if docker volume ls --format "{{.Name}}" | grep -q "^elec_humble_jetson_home_volume$$"; then \
		docker volume rm elec_humble_jetson_home_volume; \
		echo "$(GREEN)[SUCCESS] Volume removed: elec_humble_jetson_home_volume$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Volume does not exist: elec_humble_jetson_home_volume$(NC)"; \
	fi

status: docker-check ## Show status of Avionics ROS Jetson containers and volumes
	@echo "$(BLUE)[INFO] Avionics ROS Jetson status:$(NC)"
	@echo ""
	@echo "$(GREEN)Containers:$(NC)"
	@docker ps -a --filter "name=$(CONTAINER_NAME)" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" || echo "No containers found"
	@echo ""
	@echo "$(GREEN)Volumes:$(NC)"
	@docker volume ls --filter "name=elec_humble_jetson" --format "table {{.Name}}\t{{.Driver}}" || echo "No volumes found"
	@echo ""
	@echo "$(GREEN)Images:$(NC)"
	@docker images $(IMAGE_NAME) --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}" || echo "No images found"

launch-stack: ## Launch avionics stack in running container
	@echo "$(BLUE)[INFO] Launching avionics stack in running container...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)[ERROR] Container $(CONTAINER_NAME) is not running$(NC)"; \
		echo "Start it first with: make run or make start-stack"; \
		exit 1; \
	fi
	docker exec -it $(CONTAINER_NAME) /bin/bash -c "sudo bash -c 'cd /home/$USERNAME/dev_ws; source install/setup.sh; ros2 launch avionics_nexus launch.py'"

logs: docker-check ## Show logs from the Avionics ROS Jetson container
	@echo "$(BLUE)[INFO] Showing Avionics ROS Jetson container logs...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker compose logs -f avionics-jetson; \
	else \
		echo "$(YELLOW)[INFO] Container is not running$(NC)"; \
	fi