# ERC Control Station Desktop Docker Makefile
# Provides convenient commands for managing the Control Station containers using Docker Compose

.PHONY: help build run run-mac attach stop clean docker-check x11-setup status logs launch-cs remove_volume remove_network create_volume create_network

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
IMAGE_NAME = ghcr.io/epflxplore/cs:humble-desktop
CONTAINER_NAME = cs_humble_desktop
USERNAME = xplore
PARENT_DIR := $(abspath $(CURDIR)/..)

# Export environment variables for Docker Compose
export XAUTH
export PARENT_DIR
export DISPLAY

help: ## Show this help message
	@echo "$(BLUE)ERC Control Station Desktop Docker Commands$(NC)"
	@echo ""
	@echo "$(GREEN)Available commands:$(NC)"
	@awk 'BEGIN {FS = ":.*##"} /^[a-zA-Z_-]+:.*##/ { printf "  $(YELLOW)%-15s$(NC) %s\n", $$1, $$2 }' $(MAKEFILE_LIST)
	@echo ""
	@echo "$(BLUE)Examples:$(NC)"
	@echo "  make build       # Build the Control Station image"
	@echo "  make run         # Run Control Station container"
	@echo "  make run-cs      # Run CS with launch script"
	@echo "  make run-mac     # Run CS using Docker Compose (macOS)"
	@echo "  make attach      # Attach to running container"
	@echo "  make stop        # Stop the container"
	@echo ""
	@echo "$(YELLOW)Note:$(NC) Control Station provides web interface on ports 3000 and 9090"

docker-check: ## Check if Docker is running
	@if ! docker info > /dev/null 2>&1; then \
		echo "$(RED)[ERROR] Docker is not running. Please start Docker and try again.$(NC)"; \
		exit 1; \
	fi

x11-setup: ## Setup X11 forwarding for GUI applications
	@echo "Preparing Xauthority data..."
	@xauth_list=$$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/'); \
	if [ ! -f "$(XAUTH)" ]; then \
		if [ ! -z "$$xauth_list" ]; then \
			echo $$xauth_list | xauth -f "$(XAUTH)" nmerge -; \
		else \
			touch "$(XAUTH)"; \
		fi; \
		chmod a+r "$(XAUTH)"; \
	fi
	@echo "Done."
	@echo ""
	@echo "Verifying file contents:"
	@file $(XAUTH)
	@echo "--> It should say \"X11 Xauthority data\"."
	@echo ""
	@echo "Permissions:"
	@ls -FAlh $(XAUTH)

build: docker-check ## Build the Control Station Docker image (with cache refresh)
	@echo "$(BLUE)[INFO] Building Control Station image with fresh cache...$(NC)"
	docker build --pull --no-cache --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

build-quick: docker-check ## Build the Control Station Docker image (using cache)
	@echo "$(BLUE)[INFO] Building Control Station image (using cache)...$(NC)"
	docker build --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

run: docker-check x11-setup ## Run Control Station container (Linux - privileged, host network)
	@echo "$(BLUE)[INFO] Starting Control Station container (Linux mode)...$(NC)"
	docker compose --profile default up -d cs
	@echo "$(GREEN)[SUCCESS] Control Station started in Linux mode$(NC)"
	@echo "$(YELLOW)Use 'make attach' to connect to the container$(NC)"

run-cs: docker-check x11-setup run launch-cs

run-mac: docker-check x11-setup ## Run Control Station container (macOS - port mapping)
	@echo "$(BLUE)[INFO] Starting Control Station container (macOS mode)...$(NC)"
	@echo "$(YELLOW)[INFO] Web interfaces will be available at:$(NC)"
	@echo "  - Main Interface: http://localhost:3000"
	@echo "  - ROSBridge: http://localhost:9090"
	@echo ""
	docker compose --profile mac up -d cs-mac
	@echo "$(GREEN)[SUCCESS] Control Station started in macOS mode$(NC)"
	@echo "$(YELLOW)Use 'make attach' to connect to the container$(NC)"

attach: docker-check ## Attach to running Control Station container
	@echo "$(BLUE)[INFO] Attaching to Control Station container...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)❌ Control Station container '$(CONTAINER_NAME)' is not running$(NC)"; \
		echo ""; \
		echo "To start the Control Station container, run one of:"; \
		echo "  make run      # Interactive mode"; \
		echo "  make run-cs   # With launch script"; \
		echo "  make run-mac  # Docker Compose mode"; \
		echo ""; \
		exit 1; \
	fi
	@echo "🔗 Attaching to Control Station container..."
	@echo ""
	docker exec -it $(CONTAINER_NAME) /bin/bash -c " \
		export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
		export DISPLAY=unix\$$DISPLAY; \
		export QT_X11_NO_MITSHM=1; \
		export PYTHONPATH=/home/xplore/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/xplore/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
		source install/setup.bash; \
		echo '🚀 Welcome to Control Station container!'; \
		echo 'Environment configured with:'; \
		echo '  - ROS 2 Humble'; \
		echo '  - CycloneDDS middleware'; \
		echo '  - X11 forwarding'; \
		echo '  - Web interfaces (ports 3000, 9090)'; \
		echo '  - Control Station stack'; \
		echo ''; \
		/bin/bash"

stop: docker-check ## Stop the Control Station container
	@echo "$(BLUE)[INFO] Stopping Control Station container...$(NC)"
	docker stop $(CONTAINER_NAME) >/dev/null 2>&1 || true;
	@echo "$(GREEN)[SUCCESS] Container stopped$(NC)"

clean: docker-check ## Clean up containers, volumes, and images
	@echo "$(YELLOW)Cleaning up Control Station resources...$(NC)"
	@echo "This will remove containers, volumes, and images. Continue? [y/N]" && read ans && [ $${ans:-N} = y ]
	@docker compose down --volumes 2>/dev/null || true
	@if docker volume ls -q | grep -q "cs_humble_desktop_home_volume"; then \
		docker volume rm cs_humble_desktop_home_volume; \
	fi
	@if docker images -q $(IMAGE_NAME) | head -1 | xargs -r docker rmi; then \
		echo "$(GREEN)Cleanup complete!$(NC)"; \
	fi

create_volume: docker-check ## Create the persistent home volume for Control Station
	@echo "$(BLUE)[INFO] Creating persistent home volume for Control Station...$(NC)"
	@if ! docker volume ls --format "{{.Name}}" | grep -q "^cs_humble_desktop_home_volume$$"; then \
		docker volume create cs_humble_desktop_home_volume; \
		echo "$(GREEN)[SUCCESS] Volume created: cs_humble_desktop_home_volume$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Volume already exists: cs_humble_desktop_home_volume$(NC)"; \
	fi

create_network: docker-check ## Create the frontend network for Control Station
	@echo "$(BLUE)[INFO] Creating frontend network for Control Station...$(NC)"
	@if ! docker network ls --format "{{.Name}}" | grep -q "^cs_frontend_net$$"; then \
		docker network create cs_frontend_net; \
		echo "$(GREEN)[SUCCESS] Network created: cs_frontend_net$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Network already exists: cs_frontend_net$(NC)"; \
	fi

remove_volume: docker-check ## Remove the persistent home volume for Control Station
	@echo "$(BLUE)[INFO] Removing persistent home volume for Control Station...$(NC)"
	@if docker volume ls --format "{{.Name}}" | grep -q "^cs_humble_desktop_home_volume$$"; then \
		docker volume rm cs_humble_desktop_home_volume && \
		echo "$(GREEN)[SUCCESS] Volume removed: cs_humble_desktop_home_volume$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Volume not found: cs_humble_desktop_home_volume$(NC)"; \
	fi

remove_network: docker-check ## Remove the frontend network for Control Station
	@echo "$(BLUE)[INFO] Removing frontend network for Control Station...$(NC)"
	@if docker network ls --format "{{.Name}}" | grep -q "^cs_frontend_net$$"; then \
		docker network rm cs_frontend_net && \
		echo "$(GREEN)[SUCCESS] Network removed: cs_frontend_net$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Network not found: cs_frontend_net$(NC)"; \
	fi

remove_container: docker-check ## Remove the Control Station container
	@echo "$(BLUE)[INFO] Removing Control Station container...$(NC)"
	@if docker ps -a --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker rm -f $(CONTAINER_NAME) && \
		echo "$(GREEN)[SUCCESS] Container removed: $(CONTAINER_NAME)$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Container not found: $(CONTAINER_NAME)$(NC)"; \
	fi

status: docker-check ## Show status of Control Station containers and volumes
	@echo "$(BLUE)[INFO] Control Station status:$(NC)"
	@echo ""
	@echo "$(GREEN)Docker Compose Services:$(NC)"
	@docker ps -a 2>/dev/null || echo "No compose services found"
	@echo ""
	@echo "$(GREEN)Volumes:$(NC)"
	@docker volume ls --filter "name=cs_humble_desktop" --format "table {{.Name}}\t{{.Driver}}" || echo "No volumes found"
	@echo ""
	@echo "$(GREEN)Images:$(NC)"
	@docker images $(IMAGE_NAME) --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}" || echo "No images found"
	@echo ""
	@echo "$(GREEN)Port Status:$(NC)"
	@echo "Checking web interface ports..."
	@if nc -z localhost 3000 2>/dev/null; then \
		echo "  ✅ Port 3000 (Main Interface) - ACTIVE"; \
	else \
		echo "  ❌ Port 3000 (Main Interface) - INACTIVE"; \
	fi
	@if nc -z localhost 9090 2>/dev/null; then \
		echo "  ✅ Port 9090 (ROSBridge) - ACTIVE"; \
	else \
		echo "  ❌ Port 9090 (ROSBridge) - INACTIVE"; \
	fi

logs: docker-check ## Show logs from the Control Station container
	@echo "$(BLUE)[INFO] Showing Control Station container logs...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker compose logs -f; \
	else \
		echo "$(YELLOW)[INFO] No containers are running$(NC)"; \
	fi

launch-cs: ## Launch Control Station stack in running container
	@echo "$(BLUE)[INFO] Launching Control Station stack...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)[ERROR] Container $(CONTAINER_NAME) is not running$(NC)"; \
		echo "Start it first with: make run or make run-mac"; \
		exit 1; \
	fi
	docker exec $(CONTAINER_NAME) /bin/bash -c " \
		cd /home/$(USERNAME)/dev_ws/src; \
		source ../install/setup.bash 2>/dev/null || true; \
		export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
		echo 'Launching Control Station...'; \
		/bin/bash ./launch.sh || echo 'Launch script not found or failed'"