# HD Jetson Docker Makefile
# Provides convenient commands for managing the HD (Heavy Duty) Jetson containers

.PHONY: help build run run-hd-stack run-motors attach stop stop-hd-stack clean docker-check x11-setup jetson-check

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

# Export environment variables
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
	@echo "$(BLUE)[INFO] Setting up X11 forwarding...$(NC)"
	@if [ ! -f "$(XAUTH)" ]; then \
		touch "$(XAUTH)"; \
	fi
	@if [ -n "$$DISPLAY" ]; then \
		xauth nlist "$$DISPLAY" 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f "$(XAUTH)" nmerge - 2>/dev/null || true; \
		chmod a+r "$(XAUTH)"; \
		echo "$(GREEN)[SUCCESS] X authority data configured$(NC)"; \
	else \
		echo "$(YELLOW)[WARNING] DISPLAY not set$(NC)"; \
	fi
	@echo "$(BLUE)[INFO] Verifying file contents:$(NC)"
	@file $(XAUTH) || echo "$(YELLOW)[WARNING] Could not verify X authority file$(NC)"
	@echo "$(BLUE)[INFO] Permissions:$(NC)"
	@ls -FAlh $(XAUTH)

build: docker-check ## Build the HD Jetson Docker image
	@echo "$(BLUE)[INFO] Building HD Jetson image...$(NC)"
	docker build --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

run: docker-check x11-setup ## Run interactive HD Jetson container
	@echo "$(BLUE)[INFO] Starting HD Jetson container...$(NC)"
	docker run -it \
		--name $(CONTAINER_NAME) \
		--rm \
		--runtime=nvidia \
		--gpus all \
		--privileged \
		--net=host \
		--pid=host \
		--group-add video \
		--group-add render \
		--group-add input \
		--ipc=host \
		--device /dev/bus/usb \
		-e DISPLAY=unix$$DISPLAY \
		-e QT_X11_NO_MITSHM=1 \
		-e XAUTHORITY=$(XAUTH) \
		-e CYCLONEDDS_URI="file:///home/$(USERNAME)/cyclonedds.xml" \
		-v $(XAUTH):$(XAUTH) \
		-v /dev:/dev \
		-v /run/jtop.sock:/run/jtop.sock \
		-v /run/user/1000/at-spi:/run/user/1000/at-spi \
		-v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra:ro \
		-v /usr/local/cuda:/usr/local/cuda:ro \
		-v $(PARENT_DIR):/home/$(USERNAME)/dev_ws/src \
		-v /home/xplore-hd/Documents/photos_competition:/home/$(USERNAME)/dev_ws/photos_competition \
		-v hd_humble_jetson_home_volume:/home/$(USERNAME) \
		-v $(CYCLONEDDS_CONFIG):/home/$(USERNAME)/cyclonedx.xml:ro \
		$(IMAGE_NAME) \
		/bin/bash

run-hd-stack: docker-check x11-setup ## Run HD launch stack (start or attach)
	@echo "$(BLUE)[INFO] Starting/Attaching to HD Stack...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(GREEN)Container $(CONTAINER_NAME) is already running. Attaching to it...$(NC)"; \
		docker exec -it $(CONTAINER_NAME) /bin/bash -c " \
			sudo chown -R $(USERNAME):$(USERNAME) /home/$(USERNAME); \
			export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
			export DISPLAY=unix\$$DISPLAY; \
			export QT_X11_NO_MITSHM=1; \
			export PYTHONPATH=/home/$(USERNAME)/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/$(USERNAME)/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
			source install/setup.bash; \
			ros2 launch hd_launch new.launch.py"; \
	else \
		echo "$(GREEN)Container $(CONTAINER_NAME) is not running. Starting a new container...$(NC)"; \
		docker run -i \
			--name $(CONTAINER_NAME) \
			--rm \
			--privileged \
			--net=host \
			--runtime=nvidia \
			--gpus all \
			--group-add video \
			--group-add render \
			--group-add input \
			-e DISPLAY=$$DISPLAY \
			-e QT_X11_NO_MITSHM=1 \
			-e XAUTHORITY=$(XAUTH) \
			-e CYCLONEDDS_URI="file:///home/$(USERNAME)/cyclonedx.xml" \
			-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
			-v $(XAUTH):$(XAUTH) \
			-v /run/user/1000/at-spi:/run/user/1000/at-spi \
			-v /dev:/dev \
			-v /home/xplore-hd/Documents/photos_competition:/home/$(USERNAME)/dev_ws/photos_competition \
			-v /run/jtop.sock:/run/jtop.sock \
			-v $(PARENT_DIR):/home/$(USERNAME)/dev_ws/src \
			-v hd_humble_jetson_home_volume:/home/$(USERNAME) \
			-v $(CYCLONEDDS_CONFIG):/home/$(USERNAME)/cyclonedx.xml:ro \
			$(IMAGE_NAME) \
			/bin/bash -c " \
				sudo chown -R $(USERNAME):$(USERNAME) /home/$(USERNAME); \
				export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
				export DISPLAY=unix\$$DISPLAY; \
				export QT_X11_NO_MITSHM=1; \
				export PYTHONPATH=/home/$(USERNAME)/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/$(USERNAME)/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
				source install/setup.bash; \
				ros2 launch hd_launch new.launch.py"; \
	fi

run-motors: docker-check x11-setup ## Run motor control container
	@echo "$(BLUE)[INFO] Starting HD Motors container...$(NC)"
	docker run -it \
		--name $(MOTORS_CONTAINER_NAME) \
		--rm \
		--privileged \
		--net=host \
		--user root \
		-e DISPLAY=unix$$DISPLAY \
		-e QT_X11_NO_MITSHM=1 \
		-e XAUTHORITY=$(XAUTH) \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(XAUTH):$(XAUTH) \
		-v /run/user/1000/at-spi:/run/user/1000/at-spi \
		-v /dev:/dev \
		-v $(PARENT_DIR):/home/$(USERNAME)/dev_ws/src \
		-v hd_humble_jetson_home_volume:/home/$(USERNAME) \
		$(IMAGE_NAME) \
		/bin/bash -c " \
			sudo chown -R $(USERNAME):$(USERNAME) /home/$(USERNAME); \
			export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
			export DISPLAY=unix\$$DISPLAY; \
			export QT_X11_NO_MITSHM=1; \
			export PYTHONPATH=/home/$(USERNAME)/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/$(USERNAME)/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
			source install/setup.bash; \
			ros2 run ethercat_device_configurator motor_control"

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
		source install/setup.bash 2>/dev/null || true; \
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
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker stop $(CONTAINER_NAME); \
		echo "$(GREEN)[SUCCESS] Container stopped$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Container is not running$(NC)"; \
	fi

stop-hd-stack: stop ## Alias for stop (stop HD stack container)

stop-motors: docker-check ## Stop the motors container
	@echo "$(BLUE)[INFO] Stopping HD Motors container...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(MOTORS_CONTAINER_NAME)$$"; then \
		docker stop $(MOTORS_CONTAINER_NAME); \
		echo "$(GREEN)[SUCCESS] Motors container stopped$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Motors container is not running$(NC)"; \
	fi

clean: docker-check ## Clean up containers, volumes, and images
	@echo "$(YELLOW)Cleaning up HD Jetson resources...$(NC)"
	@echo "This will remove containers, volumes, and images. Continue? [y/N]" && read ans && [ $${ans:-N} = y ]
	@if docker ps -a --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker rm -f $(CONTAINER_NAME); \
	fi
	@if docker ps -a --format "table {{.Names}}" | grep -q "^$(MOTORS_CONTAINER_NAME)$$"; then \
		docker rm -f $(MOTORS_CONTAINER_NAME); \
	fi
	@if docker volume ls -q | grep -q "hd_humble_jetson_home_volume"; then \
		docker volume rm hd_humble_jetson_home_volume; \
	fi
	@if docker images -q $(IMAGE_NAME) | head -1 | xargs -r docker rmi; then \
		echo "$(GREEN)Cleanup complete!$(NC)"; \
	fi

status: docker-check ## Show status of HD Jetson containers and volumes
	@echo "$(BLUE)[INFO] HD Jetson status:$(NC)"
	@echo ""
	@echo "$(GREEN)Containers:$(NC)"
	@docker ps -a --filter "name=hd_humble" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" || echo "No containers found"
	@echo ""
	@echo "$(GREEN)Volumes:$(NC)"
	@docker volume ls --filter "name=hd_humble_jetson" --format "table {{.Name}}\t{{.Driver}}" || echo "No volumes found"
	@echo ""
	@echo "$(GREEN)Images:$(NC)"
	@docker images $(IMAGE_NAME) --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}" || echo "No images found"