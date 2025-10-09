# ERC Navigation Jetson Docker Makefile
# Provides convenient commands for managing the Navigation Jetson containers

.PHONY: help build run attach stop clean docker-check x11-setup gpu-check wheels-control

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
PARENT_DIR = $(shell dirname $(PWD))
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

docker-check: ## Check if Docker is running
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

build: docker-check ## Build the Navigation Jetson Docker image
	@echo "$(BLUE)[INFO] Building Navigation Jetson image...$(NC)"
	docker build --progress=plain --network=host -t $(IMAGE_NAME) -f Dockerfile ..

build-quick: docker-check ## Build the Navigation Jetson Docker image (using cache)
	@echo "$(BLUE)[INFO] Building Navigation Jetson image (using cache)...$(NC)"
	docker build --network=host -t $(IMAGE_NAME) -f Dockerfile ..

run: docker-check x11-setup gpu-check ## Run Navigation Jetson container with GPU acceleration
	@echo "$(BLUE)[INFO] Starting Navigation Jetson container...$(NC)"
	docker run -it \
		--name $(CONTAINER_NAME) \
		--rm \
		--privileged \
		--net=host \
		--runtime=nvidia \
		--gpus all \
		-e DISPLAY=$$DISPLAY \
		-e QT_X11_NO_MITSHM=1 \
		-e XAUTHORITY=$(XAUTH) \
		-e GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11 \
		-e GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models \
		-e GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins \
		-e CYCLONEDDS_URI="file:///home/$(USERNAME)/cyclonedx.xml" \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(XAUTH):$(XAUTH) \
		-v /run/user/1000/at-spi:/run/user/1000/at-spi \
		-v /dev:/dev \
		-v /run/jtop.sock:/run/jtop.sock \
		-v $(PARENT_DIR):/home/$(USERNAME)/dev_ws/src \
		-v nav_humble_jetson_home_volume:/home/$(USERNAME) \
		-v $(PWD)/cyclonedx.xml:/home/$(USERNAME)/cyclonedx.xml:ro \
		-v $(PHOTOS_DIR):/home/$(USERNAME)/dev_ws/photos_competition \
		--add-host=os-122140001125.local:169.254.55.220 \
		$(IMAGE_NAME) \
		/bin/bash -c "sudo chown -R $(USERNAME):$(USERNAME) /home/$(USERNAME); \
			export PYTHONPATH=/home/xplore/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/xplore/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
			echo '🚀 Navigation Jetson container ready!'; \
			echo 'Features enabled:'; \
			echo '  - NVIDIA GPU acceleration'; \
			echo '  - Jetson platform optimization'; \
			echo '  - CycloneDDS middleware'; \
			echo '  - Gazebo simulation'; \
			echo '  - Wheels control system'; \
			echo ''; \
			/bin/bash"

attach: docker-check ## Attach to running Navigation Jetson container
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
		export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
		export DISPLAY=unix\$$DISPLAY; \
		export QT_X11_NO_MITSHM=1; \
		export PYTHONPATH=/home/xplore/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/xplore/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
		source install/setup.bash 2>/dev/null || true; \
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

wheels-control: docker-check x11-setup ## Launch wheels control stack (creates new container if needed)
	@echo "$(BLUE)[INFO] Launching wheels control stack...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(GREEN)[INFO] Using existing container$(NC)"; \
		docker exec -it $(CONTAINER_NAME) /bin/bash -c " \
			sudo chown -R $(USERNAME):$(USERNAME) /home/$(USERNAME); \
			source src/docker_humble_jetson/attach.sh 2>/dev/null || true; \
			export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
			export PYTHONPATH=/home/xplore/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/xplore/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
			source install/setup.bash 2>/dev/null || true; \
			echo '🎮 Launching wheels control stack...'; \
			ros2 launch wheels_control manual_stack.launch.py pub_urdf:=True"; \
	else \
		echo "$(YELLOW)[INFO] Starting new container for wheels control$(NC)"; \
		$(MAKE) x11-setup; \
		docker run -it \
			--name $(CONTAINER_NAME)_wheels \
			--rm \
			--privileged \
			--net=host \
			--runtime=nvidia \
			--gpus all \
			-e DISPLAY=$$DISPLAY \
			-e QT_X11_NO_MITSHM=1 \
			-e XAUTHORITY=$(XAUTH) \
			-e GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11 \
			-e GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models \
			-e GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins \
			-e CYCLONEDDS_URI="file:///home/$(USERNAME)/cyclonedx.xml" \
			-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
			-v $(XAUTH):$(XAUTH) \
			-v /run/user/1000/at-spi:/run/user/1000/at-spi \
			-v /dev:/dev \
			-v /run/jtop.sock:/run/jtop.sock \
			-v $(PARENT_DIR):/home/$(USERNAME)/dev_ws/src \
			-v nav_humble_jetson_home_volume:/home/$(USERNAME) \
			-v $(PWD)/cyclonedx.xml:/home/$(USERNAME)/cyclonedx.xml:ro \
			-v $(PHOTOS_DIR):/home/$(USERNAME)/dev_ws/photos_competition \
			--add-host=os-122140001125.local:169.254.55.220 \
			$(IMAGE_NAME) \
			/bin/bash -c "sudo chown -R $(USERNAME):$(USERNAME) /home/$(USERNAME); \
				export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
				export PYTHONPATH=/home/xplore/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/xplore/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
				source install/setup.bash 2>/dev/null || true; \
				echo '🎮 Launching wheels control stack...'; \
				ros2 launch wheels_control manual_stack.launch.py pub_urdf:=True"; \
	fi

stop: docker-check ## Stop the Navigation Jetson container
	@echo "$(BLUE)[INFO] Stopping Navigation Jetson containers...$(NC)"
	@containers_stopped=0; \
	for container in $(CONTAINER_NAME) $(CONTAINER_NAME)_wheels; do \
		if docker ps --format "table {{.Names}}" | grep -q "^$$container$$"; then \
			docker stop $$container; \
			containers_stopped=1; \
		fi; \
	done; \
	if [ $$containers_stopped -eq 1 ]; then \
		echo "$(GREEN)[SUCCESS] Containers stopped$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] No containers were running$(NC)"; \
	fi

clean: docker-check ## Clean up containers, volumes, and images
	@echo "$(YELLOW)Cleaning up Navigation Jetson resources...$(NC)"
	@echo "This will remove containers, volumes, and images. Continue? [y/N]" && read ans && [ $${ans:-N} = y ]
	@containers_removed=0; \
	for container in $(CONTAINER_NAME) $(CONTAINER_NAME)_wheels; do \
		if docker ps -a --format "table {{.Names}}" | grep -q "^$$container$$"; then \
			docker rm -f $$container; \
			containers_removed=1; \
		fi; \
	done
	@if docker volume ls -q | grep -q "nav_humble_jetson_home_volume"; then \
		docker volume rm nav_humble_jetson_home_volume; \
	fi
	@if docker images -q $(IMAGE_NAME) | head -1 | xargs -r docker rmi; then \
		echo "$(GREEN)Cleanup complete!$(NC)"; \
	fi

status: docker-check ## Show status of Navigation Jetson containers and volumes
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

hardware-check: ## Check Jetson hardware and Navigation requirements
	@echo "$(BLUE)[INFO] Checking Navigation Jetson hardware requirements...$(NC)"
	@echo ""
	@echo "$(GREEN)Jetson Platform:$(NC)"
	@if [ -f /etc/nv_tegra_release ]; then \
		cat /etc/nv_tegra_release; \
	else \
		echo "$(YELLOW)Not running on Jetson platform$(NC)"; \
	fi
	@echo ""
	@echo "$(GREEN)GPU Information:$(NC)"
	@if command -v nvidia-smi > /dev/null 2>&1; then \
		nvidia-smi --query-gpu=name,memory.total,memory.used --format=csv,noheader,nounits 2>/dev/null || echo "$(YELLOW)NVIDIA GPU not available$(NC)"; \
	else \
		echo "$(YELLOW)nvidia-smi not available$(NC)"; \
	fi
	@echo ""
	@echo "$(GREEN)Jetson Tools:$(NC)"
	@echo "JTop Socket:"
	@if [ -S /run/jtop.sock ]; then \
		echo "  ✅ /run/jtop.sock"; \
	else \
		echo "  $(RED)❌ /run/jtop.sock missing$(NC)"; \
	fi
	@echo ""
	@echo "$(GREEN)Configuration Files:$(NC)"
	@echo "CycloneDDS Config:"
	@if [ -f $(PWD)/cyclonedx.xml ]; then \
		echo "  ✅ $(PWD)/cyclonedx.xml"; \
	else \
		echo "  $(RED)❌ $(PWD)/cyclonedx.xml missing$(NC)"; \
	fi
	@echo "Photos Directory:"
	@if [ -d $(PHOTOS_DIR) ]; then \
		echo "  ✅ $(PHOTOS_DIR)"; \
	else \
		echo "  $(YELLOW)⚠️  $(PHOTOS_DIR) not found$(NC)"; \
	fi

dev-shell: docker-check x11-setup ## Run Navigation development shell without auto-setup
	@echo "$(BLUE)[INFO] Starting Navigation Jetson development shell...$(NC)"
	docker run -it \
		--name $(CONTAINER_NAME)_dev \
		--rm \
		--privileged \
		--net=host \
		--runtime=nvidia \
		--gpus all \
		-e DISPLAY=$$DISPLAY \
		-e QT_X11_NO_MITSHM=1 \
		-e XAUTHORITY=$(XAUTH) \
		-e GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11 \
		-e GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models \
		-e GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(XAUTH):$(XAUTH) \
		-v /run/user/1000/at-spi:/run/user/1000/at-spi \
		-v /dev:/dev \
		-v /run/jtop.sock:/run/jtop.sock \
		-v $(PARENT_DIR):/home/$(USERNAME)/dev_ws/src \
		-v nav_humble_jetson_home_volume:/home/$(USERNAME) \
		-v $(PWD)/cyclonedx.xml:/home/$(USERNAME)/cyclonedx.xml:ro \
		-v $(PHOTOS_DIR):/home/$(USERNAME)/dev_ws/photos_competition \
		--add-host=os-122140001125.local:169.254.55.220 \
		$(IMAGE_NAME) \
		/bin/bash -c " \
			export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
			source install/setup.bash 2>/dev/null || true; \
			echo '🔧 Navigation development shell ready!'; \
			/bin/bash"

logs: docker-check ## Show logs from the Navigation Jetson container
	@echo "$(BLUE)[INFO] Showing Navigation Jetson container logs...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker logs -f $(CONTAINER_NAME); \
	elif docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)_wheels$$"; then \
		docker logs -f $(CONTAINER_NAME)_wheels; \
	else \
		echo "$(YELLOW)[INFO] No containers are running$(NC)"; \
	fi

# Navigation specific commands
nav-test: ## Run Navigation stack tests (requires running container)
	@echo "$(BLUE)[INFO] Running Navigation stack tests...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)[ERROR] Container $(CONTAINER_NAME) is not running$(NC)"; \
		echo "Start it first with: make run"; \
		exit 1; \
	fi
	docker exec $(CONTAINER_NAME) /bin/bash -c " \
		cd /home/$(USERNAME)/dev_ws; \
		source install/setup.bash 2>/dev/null || true; \
		export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
		echo 'Running Navigation stack tests...'; \
		colcon test --packages-select nav_pkg || echo 'Tests completed with warnings'"

launch-nav: ## Launch full Navigation stack in running container
	@echo "$(BLUE)[INFO] Launching Navigation stack...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)[ERROR] Container $(CONTAINER_NAME) is not running$(NC)"; \
		echo "Start it first with: make run"; \
		exit 1; \
	fi
	docker exec $(CONTAINER_NAME) /bin/bash -c " \
		cd /home/$(USERNAME)/dev_ws; \
		source install/setup.bash 2>/dev/null || true; \
		export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
		ros2 launch nav_pkg navigation_stack.launch.py || echo 'Launch file may not exist yet'"

wheels-status: ## Check wheels control system status
	@echo "$(BLUE)[INFO] Checking wheels control system status...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)"; then \
		docker exec $(CONTAINER_NAME) /bin/bash -c " \
			export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
			echo 'Active ROS2 nodes:'; \
			ros2 node list 2>/dev/null | grep -E '(wheels|control)' || echo 'No wheels control nodes found'; \
			echo ''; \
			echo 'Available topics:'; \
			ros2 topic list 2>/dev/null | grep -E '(cmd_vel|wheel|motor)' || echo 'No wheels control topics found'"; \
	else \
		echo "$(YELLOW)[INFO] No Navigation containers are running$(NC)"; \
	fi

performance-check: ## Check Navigation performance metrics (requires running container)
	@echo "$(BLUE)[INFO] Checking Navigation performance metrics...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)"; then \
		echo "$(RED)[ERROR] No Navigation containers are running$(NC)"; \
		echo "Start one first with: make run"; \
		exit 1; \
	fi
	@container=$$(docker ps --format "table {{.Names}}" | grep "^$(CONTAINER_NAME)" | head -1); \
	docker exec $$container /bin/bash -c " \
		echo 'GPU Memory Usage:'; \
		nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader,nounits 2>/dev/null || echo 'GPU info not available'; \
		echo ''; \
		echo 'System Memory:'; \
		free -h; \
		echo ''; \
		echo 'CPU Usage:'; \
		top -bn1 | grep 'Cpu(s)' || echo 'CPU info not available'; \
		echo ''; \
		echo 'JTop Status:'; \
		if [ -S /run/jtop.sock ]; then echo 'JTop socket available'; else echo 'JTop socket not available'; fi"

jetson-info: ## Show detailed Jetson platform information
	@echo "$(BLUE)[INFO] Jetson Platform Information:$(NC)"
	@echo ""
	@echo "$(GREEN)Jetson Model:$(NC)"
	@if [ -f /proc/device-tree/model ]; then \
		cat /proc/device-tree/model; echo; \
	else \
		echo "$(YELLOW)Model information not available$(NC)"; \
	fi
	@echo ""
	@echo "$(GREEN)Jetson L4T Release:$(NC)"
	@if [ -f /etc/nv_tegra_release ]; then \
		cat /etc/nv_tegra_release; \
	else \
		echo "$(YELLOW)L4T information not available$(NC)"; \
	fi
	@echo ""
	@echo "$(GREEN)CUDA Version:$(NC)"
	@if command -v nvcc > /dev/null 2>&1; then \
		nvcc --version | tail -1; \
	else \
		echo "$(YELLOW)CUDA compiler not available$(NC)"; \
	fi
	@echo ""
	@echo "$(GREEN)JetPack Components:$(NC)"
	@if command -v jetson_release > /dev/null 2>&1; then \
		jetson_release -v; \
	else \
		echo "$(YELLOW)jetson_release not available$(NC)"; \
	fi

config-check: ## Validate configuration files and paths
	@echo "$(BLUE)[INFO] Validating Navigation configuration...$(NC)"
	@echo ""
	@echo "$(GREEN)CycloneDDS Configuration:$(NC)"
	@if [ -f $(PWD)/cyclonedx.xml ]; then \
		echo "✅ Configuration file exists: $(PWD)/cyclonedx.xml"; \
		echo "File size: $$(wc -c < $(PWD)/cyclonedx.xml) bytes"; \
	else \
		echo "$(RED)❌ CycloneDDS configuration missing: $(PWD)/cyclonedx.xml$(NC)"; \
	fi
	@echo ""
	@echo "$(GREEN)Volume Directories:$(NC)"
	@echo "Parent directory: $(PARENT_DIR)"
	@if [ -d "$(PARENT_DIR)" ]; then \
		echo "✅ Source mount point exists"; \
	else \
		echo "$(RED)❌ Source directory missing$(NC)"; \
	fi
	@echo ""
	@echo "Photos directory: $(PHOTOS_DIR)"
	@if [ -d "$(PHOTOS_DIR)" ]; then \
		echo "✅ Photos directory exists"; \
	else \
		echo "$(YELLOW)⚠️  Photos directory not found (optional)$(NC)"; \
	fi
	@echo ""
	@echo "$(GREEN)Network Configuration:$(NC)"
	@echo "Host mapping: os-122140001125.local:169.254.55.220"