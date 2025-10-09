# Avionics ROS Jetson Docker Makefile
# Provides convenient commands for managing the Avionics ROS Jetson containers

.PHONY: help build run attach stop clean docker-check x11-setup start-stack stop-stack

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
PARENT_DIR = $(shell dirname $(PWD))

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
	@echo "  make build        # Build the Avionics ROS Jetson image"
	@echo "  make run          # Run interactive container"
	@echo "  make start-stack  # Start avionics stack automatically"
	@echo "  make attach       # Attach to running container"
	@echo "  make stop         # Stop the container"
	@echo ""

docker-check: ## Check if Docker is running
	@if ! docker info > /dev/null 2>&1; then \
		echo "$(RED)[ERROR] Docker is not running. Please start Docker and try again.$(NC)"; \
		exit 1; \
	fi

x11-setup: ## Setup X11 forwarding for GUI applications
	@echo "$(BLUE)[INFO] Setting up X11 forwarding...$(NC)"
	@if [ -f "$(XAUTH)" ] && ! xauth -f "$(XAUTH)" list > /dev/null 2>&1; then \
		echo "$(YELLOW)[WARNING] Removing corrupted X authority file...$(NC)"; \
		rm -f "$(XAUTH)"; \
	fi
	@if [ ! -f "$(XAUTH)" ]; then \
		echo "$(BLUE)[INFO] Creating X authority file...$(NC)"; \
		xauth_list=$$(xauth nlist :0 2>/dev/null | tail -n 1 | sed -e 's/^..../ffff/' || true); \
		if [ -n "$$xauth_list" ]; then \
			echo "$$xauth_list" | xauth -f "$(XAUTH)" nmerge -; \
			echo "$(GREEN)[SUCCESS] X authority data merged successfully$(NC)"; \
		else \
			touch "$(XAUTH)"; \
			echo "$(YELLOW)[WARNING] Created empty X authority file (no X session found)$(NC)"; \
		fi; \
		chmod a+r "$(XAUTH)"; \
	fi
	@echo "$(BLUE)[INFO] Verifying file contents:$(NC)"
	@file $(XAUTH) || echo "$(YELLOW)[WARNING] Could not verify X authority file$(NC)"
	@echo "$(BLUE)[INFO] Permissions:$(NC)"
	@ls -FAlh $(XAUTH)

build: docker-check ## Build the Avionics ROS Jetson Docker image
	@echo "$(BLUE)[INFO] Building Avionics ROS Jetson image...$(NC)"
	docker build --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

run: docker-check x11-setup ## Run Avionics ROS Jetson container (interactive)
	@echo "$(BLUE)[INFO] Starting Avionics ROS Jetson container...$(NC)"
	docker run -it \
		--name $(CONTAINER_NAME) \
		--rm \
		--privileged \
		--net=host \
		-e DISPLAY=unix$$DISPLAY \
		-e QT_X11_NO_MITSHM=1 \
		-e XAUTHORITY=$(XAUTH) \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(XAUTH):$(XAUTH) \
		-v /run/user/1000/at-spi:/run/user/1000/at-spi \
		-v /dev:/dev \
		-v $(PARENT_DIR):/home/$(USERNAME)/dev_ws/src \
		-v elec_humble_jetson_home_volume:/home/$(USERNAME) \
		$(IMAGE_NAME) \
		/bin/bash -c "sudo chown -R $(USERNAME):$(USERNAME) /home/$(USERNAME); /bin/bash"

start-stack: docker-check x11-setup ## Start Avionics ROS stack automatically (production mode)
	@echo "$(BLUE)[INFO] Starting Avionics ROS Jetson stack...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(YELLOW)[INFO] Container $(CONTAINER_NAME) is already running. Launching stack...$(NC)"; \
		docker exec -it $(CONTAINER_NAME) /bin/bash -c " \
			sudo chown -R $(USERNAME):$(USERNAME) /home/$(USERNAME); \
			cd ..; \
			export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp; \
			export DISPLAY=unix\$$DISPLAY; \
			export QT_X11_NO_MITSHM=1; \
			export PYTHONPATH=/home/$(USERNAME)/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/$(USERNAME)/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
			source install/setup.bash 2>/dev/null || true; \
			ros2 launch avionics_nexus launch.py"; \
	else \
		echo "$(BLUE)[INFO] Starting new container with avionics stack...$(NC)"; \
		docker run -i \
			--name $(CONTAINER_NAME) \
			--rm \
			--privileged \
			--net=host \
			-e DISPLAY=$$DISPLAY \
			-e QT_X11_NO_MITSHM=1 \
			-e XAUTHORITY=$(XAUTH) \
			-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
			-v $(XAUTH):$(XAUTH) \
			-v /run/user/1000/at-spi:/run/user/1000/at-spi \
			-v /dev:/dev \
			-v $(PARENT_DIR):/home/$(USERNAME)/dev_ws/src \
			-v elec_humble_jetson_home_volume:/home/$(USERNAME) \
			$(IMAGE_NAME) \
			/bin/bash -c "sudo bash -c 'cd /home/$(USERNAME)/dev_ws; source install/setup.sh; ros2 launch avionics_nexus launch.py'"; \
	fi

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
		export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp; \
		export DISPLAY=unix\$$DISPLAY; \
		export QT_X11_NO_MITSHM=1; \
		export PYTHONPATH=/home/$(USERNAME)/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/$(USERNAME)/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
		source install/setup.bash 2>/dev/null || true; \
		echo '🚀 Welcome to Avionics ROS Jetson container!'; \
		echo 'Environment configured with:'; \
		echo '  - ROS 2 Humble'; \
		echo '  - CycloneDX middleware'; \
		echo '  - X11 forwarding'; \
		echo '  - Jetson hardware optimization'; \
		echo '  - Full device access'; \
		echo ''; \
		/bin/bash"

stop: docker-check ## Stop the Avionics ROS Jetson container
	@echo "$(BLUE)[INFO] Stopping Avionics ROS Jetson container...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker stop $(CONTAINER_NAME); \
		echo "$(GREEN)[SUCCESS] Container stopped$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Container is not running$(NC)"; \
	fi

stop-stack: stop ## Stop the Avionics ROS stack (alias for stop)

clean: docker-check ## Clean up containers, volumes, and images
	@echo "$(YELLOW)Cleaning up Avionics ROS Jetson resources...$(NC)"
	@echo "This will remove containers, volumes, and images. Continue? [y/N]" && read ans && [ $${ans:-N} = y ]
	@if docker ps -a --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker rm -f $(CONTAINER_NAME); \
	fi
	@if docker volume ls -q | grep -q "elec_humble_jetson_home_volume"; then \
		docker volume rm elec_humble_jetson_home_volume; \
	fi
	@if docker images -q $(IMAGE_NAME) | head -1 | xargs -r docker rmi; then \
		echo "$(GREEN)Cleanup complete!$(NC)"; \
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

hardware-check: ## Check Jetson hardware and device availability
	@echo "$(BLUE)[INFO] Checking Jetson hardware interfaces...$(NC)"
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
	@echo "$(GREEN)Serial Devices:$(NC)"
	@if ls /dev/ttyUSB* > /dev/null 2>&1; then \
		ls -la /dev/ttyUSB*; \
	else \
		echo "$(YELLOW)No USB serial devices found$(NC)"; \
	fi
	@if ls /dev/ttyACM* > /dev/null 2>&1; then \
		ls -la /dev/ttyACM*; \
	else \
		echo "$(YELLOW)No ACM serial devices found$(NC)"; \
	fi
	@echo ""
	@echo "$(GREEN)Device Permissions:$(NC)"
	@groups $(USER) | grep -E "(dialout|video|gpio)" || echo "$(YELLOW)User not in hardware groups$(NC)"

dev-shell: docker-check x11-setup ## Run Jetson development shell without auto-launch
	@echo "$(BLUE)[INFO] Starting Avionics ROS Jetson development shell...$(NC)"
	docker run -it \
		--name $(CONTAINER_NAME)_dev \
		--rm \
		--privileged \
		--net=host \
		-e DISPLAY=unix$$DISPLAY \
		-e QT_X11_NO_MITSHM=1 \
		-e XAUTHORITY=$(XAUTH) \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(XAUTH):$(XAUTH) \
		-v /run/user/1000/at-spi:/run/user/1000/at-spi \
		-v /dev:/dev \
		-v $(PARENT_DIR):/home/$(USERNAME)/dev_ws/src \
		-v elec_humble_jetson_home_volume:/home/$(USERNAME) \
		$(IMAGE_NAME) \
		/bin/bash -c " \
			export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp; \
			export PYTHONPATH=/home/$(USERNAME)/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/$(USERNAME)/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
			source install/setup.bash 2>/dev/null || true; \
			echo '🔧 Jetson development shell ready!'; \
			/bin/bash"

launch-stack: ## Launch avionics stack in running container
	@echo "$(BLUE)[INFO] Launching avionics stack in running container...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)[ERROR] Container $(CONTAINER_NAME) is not running$(NC)"; \
		echo "Start it first with: make run or make start-stack"; \
		exit 1; \
	fi
	docker exec $(CONTAINER_NAME) /bin/bash -c " \
		cd /home/$(USERNAME)/dev_ws; \
		export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp; \
		source install/setup.bash 2>/dev/null || true; \
		ros2 launch avionics_nexus launch.py"

logs: docker-check ## Show logs from the Avionics ROS Jetson container
	@echo "$(BLUE)[INFO] Showing Avionics ROS Jetson container logs...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker logs -f $(CONTAINER_NAME); \
	else \
		echo "$(YELLOW)[INFO] Container is not running$(NC)"; \
	fi