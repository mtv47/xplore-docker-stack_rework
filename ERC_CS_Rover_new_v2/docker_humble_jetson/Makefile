# Rover Jetson Docker Makefile
# Provides convenient commands for managing the Rover Jetson containers

.PHONY: help build build-quick run run-rover attach stop stop-rover stop-cameras clean docker-check x11-setup

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
PARENT_DIR = $(shell dirname $(PWD))
JTOP_GID = $(shell getent group jtop | awk -F: '{print $$3}' 2>/dev/null || echo "")

# Export environment variables
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
		-v /run/jtop.sock:/run/jtop.sock \
		-v /dev:/dev \
		-v /home/$(USERNAME)/photos_competition:/home/$(USERNAME)/dev_ws/photos_competition \
		-v $(PARENT_DIR):/home/$(USERNAME)/dev_ws/src \
		-v rover_humble_jetson_home_volume:/home/$(USERNAME) \
		$(IMAGE_NAME) \
		/bin/bash -c "sudo chown -R $(USERNAME):$(USERNAME) /home/$(USERNAME); /bin/bash"

run-rover: docker-check x11-setup ## Run rover launch script (start or attach)
	@echo "$(BLUE)[INFO] Starting/Attaching to Rover...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(GREEN)Container $(CONTAINER_NAME) is already running. Attaching to it...$(NC)"; \
		docker exec -it $(CONTAINER_NAME) /bin/bash -c " \
			sudo chown -R $(USERNAME):$(USERNAME) /home/$(USERNAME); \
			export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
			export DISPLAY=unix\$$DISPLAY; \
			export QT_X11_NO_MITSHM=1; \
			export PYTHONPATH=/home/$(USERNAME)/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/$(USERNAME)/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
			source install/setup.bash; \
			ros2 launch rover_pkg launch.py"; \
	else \
		echo "$(GREEN)Container $(CONTAINER_NAME) is not running. Starting a new container...$(NC)"; \
		docker run -i \
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
			-v /home/$(USERNAME)/photos_competition:/home/$(USERNAME)/dev_ws/photos_competition \
			-v $(PARENT_DIR):/home/$(USERNAME)/dev_ws/src \
			-v rover_humble_jetson_home_volume:/home/$(USERNAME) \
			$(IMAGE_NAME) \
			/bin/bash -c " \
				sudo chown -R $(USERNAME):$(USERNAME) /home/$(USERNAME); \
				export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
				export DISPLAY=unix\$$DISPLAY; \
				export QT_X11_NO_MITSHM=1; \
				export PYTHONPATH=/home/$(USERNAME)/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/$(USERNAME)/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
				source install/setup.bash; \
				ros2 launch rover_pkg launch.py"; \
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
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker stop $(CONTAINER_NAME); \
		echo "$(GREEN)[SUCCESS] Container stopped$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Container is not running$(NC)"; \
	fi

stop-rover: stop ## Alias for stop (stop rover container)

stop-cameras: docker-check ## Stop the camera container
	@echo "$(BLUE)[INFO] Stopping camera container...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CAMERA_CONTAINER_NAME)$$"; then \
		docker stop $(CAMERA_CONTAINER_NAME); \
		echo "$(GREEN)[SUCCESS] Camera container stopped$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Camera container is not running$(NC)"; \
	fi

clean: docker-check ## Clean up containers, volumes, and images
	@echo "$(YELLOW)Cleaning up Rover Jetson resources...$(NC)"
	@echo "This will remove containers, volumes, and images. Continue? [y/N]" && read ans && [ $${ans:-N} = y ]
	@if docker ps -a --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker rm -f $(CONTAINER_NAME); \
	fi
	@if docker ps -a --format "table {{.Names}}" | grep -q "^$(CAMERA_CONTAINER_NAME)$$"; then \
		docker rm -f $(CAMERA_CONTAINER_NAME); \
	fi
	@if docker volume ls -q | grep -q "rover_humble_jetson_home_volume"; then \
		docker volume rm rover_humble_jetson_home_volume; \
	fi
	@if docker images -q $(IMAGE_NAME) | head -1 | xargs -r docker rmi; then \
		echo "$(GREEN)Cleanup complete!$(NC)"; \
	fi

status: docker-check ## Show status of Rover Jetson containers and volumes
	@echo "$(BLUE)[INFO] Rover Jetson status:$(NC)"
	@echo ""
	@echo "$(GREEN)Containers:$(NC)"
	@docker ps -a --filter "name=rover_humble_jetson" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" || echo "No containers found"
	@echo ""
	@echo "$(GREEN)Volumes:$(NC)"
	@docker volume ls --filter "name=rover_humble_jetson" --format "table {{.Name}}\t{{.Driver}}" || echo "No volumes found"
	@echo ""
	@echo "$(GREEN)Images:$(NC)"
	@docker images $(IMAGE_NAME) --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}" || echo "No images found"