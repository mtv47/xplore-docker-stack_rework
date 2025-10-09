# Rover Desktop Docker Makefile
# Provides convenient commands for managing the Rover Desktop containers

.PHONY: help build run run-mac run-cameras attach stop clean docker-check x11-setup

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
PARENT_DIR = $(shell dirname $(PWD))

# Export environment variables
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
	@echo "  make build       # Build the Rover image"
	@echo "  make run         # Run interactive container"
	@echo "  make run-mac     # Run for Mac with network"
	@echo "  make run-cameras # Run camera service"
	@echo "  make attach      # Attach to running container"
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

build: docker-check ## Build the Rover Desktop Docker image
	@echo "$(BLUE)[INFO] Building Rover Desktop image...$(NC)"
	docker build --no-cache --progress=plain --target build-rover -t $(IMAGE_NAME) -f Dockerfile ..

run: docker-check x11-setup ## Run interactive Rover Desktop container
	@echo "$(BLUE)[INFO] Starting Rover Desktop container...$(NC)"
	docker run -it \
		--name $(CONTAINER_NAME) \
		--rm \
		--privileged \
		--net=host \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e DISPLAY=unix$$DISPLAY \
		-e QT_X11_NO_MITSHM=1 \
		-e XAUTHORITY=$(XAUTH) \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(XAUTH):$(XAUTH) \
		-v /run/user/1000/at-spi:/run/user/1000/at-spi \
		-v /run/jtop.sock:/run/jtop.sock \
		-v /dev:/dev \
		-v $(PARENT_DIR):/home/$(USERNAME)/dev_ws/src \
		-v rover_humble_desktop_home_volume:/home/$(USERNAME) \
		$(IMAGE_NAME) \
		/bin/bash -c "sudo chown -R $(USERNAME):$(USERNAME) /home/$(USERNAME); /bin/bash"

run-mac: docker-check x11-setup ## Run Rover Desktop container for Mac with network
	@echo "$(BLUE)[INFO] Starting Rover Desktop container for Mac...$(NC)"
	docker run -it \
		--name $(CONTAINER_NAME) \
		--rm \
		--privileged \
		--network docker_humble_desktop_cs_frontend_net \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e DISPLAY=unix$$DISPLAY \
		-e QT_X11_NO_MITSHM=1 \
		-e XAUTHORITY=$(XAUTH) \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(XAUTH):$(XAUTH) \
		-v /run/user/1000/at-spi:/run/user/1000/at-spi \
		-v /run/jtop.sock:/run/jtop.sock \
		-v /dev:/dev \
		-v $(PARENT_DIR):/home/$(USERNAME)/dev_ws/src \
		-v rover_humble_desktop_home_volume:/home/$(USERNAME) \
		$(IMAGE_NAME) \
		/bin/bash -c "sudo chown -R $(USERNAME):$(USERNAME) /home/$(USERNAME); /bin/bash"

run-cameras: docker-check x11-setup ## Run camera service
	@echo "$(BLUE)[INFO] Starting camera service...$(NC)"
	docker run \
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
		-v $(PARENT_DIR):/home/$(USERNAME)/dev_ws/src \
		-v rover_humble_desktop_home_volume:/home/$(USERNAME) \
		$(IMAGE_NAME) \
		/bin/bash -c "colcon build && unset RMW_IMPLEMENTATION && source install/setup.bash && ros2 run rover_pkg new_camera_cs"

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
	@echo "🔗 Attaching to Rover Desktop container..."
	@echo ""
	docker exec -it $(CONTAINER_NAME) /bin/bash -c " \
		export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
		export DISPLAY=unix\$$DISPLAY; \
		export QT_X11_NO_MITSHM=1; \
		export PYTHONPATH=/home/$(USERNAME)/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/$(USERNAME)/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
		source install/setup.bash 2>/dev/null || true; \
		echo '🚀 Welcome to Rover Desktop container!'; \
		echo 'Environment configured with:'; \
		echo '  - ROS 2 Humble'; \
		echo '  - CycloneDDS middleware'; \
		echo '  - X11 forwarding'; \
		echo ''; \
		/bin/bash"

stop: docker-check ## Stop the Rover Desktop container
	@echo "$(BLUE)[INFO] Stopping Rover Desktop container...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker stop $(CONTAINER_NAME); \
		echo "$(GREEN)[SUCCESS] Container stopped$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Container is not running$(NC)"; \
	fi

clean: docker-check ## Clean up containers, volumes, and images
	@echo "$(YELLOW)Cleaning up Rover Desktop resources...$(NC)"
	@echo "This will remove containers, volumes, and images. Continue? [y/N]" && read ans && [ $${ans:-N} = y ]
	@if docker ps -a --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker rm -f $(CONTAINER_NAME); \
	fi
	@if docker volume ls -q | grep -q "rover_humble_desktop_home_volume"; then \
		docker volume rm rover_humble_desktop_home_volume; \
	fi
	@if docker images -q $(IMAGE_NAME) | head -1 | xargs -r docker rmi; then \
		echo "$(GREEN)Cleanup complete!$(NC)"; \
	fi

status: docker-check ## Show status of Rover Desktop containers and volumes
	@echo "$(BLUE)[INFO] Rover Desktop status:$(NC)"
	@echo ""
	@echo "$(GREEN)Containers:$(NC)"
	@docker ps -a --filter "name=$(CONTAINER_NAME)" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" || echo "No containers found"
	@echo ""
	@echo "$(GREEN)Volumes:$(NC)"
	@docker volume ls --filter "name=rover_humble_desktop" --format "table {{.Name}}\t{{.Driver}}" || echo "No volumes found"
	@echo ""
	@echo "$(GREEN)Images:$(NC)"
	@docker images $(IMAGE_NAME) --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}" || echo "No images found"