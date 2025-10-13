# ERC Navigation Desktop Docker Makefile
# Provides convenient commands for managing the Navigation Desktop containers

.PHONY: help build run run-gpu run-no-gpu attach stop clean docker-check x11-setup gpu-check

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
IMAGE_NAME = ghcr.io/epflxplore/nav:humble-desktop
CONTAINER_NAME = nav_humble_desktop
USERNAME = xplore
PARENT_DIR = $(shell dirname $(PWD))
LIDAR_HOST = os-122140001125.local:169.254.55.220

# Export environment variables
export XAUTH
export PARENT_DIR
export DISPLAY

help: ## Show this help message
	@echo "$(BLUE)ERC Navigation Desktop Docker Commands$(NC)"
	@echo ""
	@echo "$(GREEN)Available commands:$(NC)"
	@awk 'BEGIN {FS = ":.*##"} /^[a-zA-Z_-]+:.*##/ { printf "  $(YELLOW)%-15s$(NC) %s\n", $$1, $$2 }' $(MAKEFILE_LIST)
	@echo ""
	@echo "$(BLUE)Examples:$(NC)"
	@echo "  make build       # Build the Navigation Desktop image"
	@echo "  make run-gpu     # Run with NVIDIA GPU acceleration"
	@echo "  make run-no-gpu  # Run without GPU (compatibility mode)"
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

gpu-check: ## Check GPU availability and NVIDIA runtime
	@echo "$(BLUE)[INFO] Checking GPU availability...$(NC)"
	@if command -v nvidia-smi > /dev/null 2>&1; then \
		echo "$(GREEN)[INFO] NVIDIA GPU detected:$(NC)"; \
		nvidia-smi --query-gpu=name,memory.total,memory.used --format=csv,noheader,nounits 2>/dev/null || echo "$(YELLOW)[WARNING] Could not query GPU details$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] No NVIDIA GPU detected$(NC)"; \
	fi
	@echo ""
	@echo "$(GREEN)Docker NVIDIA Runtime:$(NC)"
	@if docker info 2>/dev/null | grep -q nvidia; then \
		echo "✅ NVIDIA Docker runtime available"; \
	else \
		echo "$(YELLOW)⚠️  NVIDIA Docker runtime not available$(NC)"; \
	fi

build: docker-check ## Build the Navigation Desktop Docker image (with cache refresh)
	@echo "$(BLUE)[INFO] Building Navigation Desktop image with fresh cache...$(NC)"
	docker build --pull --no-cache --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

build-quick: docker-check ## Build the Navigation Desktop Docker image (using cache)
	@echo "$(BLUE)[INFO] Building Navigation Desktop image (using cache)...$(NC)"
	docker build --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

run: run-gpu ## Default run with GPU acceleration (alias for run-gpu)

run-gpu: docker-check x11-setup gpu-check ## Run Navigation Desktop container with NVIDIA GPU acceleration
	@echo "$(BLUE)[INFO] Starting Navigation Desktop container with GPU acceleration...$(NC)"
	docker run -it \
		--name $(CONTAINER_NAME) \
		--rm \
		--runtime=nvidia \
		--gpus all \
		--privileged \
		--net=host \
		-e DISPLAY=unix$$DISPLAY \
		-e QT_X11_NO_MITSHM=1 \
		-e XAUTHORITY=$(XAUTH) \
		-e NVIDIA_VISIBLE_DEVICES=all \
		-e NVIDIA_DRIVER_CAPABILITIES=all \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(XAUTH):$(XAUTH) \
		-v /run/user/1000/at-spi:/run/user/1000/at-spi \
		-v /dev:/dev \
		-v $(PARENT_DIR):/home/$(USERNAME)/dev_ws/src \
		-v nav_humble_desktop_home_volume:/home/$(USERNAME) \
		--add-host=$(LIDAR_HOST) \
		$(IMAGE_NAME) \
		/bin/bash -c "sudo chown -R $(USERNAME):$(USERNAME) /home/$(USERNAME); \
			echo '🚀 Navigation Desktop with GPU acceleration ready!'; \
			echo 'GPU-accelerated features available:'; \
			echo '  - SLAM processing'; \
			echo '  - Computer vision'; \
			echo '  - Real-time mapping'; \
			echo '  - LiDAR processing'; \
			echo ''; \
			/bin/bash"

run-no-gpu: docker-check x11-setup ## Run Navigation Desktop container without GPU (compatibility mode)
	@echo "$(BLUE)[INFO] Starting Navigation Desktop container in compatibility mode (no GPU)...$(NC)"
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
		-v nav_humble_desktop_home_volume:/home/$(USERNAME) \
		--add-host=$(LIDAR_HOST) \
		$(IMAGE_NAME) \
		/bin/bash -c "sudo chown -R $(USERNAME):$(USERNAME) /home/$(USERNAME); \
			echo '🚀 Navigation Desktop ready (compatibility mode)!'; \
			echo 'Running without GPU acceleration'; \
			echo 'Note: Some features may run slower'; \
			echo ''; \
			/bin/bash"

attach: docker-check ## Attach to running Navigation Desktop container
	@echo "$(BLUE)[INFO] Attaching to Navigation Desktop container...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)❌ Navigation Desktop container '$(CONTAINER_NAME)' is not running$(NC)"; \
		echo ""; \
		echo "To start the Navigation Desktop, run:"; \
		echo "  make run-gpu     # With GPU acceleration"; \
		echo "  make run-no-gpu  # Compatibility mode"; \
		echo ""; \
		exit 1; \
	fi
	@echo "🔗 Attaching to Navigation Desktop container..."
	@echo ""
	docker exec -it $(CONTAINER_NAME) /bin/bash -c " \
		echo '🚀 Welcome to Navigation Desktop container!'; \
		echo 'Environment configured with:'; \
		echo '  - ROS 2 Humble'; \
		echo '  - Navigation stack'; \
		echo '  - SLAM capabilities'; \
		echo '  - LiDAR integration'; \
		echo '  - Real-time mapping'; \
		echo ''; \
		/bin/bash"

stop: docker-check ## Stop the Navigation Desktop container
	@echo "$(BLUE)[INFO] Stopping Navigation Desktop container...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker stop $(CONTAINER_NAME); \
		echo "$(GREEN)[SUCCESS] Container stopped$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Container is not running$(NC)"; \
	fi

clean: docker-check ## Clean up containers, volumes, and images
	@echo "$(YELLOW)Cleaning up Navigation Desktop resources...$(NC)"
	@echo "This will remove containers, volumes, and images. Continue? [y/N]" && read ans && [ $${ans:-N} = y ]
	@if docker ps -a --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker rm -f $(CONTAINER_NAME); \
	fi
	@if docker volume ls -q | grep -q "nav_humble_desktop_home_volume"; then \
		docker volume rm nav_humble_desktop_home_volume; \
	fi
	@if docker images -q $(IMAGE_NAME) | head -1 | xargs -r docker rmi; then \
		echo "$(GREEN)Cleanup complete!$(NC)"; \
	fi

status: docker-check ## Show status of Navigation Desktop containers and volumes
	@echo "$(BLUE)[INFO] Navigation Desktop status:$(NC)"
	@echo ""
	@echo "$(GREEN)Containers:$(NC)"
	@docker ps -a --filter "name=$(CONTAINER_NAME)" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" || echo "No containers found"
	@echo ""
	@echo "$(GREEN)Volumes:$(NC)"
	@docker volume ls --filter "name=nav_humble_desktop" --format "table {{.Name}}\t{{.Driver}}" || echo "No volumes found"
	@echo ""
	@echo "$(GREEN)Images:$(NC)"
	@docker images $(IMAGE_NAME) --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}" || echo "No images found"

logs: docker-check ## Show logs from the Navigation Desktop container
	@echo "$(BLUE)[INFO] Showing Navigation Desktop container logs...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker logs -f $(CONTAINER_NAME); \
	else \
		echo "$(YELLOW)[INFO] Container is not running$(NC)"; \
	fi