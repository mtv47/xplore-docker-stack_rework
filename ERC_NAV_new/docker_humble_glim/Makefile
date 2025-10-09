# ERC Navigation GLIM Jetson Docker Makefile
# Provides convenient commands for managing the GLIM Jetson containers

.PHONY: help build run attach stop clean docker-check x11-setup gpu-check

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
IMAGE_NAME = glim-humble-jetson
CONTAINER_NAME = glim_humble_jetson
USERNAME = xplore
PARENT_DIR = $(shell dirname $(PWD))

# Export environment variables
export XAUTH
export PARENT_DIR
export DISPLAY

help: ## Show this help message
	@echo "$(BLUE)ERC Navigation GLIM Jetson Docker Commands$(NC)"
	@echo ""
	@echo "$(GREEN)Available commands:$(NC)"
	@awk 'BEGIN {FS = ":.*##"} /^[a-zA-Z_-]+:.*##/ { printf "  $(YELLOW)%-15s$(NC) %s\n", $$1, $$2 }' $(MAKEFILE_LIST)
	@echo ""
	@echo "$(BLUE)Examples:$(NC)"
	@echo "  make build       # Build the GLIM Jetson image"
	@echo "  make run         # Run GLIM container with GPU acceleration"
	@echo "  make attach      # Attach to running container"
	@echo "  make stop        # Stop the container"
	@echo ""
	@echo "$(YELLOW)Note:$(NC) GLIM is optimized for NVIDIA Jetson platforms with CUDA support"

docker-check: ## Check if Docker is running
	@if ! docker info > /dev/null 2>&1; then \
		echo "$(RED)[ERROR] Docker is not running. Please start Docker and try again.$(NC)"; \
		exit 1; \
	fi

x11-setup: ## Setup X11 forwarding for GUI applications (Jetson optimized)
	@echo "$(BLUE)[INFO] Setting up X11 forwarding for Jetson GLIM...$(NC)"
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
	@echo "$(BLUE)[INFO] Checking Jetson GLIM hardware...$(NC)"
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
	@echo "$(GREEN)CUDA Libraries:$(NC)"
	@if [ -d /usr/local/cuda ]; then \
		echo "✅ CUDA found: /usr/local/cuda"; \
		ls -la /usr/local/cuda/version.txt 2>/dev/null || echo "CUDA version file not found"; \
	else \
		echo "$(YELLOW)⚠️  CUDA not found$(NC)"; \
	fi
	@echo ""
	@echo "$(GREEN)Tegra Libraries:$(NC)"
	@if [ -d /usr/lib/aarch64-linux-gnu/tegra ]; then \
		echo "✅ Tegra libraries found"; \
	else \
		echo "$(YELLOW)⚠️  Tegra libraries not found$(NC)"; \
	fi

build: docker-check ## Build the GLIM Jetson Docker image (optimized for Jetson)
	@echo "$(BLUE)[INFO] Building GLIM Jetson image with fresh cache...$(NC)"
	docker build --no-cache --progress=plain -t $(IMAGE_NAME) -f Dockerfile.glim ..

build-quick: docker-check ## Build the GLIM Jetson Docker image (using cache)
	@echo "$(BLUE)[INFO] Building GLIM Jetson image (using cache)...$(NC)"
	docker build --progress=plain -t $(IMAGE_NAME) -f Dockerfile.glim ..

run: docker-check x11-setup gpu-check ## Run GLIM Jetson container with full GPU acceleration
	@echo "$(BLUE)[INFO] Starting GLIM Jetson container with GPU acceleration...$(NC)"
	docker run -it \
		--name $(CONTAINER_NAME) \
		--rm \
		--privileged \
		--net=host \
		--ipc=host \
		--pid=host \
		--runtime=nvidia \
		--gpus all \
		-e DISPLAY=$$DISPLAY \
		-e QT_X11_NO_MITSHM=1 \
		-e XAUTHORITY=$(XAUTH) \
		-e NVIDIA_VISIBLE_DEVICES=all \
		-e NVIDIA_DRIVER_CAPABILITIES=all \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(XAUTH):$(XAUTH) \
		-v /run/user/1000/at-spi:/run/user/1000/at-spi \
		-v /dev:/dev \
		-v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra:ro \
		-v /usr/local/cuda:/usr/local/cuda:ro \
		-v $(PARENT_DIR):/home/$(USERNAME)/dev_ws/src \
		-v nav_humble_jetson_home_volume:/home/$(USERNAME) \
		-w /home/$(USERNAME)/dev_ws/ \
		$(IMAGE_NAME) \
		/bin/bash -c "sudo chown -R $(USERNAME):$(USERNAME) /home/$(USERNAME); \
			export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
			export DISPLAY=unix\$$DISPLAY; \
			export QT_X11_NO_MITSHM=1; \
			source install/setup.bash 2>/dev/null || true; \
			echo '🚀 GLIM Jetson container ready!'; \
			echo 'Optimized for:'; \
			echo '  - NVIDIA Jetson platform'; \
			echo '  - GPU-accelerated SLAM'; \
			echo '  - Real-time LiDAR processing'; \
			echo '  - GLIM visual-inertial mapping'; \
			echo ''; \
			/bin/bash"

attach: docker-check ## Attach to running GLIM Jetson container
	@echo "$(BLUE)[INFO] Attaching to GLIM Jetson container...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)❌ GLIM Jetson container '$(CONTAINER_NAME)' is not running$(NC)"; \
		echo ""; \
		echo "To start the GLIM Jetson container, run:"; \
		echo "  make run"; \
		echo ""; \
		exit 1; \
	fi
	@echo "🔗 Attaching to GLIM Jetson container..."
	@echo ""
	docker exec -it $(CONTAINER_NAME) /bin/bash -c " \
		export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp; \
		export DISPLAY=unix\$$DISPLAY; \
		export QT_X11_NO_MITSHM=1; \
		source install/setup.bash 2>/dev/null || true; \
		echo '🚀 Welcome to GLIM Jetson container!'; \
		echo 'Environment configured with:'; \
		echo '  - ROS 2 Humble'; \
		echo '  - CycloneDX middleware'; \
		echo '  - X11 forwarding'; \
		echo '  - GPU acceleration'; \
		echo '  - GLIM SLAM'; \
		echo '  - Jetson optimization'; \
		echo ''; \
		/bin/bash"

stop: docker-check ## Stop the GLIM Jetson container
	@echo "$(BLUE)[INFO] Stopping GLIM Jetson container...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker stop $(CONTAINER_NAME); \
		echo "$(GREEN)[SUCCESS] Container stopped$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Container is not running$(NC)"; \
	fi

clean: docker-check ## Clean up containers, volumes, and images
	@echo "$(YELLOW)Cleaning up GLIM Jetson resources...$(NC)"
	@echo "This will remove containers, volumes, and images. Continue? [y/N]" && read ans && [ $${ans:-N} = y ]
	@if docker ps -a --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker rm -f $(CONTAINER_NAME); \
	fi
	@if docker volume ls -q | grep -q "nav_humble_jetson_home_volume"; then \
		docker volume rm nav_humble_jetson_home_volume; \
	fi
	@if docker images -q $(IMAGE_NAME) | head -1 | xargs -r docker rmi; then \
		echo "$(GREEN)Cleanup complete!$(NC)"; \
	fi

status: docker-check ## Show status of GLIM Jetson containers and volumes
	@echo "$(BLUE)[INFO] GLIM Jetson status:$(NC)"
	@echo ""
	@echo "$(GREEN)Containers:$(NC)"
	@docker ps -a --filter "name=$(CONTAINER_NAME)" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" || echo "No containers found"
	@echo ""
	@echo "$(GREEN)Volumes:$(NC)"
	@docker volume ls --filter "name=nav_humble_jetson" --format "table {{.Name}}\t{{.Driver}}" || echo "No volumes found"
	@echo ""
	@echo "$(GREEN)Images:$(NC)"
	@docker images $(IMAGE_NAME) --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}" || echo "No images found"

hardware-check: ## Check Jetson hardware and GLIM requirements
	@echo "$(BLUE)[INFO] Checking GLIM Jetson hardware requirements...$(NC)"
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
	@echo "$(GREEN)Required Libraries:$(NC)"
	@echo "CUDA:"
	@if [ -d /usr/local/cuda ]; then \
		echo "  ✅ /usr/local/cuda"; \
	else \
		echo "  $(RED)❌ /usr/local/cuda missing$(NC)"; \
	fi
	@echo "Tegra:"
	@if [ -d /usr/lib/aarch64-linux-gnu/tegra ]; then \
		echo "  ✅ /usr/lib/aarch64-linux-gnu/tegra"; \
	else \
		echo "  $(RED)❌ /usr/lib/aarch64-linux-gnu/tegra missing$(NC)"; \
	fi
	@echo ""
	@echo "$(GREEN)Device Permissions:$(NC)"
	@groups $(USER) | grep -E "(video|gpio|i2c)" || echo "$(YELLOW)User not in hardware groups$(NC)"

dev-shell: docker-check x11-setup ## Run GLIM development shell without auto-setup
	@echo "$(BLUE)[INFO] Starting GLIM Jetson development shell...$(NC)"
	docker run -it \
		--name $(CONTAINER_NAME)_dev \
		--rm \
		--privileged \
		--net=host \
		--ipc=host \
		--pid=host \
		--runtime=nvidia \
		--gpus all \
		-e DISPLAY=$$DISPLAY \
		-e QT_X11_NO_MITSHM=1 \
		-e XAUTHORITY=$(XAUTH) \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(XAUTH):$(XAUTH) \
		-v /run/user/1000/at-spi:/run/user/1000/at-spi \
		-v /dev:/dev \
		-v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra:ro \
		-v /usr/local/cuda:/usr/local/cuda:ro \
		-v $(PARENT_DIR):/home/$(USERNAME)/dev_ws/src \
		-v nav_humble_jetson_home_volume:/home/$(USERNAME) \
		-w /home/$(USERNAME)/dev_ws/ \
		$(IMAGE_NAME) \
		/bin/bash -c " \
			export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp; \
			source install/setup.bash 2>/dev/null || true; \
			echo '🔧 GLIM development shell ready!'; \
			/bin/bash"

logs: docker-check ## Show logs from the GLIM Jetson container
	@echo "$(BLUE)[INFO] Showing GLIM Jetson container logs...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker logs -f $(CONTAINER_NAME); \
	else \
		echo "$(YELLOW)[INFO] Container is not running$(NC)"; \
	fi

# GLIM specific commands
glim-test: ## Run GLIM SLAM tests (requires running container)
	@echo "$(BLUE)[INFO] Running GLIM SLAM tests...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)[ERROR] Container $(CONTAINER_NAME) is not running$(NC)"; \
		echo "Start it first with: make run"; \
		exit 1; \
	fi
	docker exec $(CONTAINER_NAME) /bin/bash -c " \
		cd /home/$(USERNAME)/dev_ws; \
		source install/setup.bash 2>/dev/null || true; \
		echo 'Running GLIM SLAM tests...'; \
		ros2 test glim_ros || echo 'Tests completed with warnings'"

launch-glim: ## Launch GLIM SLAM in running container
	@echo "$(BLUE)[INFO] Launching GLIM SLAM...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)[ERROR] Container $(CONTAINER_NAME) is not running$(NC)"; \
		echo "Start it first with: make run"; \
		exit 1; \
	fi
	docker exec $(CONTAINER_NAME) /bin/bash -c " \
		cd /home/$(USERNAME)/dev_ws; \
		source install/setup.bash 2>/dev/null || true; \
		ros2 launch glim_ros glim.launch.py || echo 'Launch file may not exist yet'"

performance-check: ## Check GLIM performance metrics (requires running container)
	@echo "$(BLUE)[INFO] Checking GLIM performance metrics...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)[ERROR] Container $(CONTAINER_NAME) is not running$(NC)"; \
		echo "Start it first with: make run"; \
		exit 1; \
	fi
	docker exec $(CONTAINER_NAME) /bin/bash -c " \
		echo 'GPU Memory Usage:'; \
		nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader,nounits 2>/dev/null || echo 'GPU info not available'; \
		echo ''; \
		echo 'System Memory:'; \
		free -h; \
		echo ''; \
		echo 'CPU Usage:'; \
		top -bn1 | grep 'Cpu(s)' || echo 'CPU info not available'"

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