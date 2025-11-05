# ERC Navigation GLIM Jetson Docker Makefile
# Provides convenient commands for managing the GLIM Jetson containers

.PHONY: help build run attach stop compose-check x11-setup gpu-check, status logs create_volume remove_volume remove_container

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
PARENT_DIR := $(abspath $(CURDIR)/..)

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

compose-check: ## Check if Docker Compose is available
	@if ! docker compose version > /dev/null 2>&1; then \
		echo "$(RED)[ERROR] Docker Compose is not available. Please install Docker Compose and try again.$(NC)"; \
		exit 1; \
	fi
	@if ! docker info > /dev/null 2>&1; then \
		echo "$(RED)[ERROR] Docker is not running. Please start Docker and try again.$(NC)"; \
		exit 1; \
	fi

x11-setup: ## Setup X11 forwarding for GUI applications (Jetson optimized)
	@echo "$(BLUE)[INFO] Setting up X11 forwarding for Jetson GLIM...$(NC)"
	@if [ ! -f "$(XAUTH)" ]; then \
		touch "$(XAUTH)"; \
	fi
	@xauth nlist $$DISPLAY | sed -e 's/^..../ffff/' | xauth -f "$(XAUTH)" nmerge -
	@chmod a+r "$(XAUTH)"
	@echo "$(GREEN)[INFO] Done.$(NC)"
	@echo ""
	@echo "$(BLUE)[INFO] Verifying file contents:$(NC)"
	@file $(XAUTH) || echo "$(YELLOW)[WARNING] Could not verify X authority file$(NC)"
	@echo "--> It should say \"X11 Xauthority data\"."
	@echo ""
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

build: compose-check ## Build the GLIM Jetson Docker image (optimized for Jetson)
	@echo "$(BLUE)[INFO] Building GLIM Jetson image with fresh cache...$(NC)"
	docker build --pull --no-cache --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..	

build-quick: compose-check ## Build the GLIM Jetson Docker image (using cache)
	@echo "$(BLUE)[INFO] Building GLIM Jetson image (using cache)...$(NC)"
	docker build --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

run: compose-check x11-setup gpu-check ## Run GLIM Jetson container with full GPU acceleration
	@echo "$(BLUE)[INFO] Starting GLIM Jetson container with GPU acceleration...$(NC)"
	docker compose --profile default up -d glim-jetson

attach: compose-check ## Attach to running GLIM Jetson container
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
		source install/setup.bash; \
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

stop: compose-check ## Stop the GLIM Jetson container
	@echo "$(BLUE)[INFO] Stopping GLIM Jetson container...$(NC)"
	docker stop $(CONTAINER_NAME) >/dev/null 2>&1 || true;
	@echo "$(GREEN)[SUCCESS] Container stopped$(NC)";

create_volume: compose-check ## Create persistent Docker volume for GLIM Jetson
	@echo "$(BLUE)[INFO] Creating persistent volume for GLIM Jetson...$(NC)"
	@if ! docker volume ls --format "{{.Name}}" | grep -q "^nav_humble_jetson_home_volume$$"; then \
		docker volume create nav_humble_jetson_home_volume; \
		echo "$(GREEN)[SUCCESS] Volume created: nav_humble_jetson_home_volume$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Volume 'nav_humble_jetson_home_volume' already exists$(NC)"; \
	fi
	@echo "$(GREEN)[SUCCESS] Persistent volume created$(NC)"

remove_volume: compose-check ## Remove persistent Docker volume for GLIM Jetson
	@echo "$(BLUE)[INFO] Removing persistent volume for GLIM Jetson...$(NC)"
	@if docker volume ls --format "{{.Name}}" | grep -q "^nav_humble_jetson_home_volume$$"; then \
		docker volume rm nav_humble_jetson_home_volume; \
		echo "$(GREEN)[SUCCESS] Volume 'nav_humble_jetson_home_volume' removed$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Volume 'nav_humble_jetson_home_volume' not found$(NC)"; \
	fi

remove_container: compose-check ## Remove the GLIM Jetson container
	@echo "$(BLUE)[INFO] Removing GLIM Jetson container...$(NC)"
	@if docker ps -a --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker rm -f $(CONTAINER_NAME); \
		echo "$(GREEN)[SUCCESS] Container '$(CONTAINER_NAME)' removed$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] Container '$(CONTAINER_NAME)' not found$(NC)"; \
	fi

status: compose-check ## Show status of GLIM Jetson containers and volumes
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

logs: compose-check ## Show logs from the GLIM Jetson container
	@echo "$(BLUE)[INFO] Showing GLIM Jetson container logs...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker compose logs -f; \
	else \
		echo "$(YELLOW)[INFO] Container is not running$(NC)"; \
	fi