# ERC Control Station Desktop Docker Makefile
# Provides convenient commands for managing the Control Station containers

.PHONY: help build run run-cs run-mac attach stop clean docker-check x11-setup compose-up compose-down

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
PARENT_DIR = $(shell dirname $(PWD))

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
	@echo "$(BLUE)[INFO] Setting up X11 forwarding for Control Station...$(NC)"
	@xauth_list=$$(xauth nlist :0 2>/dev/null | tail -n 1 | sed -e 's/^..../ffff/'); \
	if [ ! -f "$(XAUTH)" ]; then \
		if [ ! -z "$$xauth_list" ]; then \
			echo $$xauth_list | xauth -f "$(XAUTH)" nmerge -; \
		else \
			touch "$(XAUTH)"; \
		fi; \
		chmod a+r "$(XAUTH)"; \
	fi
	@echo "$(BLUE)[INFO] Verifying file contents:$(NC)"
	@file $(XAUTH) || echo "$(YELLOW)[WARNING] Could not verify X authority file$(NC)"
	@echo "$(BLUE)[INFO] Permissions:$(NC)"
	@ls -FAlh $(XAUTH)

build: docker-check ## Build the Control Station Docker image
	@echo "$(BLUE)[INFO] Building Control Station image...$(NC)"
	docker build --progress=plain -t $(IMAGE_NAME) -f Dockerfile ..

build-quick: docker-check ## Build the Control Station Docker image (using cache)
	@echo "$(BLUE)[INFO] Building Control Station image (using cache)...$(NC)"
	docker build -t $(IMAGE_NAME) -f Dockerfile ..

run: docker-check x11-setup ## Run Control Station container with X11 forwarding
	@echo "$(BLUE)[INFO] Starting Control Station container...$(NC)"
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
		-v cs_humble_desktop_home_volume:/home/$(USERNAME) \
		$(IMAGE_NAME) \
		/bin/bash -c "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
			echo '🚀 Control Station container ready!'; \
			echo 'Web interfaces available at:'; \
			echo '  - Main Interface: http://localhost:3000'; \
			echo '  - ROSBridge: http://localhost:9090'; \
			echo ''; \
			/bin/bash"

run-cs: docker-check x11-setup ## Run Control Station with launch script execution
	@echo "$(BLUE)[INFO] Starting Control Station with launch script...$(NC)"
	/usr/bin/docker run \
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
		-v cs_humble_desktop_home_volume:/home/$(USERNAME) \
		-w /home/$(USERNAME)/dev_ws/src \
		$(IMAGE_NAME) \
		/bin/bash ./launch.sh

run-mac: docker-check x11-setup ## Run Control Station using Docker Compose (optimized for macOS)
	@echo "$(BLUE)[INFO] Starting Control Station with Docker Compose (macOS mode)...$(NC)"
	@echo "$(YELLOW)[INFO] Web interfaces will be available at:$(NC)"
	@echo "  - Main Interface: http://localhost:3000"
	@echo "  - ROSBridge: http://localhost:9090"
	@echo ""
	docker compose up

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
		source install/setup.bash 2>/dev/null || true; \
		echo '🚀 Welcome to Control Station container!'; \
		echo 'Environment configured with:'; \
		echo '  - ROS 2 Humble'; \
		echo '  - CycloneDDS middleware'; \
		echo '  - X11 forwarding'; \
		echo '  - Web interfaces (ports 3000, 9090)'; \
		echo '  - Control Station stack'; \
		echo ''; \
		/bin/bash"

compose-up: docker-check x11-setup ## Start Control Station using Docker Compose
	@echo "$(BLUE)[INFO] Starting Control Station with Docker Compose...$(NC)"
	docker compose up -d
	@echo "$(GREEN)[SUCCESS] Control Station started in background$(NC)"
	@echo "Web interfaces available at:"
	@echo "  - Main Interface: http://localhost:3000"
	@echo "  - ROSBridge: http://localhost:9090"

compose-down: docker-check ## Stop Control Station Docker Compose services
	@echo "$(BLUE)[INFO] Stopping Control Station Docker Compose services...$(NC)"
	docker compose down
	@echo "$(GREEN)[SUCCESS] Services stopped$(NC)"

stop: docker-check ## Stop the Control Station container
	@echo "$(BLUE)[INFO] Stopping Control Station container...$(NC)"
	@containers_stopped=0; \
	if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker stop $(CONTAINER_NAME); \
		containers_stopped=1; \
	fi; \
	if docker compose ps --services --filter "status=running" 2>/dev/null | grep -q "cs"; then \
		docker compose down; \
		containers_stopped=1; \
	fi; \
	if [ $$containers_stopped -eq 1 ]; then \
		echo "$(GREEN)[SUCCESS] Container(s) stopped$(NC)"; \
	else \
		echo "$(YELLOW)[INFO] No containers were running$(NC)"; \
	fi

clean: docker-check ## Clean up containers, volumes, and images
	@echo "$(YELLOW)Cleaning up Control Station resources...$(NC)"
	@echo "This will remove containers, volumes, and images. Continue? [y/N]" && read ans && [ $${ans:-N} = y ]
	@if docker ps -a --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker rm -f $(CONTAINER_NAME); \
	fi
	@docker compose down --volumes 2>/dev/null || true
	@if docker volume ls -q | grep -q "cs_humble_desktop_home_volume"; then \
		docker volume rm cs_humble_desktop_home_volume; \
	fi
	@if docker images -q $(IMAGE_NAME) | head -1 | xargs -r docker rmi; then \
		echo "$(GREEN)Cleanup complete!$(NC)"; \
	fi

status: docker-check ## Show status of Control Station containers and services
	@echo "$(BLUE)[INFO] Control Station status:$(NC)"
	@echo ""
	@echo "$(GREEN)Direct Containers:$(NC)"
	@docker ps -a --filter "name=$(CONTAINER_NAME)" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" || echo "No direct containers found"
	@echo ""
	@echo "$(GREEN)Docker Compose Services:$(NC)"
	@docker compose ps 2>/dev/null || echo "No compose services found"
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
		docker logs -f $(CONTAINER_NAME); \
	elif docker compose ps --services 2>/dev/null | grep -q "cs"; then \
		docker compose logs -f cs; \
	else \
		echo "$(YELLOW)[INFO] No containers are running$(NC)"; \
	fi

# Control Station specific commands
cs-test: ## Run Control Station tests (requires running container)
	@echo "$(BLUE)[INFO] Running Control Station tests...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)[ERROR] Container $(CONTAINER_NAME) is not running$(NC)"; \
		echo "Start it first with: make run"; \
		exit 1; \
	fi
	docker exec $(CONTAINER_NAME) /bin/bash -c " \
		cd /home/$(USERNAME)/dev_ws; \
		source install/setup.bash 2>/dev/null || true; \
		export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
		echo 'Running Control Station tests...'; \
		colcon test --packages-select cs_pkg || echo 'Tests completed with warnings'"

launch-cs: ## Launch Control Station stack in running container
	@echo "$(BLUE)[INFO] Launching Control Station stack...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "$(RED)[ERROR] Container $(CONTAINER_NAME) is not running$(NC)"; \
		echo "Start it first with: make run"; \
		exit 1; \
	fi
	docker exec $(CONTAINER_NAME) /bin/bash -c " \
		cd /home/$(USERNAME)/dev_ws/src; \
		source ../install/setup.bash 2>/dev/null || true; \
		export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
		echo 'Launching Control Station...'; \
		/bin/bash ./launch.sh || echo 'Launch script not found or failed'"

web-status: ## Check web interface status and accessibility
	@echo "$(BLUE)[INFO] Checking Control Station web interfaces...$(NC)"
	@echo ""
	@echo "$(GREEN)Port Status:$(NC)"
	@if nc -z localhost 3000 2>/dev/null; then \
		echo "  ✅ Main Interface (port 3000): http://localhost:3000 - ACTIVE"; \
	else \
		echo "  ❌ Main Interface (port 3000): http://localhost:3000 - INACTIVE"; \
	fi
	@if nc -z localhost 9090 2>/dev/null; then \
		echo "  ✅ ROSBridge (port 9090): http://localhost:9090 - ACTIVE"; \
	else \
		echo "  ❌ ROSBridge (port 9090): http://localhost:9090 - INACTIVE"; \
	fi
	@echo ""
	@echo "$(GREEN)Network Interfaces:$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
		docker exec $(CONTAINER_NAME) /bin/bash -c " \
			echo 'Container network status:'; \
			ip addr show | grep -E '(inet|UP|DOWN)' | grep -v 127.0.0.1 || echo 'Network info not available'"; \
	else \
		echo "$(YELLOW)Container not running$(NC)"; \
	fi

dev-shell: docker-check x11-setup ## Run Control Station development shell without auto-setup
	@echo "$(BLUE)[INFO] Starting Control Station development shell...$(NC)"
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
		-v cs_humble_desktop_home_volume:/home/$(USERNAME) \
		$(IMAGE_NAME) \
		/bin/bash -c " \
			export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
			source install/setup.bash 2>/dev/null || true; \
			echo '🔧 Control Station development shell ready!'; \
			/bin/bash"

config-check: ## Validate configuration files and Docker setup
	@echo "$(BLUE)[INFO] Validating Control Station configuration...$(NC)"
	@echo ""
	@echo "$(GREEN)Docker Compose Configuration:$(NC)"
	@if [ -f $(PWD)/compose.yaml ]; then \
		echo "✅ compose.yaml exists"; \
		echo "Validating compose file..."; \
		docker compose config >/dev/null 2>&1 && echo "✅ compose.yaml is valid" || echo "$(RED)❌ compose.yaml has errors$(NC)"; \
	else \
		echo "$(RED)❌ compose.yaml missing$(NC)"; \
	fi
	@echo ""
	@echo "$(GREEN)CycloneDDS Configuration:$(NC)"
	@if [ -f $(PWD)/cyclonedds.xml ]; then \
		echo "✅ cyclonedds.xml exists"; \
		echo "File size: $$(wc -c < $(PWD)/cyclonedds.xml) bytes"; \
	else \
		echo "$(YELLOW)⚠️  cyclonedds.xml not found (optional)$(NC)"; \
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
	@echo "$(GREEN)X11 Setup:$(NC)"
	@if [ -f "$(XAUTH)" ]; then \
		echo "✅ X11 authority file exists: $(XAUTH)"; \
	else \
		echo "$(YELLOW)⚠️  X11 authority file not found (will be created)$(NC)"; \
	fi

compose-logs: ## Show Docker Compose logs
	@echo "$(BLUE)[INFO] Showing Docker Compose logs...$(NC)"
	docker compose logs -f

compose-shell: ## Access shell in Docker Compose service
	@echo "$(BLUE)[INFO] Accessing shell in Docker Compose service...$(NC)"
	@if ! docker compose ps --services --filter "status=running" 2>/dev/null | grep -q "cs"; then \
		echo "$(RED)[ERROR] Docker Compose service 'cs' is not running$(NC)"; \
		echo "Start it first with: make compose-up"; \
		exit 1; \
	fi
	docker compose exec cs /bin/bash -c " \
		export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
		export DISPLAY=unix\$$DISPLAY; \
		export QT_X11_NO_MITSHM=1; \
		export PYTHONPATH=/home/xplore/dev_ws/install/rover_pkg/lib/python3.10/site-packages:/home/xplore/dev_ws/install/custom_msg/local/lib/python3.10/dist-packages:/opt/ros/humble/install/local/lib/python3.10/dist-packages:/opt/ros/humble/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages; \
		source install/setup.bash 2>/dev/null || true; \
		echo '🚀 Welcome to Control Station Compose shell!'; \
		/bin/bash"

restart: ## Restart Control Station services (both direct and compose)
	@echo "$(BLUE)[INFO] Restarting Control Station services...$(NC)"
	@$(MAKE) stop
	@sleep 2
	@echo "$(BLUE)[INFO] Starting services again...$(NC)"
	@$(MAKE) run

port-forward: ## Setup port forwarding for remote access
	@echo "$(BLUE)[INFO] Control Station port forwarding information:$(NC)"
	@echo ""
	@echo "$(GREEN)Local Access:$(NC)"
	@echo "  - Main Interface: http://localhost:3000"
	@echo "  - ROSBridge: http://localhost:9090"
	@echo ""
	@echo "$(GREEN)Remote Access Setup:$(NC)"
	@echo "  For SSH tunneling from remote machine:"
	@echo "  ssh -L 3000:localhost:3000 -L 9090:localhost:9090 user@this-machine"
	@echo ""
	@echo "  Then access from remote browser:"
	@echo "  - Main Interface: http://localhost:3000"
	@echo "  - ROSBridge: http://localhost:9090"