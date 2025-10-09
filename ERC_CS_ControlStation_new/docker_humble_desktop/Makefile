# Control Station Docker Compose Makefile
# Provides convenient commands for managing the Control Station

.PHONY: help up down restart build rebuild logs shell status clean attach x11-setup docker-check dev stop start health backup restore

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
COMPOSE_FILE = compose.yaml
SERVICE_NAME = cs
PARENT_DIR = $(shell dirname $(PWD))

# Export environment variables for Docker Compose
export XAUTH
export PARENT_DIR
export DISPLAY

help: ## Show this help message
	@echo "$(BLUE)Control Station Docker Compose Commands$(NC)"
	@echo ""
	@echo "$(GREEN)Available commands:$(NC)"
	@awk 'BEGIN {FS = ":.*##"} /^[a-zA-Z_-]+:.*##/ { printf "  $(YELLOW)%-15s$(NC) %s\n", $$1, $$2 }' $(MAKEFILE_LIST)
	@echo ""
	@echo "$(BLUE)Examples:$(NC)"
	@echo "  make up          # Start the Control Station"
	@echo "  make logs        # View logs"
	@echo "  make shell       # Open shell in container"
	@echo ""

docker-check: ## Check if Docker is running
	@if ! docker info > /dev/null 2>&1; then \
		echo "$(RED)[ERROR] Docker is not running. Please start Docker and try again.$(NC)"; \
		exit 1; \
	fi
	@if ! command -v docker compose > /dev/null 2>&1; then \
		echo "$(RED)[ERROR] Docker Compose is not available. Please install Docker Compose and try again.$(NC)"; \
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
		xauth_list=$$(xauth nlist "$$DISPLAY" 2>/dev/null | tail -n 1 | sed -e 's/^..../ffff/' || true); \
		if [ -n "$$xauth_list" ]; then \
			echo "$$xauth_list" | xauth -f "$(XAUTH)" nmerge -; \
			echo "$(GREEN)[SUCCESS] X authority data merged successfully$(NC)"; \
		else \
			touch "$(XAUTH)"; \
			echo "$(YELLOW)[WARNING] Created empty X authority file (no X session found)$(NC)"; \
		fi; \
		chmod a+r "$(XAUTH)"; \
	fi
	@echo "$(BLUE)[INFO] Environment variables set:$(NC)"
	@echo "$(BLUE)[INFO]   XAUTH=$(XAUTH)$(NC)"
	@echo "$(BLUE)[INFO]   PARENT_DIR=$(PARENT_DIR)$(NC)"
	@echo "$(BLUE)[INFO]   DISPLAY=$$DISPLAY$(NC)"

up: docker-check x11-setup ## Start the Control Station services
	@echo "$(BLUE)[INFO] Starting Control Station services...$(NC)"
	docker compose -f $(COMPOSE_FILE) up --remove-orphans

down: docker-check ## Stop the Control Station services
	@echo "$(BLUE)[INFO] Stopping Control Station services...$(NC)"
	docker compose -f $(COMPOSE_FILE) down

restart: docker-check x11-setup ## Restart the Control Station services
	@echo "$(BLUE)[INFO] Restarting Control Station services...$(NC)"
	docker compose -f $(COMPOSE_FILE) down
	docker compose -f $(COMPOSE_FILE) up --remove-orphans

build: docker-check ## Build the Control Station Docker image
	@echo "$(BLUE)[INFO] Building Control Station image...$(NC)"
	docker compose -f $(COMPOSE_FILE) build --no-cache

rebuild: docker-check x11-setup ## Rebuild and restart the Control Station
	@echo "$(BLUE)[INFO] Rebuilding and starting Control Station...$(NC)"
	docker compose -f $(COMPOSE_FILE) down
	docker compose -f $(COMPOSE_FILE) build --no-cache
	docker compose -f $(COMPOSE_FILE) up --remove-orphans

logs: docker-check ## Show Control Station logs
	@echo "$(BLUE)[INFO] Showing Control Station logs...$(NC)"
	docker compose -f $(COMPOSE_FILE) logs -f $(SERVICE_NAME)

shell: docker-check ## Open a shell in the Control Station container
	@echo "$(BLUE)[INFO] Opening shell in Control Station container...$(NC)"
	docker compose -f $(COMPOSE_FILE) exec $(SERVICE_NAME) /bin/bash

attach: docker-check ## Attach to running Control Station container with full environment
	@echo "$(BLUE)[INFO] Attaching to Control Station container...$(NC)"
	@if ! docker ps --format "table {{.Names}}" | grep -q "^cs_humble_desktop$$"; then \
		echo "$(RED)❌ Control Station container 'cs_humble_desktop' is not running$(NC)"; \
		echo ""; \
		echo "To start the Control Station, run:"; \
		echo "  make up"; \
		echo ""; \
		exit 1; \
	fi
	@echo "🔗 Attaching to Control Station container..."
	@echo ""
	docker compose -f $(COMPOSE_FILE) exec $(SERVICE_NAME) /bin/bash -c " \
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
		echo ''; \
		/bin/bash"

status: docker-check ## Show status of Control Station services
	@echo "$(BLUE)[INFO] Control Station status:$(NC)"
	docker compose -f $(COMPOSE_FILE) ps

clean: docker-check ## Clean up containers, volumes, and images
	@echo "$(YELLOW)Cleaning up Control Station resources...$(NC)"
	@echo "This will remove containers, volumes, and images. Continue? [y/N]" && read ans && [ $${ans:-N} = y ]
	docker compose -f $(COMPOSE_FILE) down -v --rmi all
	docker volume prune -f
	@echo "$(GREEN)Cleanup complete!$(NC)"

dev: docker-check x11-setup ## Start in development mode (rebuild if needed)
	@echo "$(BLUE)[INFO] Starting Control Station in development mode...$(NC)"
	docker compose -f $(COMPOSE_FILE) up --build

stop: down ## Alias for down

start: up ## Alias for up

# Health check
health: docker-check ## Check health of Control Station services
	@echo "$(BLUE)[INFO] Checking Control Station health...$(NC)"
	@if docker compose -f $(COMPOSE_FILE) ps | grep -q "cs_humble_desktop"; then \
		echo "$(GREEN)✓ Control Station is running$(NC)"; \
		docker compose -f $(COMPOSE_FILE) exec $(SERVICE_NAME) curl -f http://localhost:3000 > /dev/null 2>&1 && \
		echo "$(GREEN)✓ Web interface is accessible$(NC)" || \
		echo "$(YELLOW)⚠ Web interface not accessible$(NC)"; \
	else \
		echo "$(RED)✗ Control Station is not running$(NC)"; \
	fi

# Backup and restore
backup: docker-check ## Backup Control Station volumes
	@echo "$(BLUE)[INFO] Creating backup of Control Station volumes...$(NC)"
	@mkdir -p backups
	@timestamp=$$(date +%Y%m%d_%H%M%S); \
	docker run --rm \
		-v cs_humble_desktop_home_volume:/data \
		-v $(PWD)/backups:/backup \
		alpine tar czf /backup/cs_home_$$timestamp.tar.gz -C /data .; \
	echo "$(GREEN)Backup created: backups/cs_home_$$timestamp.tar.gz$(NC)"

restore: docker-check ## Restore Control Station volumes from backup
	@echo "$(BLUE)Available backups:$(NC)"
	@ls -la backups/*.tar.gz 2>/dev/null || echo "No backups found"
	@echo "Enter backup filename to restore:"
	@read backup_file; \
	if [ -f "backups/$$backup_file" ]; then \
		echo "$(YELLOW)Restoring from backups/$$backup_file...$(NC)"; \
		docker run --rm \
			-v cs_humble_desktop_home_volume:/data \
			-v $(PWD)/backups:/backup \
			alpine tar xzf /backup/$$backup_file -C /data; \
		echo "$(GREEN)Restore complete!$(NC)"; \
	else \
		echo "$(RED)Backup file not found!$(NC)"; \
		exit 1; \
	fi

run: docker-check x11-setup ## Run Control Station with interactive shell (equivalent to run.sh)
	@echo "$(BLUE)[INFO] Starting Control Station with interactive shell...$(NC)"
	@echo "Verifying file contents:"
	@file $(XAUTH)
	@echo "--> It should say \"X11 Xauthority data\"."
	@echo ""
	@echo "Permissions:"
	@ls -FAlh $(XAUTH)
	@echo ""
	@echo "Running docker compose with interactive shell..."
	docker compose -f $(COMPOSE_FILE) run --rm --service-ports $(SERVICE_NAME) /bin/bash

run_cs: docker-check x11-setup ## Run Control Station launch script (equivalent to run_cs.sh)
	@echo "$(BLUE)[INFO] Starting Control Station launch script...$(NC)"
	@echo "Verifying file contents:"
	@file $(XAUTH)
	@echo "--> It should say \"X11 Xauthority data\"."
	@echo ""
	@echo "Permissions:"
	@ls -FAlh $(XAUTH)
	@echo ""
	@echo "Running docker compose with launch script..."
	docker compose -f $(COMPOSE_FILE) up --remove-orphans $(SERVICE_NAME)