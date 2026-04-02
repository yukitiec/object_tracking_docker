SHELL := /bin/bash
COMPOSE := docker compose -f docker/docker-compose.yml
SERVICE := ros2_tracker.PHONY: build up down restart shell test clean precommit format

build:
	$(COMPOSE) build

up:
	$(COMPOSE) up

down:
	$(COMPOSE) down

restart: down up

shell:
	$(COMPOSE) run --rm $(SERVICE) bash

test:
	$(COMPOSE) run --rm $(SERVICE) bash -lc "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && ctest --test-dir /ros_ws/build/tracker_pkg --output-on-failure"

clean:
	$(COMPOSE) run --rm $(SERVICE) bash -lc "rm -rf /ros_ws/build /ros_ws/install /ros_ws/log"

precommit:
	pre-commit run --all-files

format:
	pre-commit run clang-format --all-files