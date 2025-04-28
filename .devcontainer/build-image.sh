#!/bin/bash

BASE_IMAGE="rwthika/ros2:jazzy" \
COMMAND="bash" \
IMAGE="helmoro:latest" \
./docker/docker-ros/scripts/build.sh