#!/bin/bash

# This script is used to run tests in a Docker container for the helmoro project.
# Start testing environment
docker run --name test_container --net host -v /tmp/.X11-unix:/tmp/.X11-unix -v /var/run/docker.sock:/var/run/docker.sock --detach helmoro tail -f /dev/null

# Run tests inside the container
echo "Start testing..."
    docker exec -e DISPLAY=$DISPLAY -e ROS_DOMAIN_ID=42 test_container bash -c "
    source ./entrypoint.sh &&
    colcon test --executor sequential &&
    colcon test-result --verbose
    "

# Stop and remove the container after testing
docker stop test_container -t 1
docker rm test_container