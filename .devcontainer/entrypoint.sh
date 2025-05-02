#!/bin/bash
set +e

# Source environment variables
source /opt/ros/jazzy/setup.bash
source /home/ws/install/setup.bash
#source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# Execute the provided command
exec "$@"
