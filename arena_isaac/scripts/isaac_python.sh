#!/usr/bin/env bash

# The name of your running container
CONTAINER_NAME="isaac-sim"

echo "Forwarding command to Isaac Sim 5.1.0 Container: $@"

# Use docker exec to run the command INSIDE the container
# -i (interactive) -t (tty) allows you to see the output in real-time
docker exec -it $CONTAINER_NAME /bin/bash -ic "source ~/.bashrc && export ROS_DOMAIN_ID=1 && /isaac-sim/python.sh $@"
