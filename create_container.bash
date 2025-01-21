#!/bin/bash

# Variables for docker run
IMAGE_NAME=rescue_major
CONTAINER_NAME=rescue_major

DOCKER_COMMAND="docker run"

xhost +

$DOCKER_COMMAND -it -d\
    --device /dev/ttyUSB0 \
    --network=host\
    --privileged \
    -v /dev:/dev \
    -v "$PWD/ws:/ws/" \
    --name=$CONTAINER_NAME\
    $IMAGE_NAME\
    bash