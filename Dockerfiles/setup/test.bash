#!/bin/bash

docker stop rescue_major_setup
docker rm rescue_major_setup

# Variables for docker run
IMAGE_NAME=rescue_major_setup
CONTAINER_NAME=rescue_major_setup

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

docker exec -it rescue_major_setup /bin/bash
cd ws
