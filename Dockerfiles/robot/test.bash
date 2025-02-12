#!/bin/bash

docker stop rescue_major_robot
docker rm rescue_major_robot

# Variables for docker run
IMAGE_NAME=rescue_major_robot
CONTAINER_NAME=rescue_major_robot

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

docker exec -it rescue_major_robot /bin/bash
cd ws
