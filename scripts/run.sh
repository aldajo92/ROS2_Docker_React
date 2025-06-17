#!/bin/bash

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"
source ${PROJECT_ROOT}/config_docker.sh
source ${PROJECT_ROOT}/config_local.sh

docker run -it \
  --name=${DOCKER_CONTAINER_NAME} \
  --network ${DOCKER_NETWORK} \
  --volume ${PROJECT_ROOT}/ros2_ws:/ros2_ws \
  --volume ${PROJECT_ROOT}/react_app:/react_app \
  --publish 3000:3000 \
  --publish 9090:9090 \
  --env HOST=0.0.0.0 \
  --env PORT=3000 \
  --rm \
  ${DOCKER_IMAGE_NAME} /bin/bash
