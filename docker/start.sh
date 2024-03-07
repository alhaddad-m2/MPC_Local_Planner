#!/bin/bash

docker_dir=$(dirname $0)

docker run -it -d --rm \
    --ipc host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --privileged \
    --net "host" \
    --name mpc \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /etc/timezone:/etc/timezone:ro \
    -v /etc/localtime:/etc/localtime:ro \
    -v $(realpath $docker_dir)/..:/home/docker_mpc/catkin_ws:rw \
    mpc:latest
