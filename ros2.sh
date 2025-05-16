#!/bin/sh

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
#docker run -it --rm -e XAUTHORITY=${XAUTH} -e DISPLAY=$DISPLAY -e QT_GRAPHICSSYSTEM=native -v $XSOCK:$XSOCK:rw -v $XAUTH:$XAUTH:rw osrf/ros:iron-desktop-full bash
docker run -it --rm --mount type=bind,consistency=cached,source=/dev/dri,target=/dev/dri --mount type=bind,source=/home/matstan/bakalarska_praca_ros2/ros2_ws,target=/ros2_ws \
	-e XAUTHORITY=${XAUTH} -e DISPLAY=$DISPLAY -e QT_GRAPHICSSYSTEM=native -v $XSOCK:$XSOCK:rw -v $XAUTH:$XAUTH:rw osrf/ros:jazzy-desktop-full bash
#	--mount type=bind,source=/home/mvagac/ros2_ws-libs,target=/opt/ros/iron/lib/python3.10/site-packages/ \

# pri problemoch s grafikou je este potrebne v shelli dockeru spustit toto: export LIBGL_ALWAYS_SOFTWARE=1


