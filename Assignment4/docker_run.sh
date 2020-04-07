if [ "$1" = "run" ]; then
    docker run -it --net host --rm -v "${PWD}/catkin_ws:/catkin_ws" --name sdc osrf/ros:kinetic-desktop-full
else
    docker exec -it sdc bash
fi
