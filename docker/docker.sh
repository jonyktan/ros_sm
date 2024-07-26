# Script to launch development docker container with GPU & GUI support
docker run -it --rm \
-v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg -e DISPLAY=:0 -e WAYLAND_DISPLAY=wayland-0 -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir -e PULSE_SERVER=/mnt/wslg/PulseServer --gpus=all \
-v $PWD/src:/root/ros2_ws/src \
--name ros_sm \
ros_sm:jazzy