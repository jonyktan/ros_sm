FROM osrf/ros:jazzy-desktop-full

RUN apt update && apt install ros-jazzy-smach ros-jazzy-smach-ros -y

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

RUN mkdir -p ~/ros2_ws/src
WORKDIR /root/ros2_ws

