FROM osrf/ros:jazzy-desktop-full AS deps

WORKDIR /root/ros2_ws

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/jazzy/setup.bash
RUN apt update \
    && apt --no-install-recommends install -y \
    ros-jazzy-smach ros-jazzy-smach-ros \
    python3 python3-pip \
    python3-colcon-common-extensions

COPY ./../yasmin /root/ros2_ws/src
RUN rosdep install --from-paths src --ignore-src -r -y

# WORKDIR /root/ros2_ws/src
# RUN pip3 install -r requirements.txt

WORKDIR /root/ros2_ws

RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --packages-skip yasmin_demos

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/jazzy/" >> ~/.bashrc