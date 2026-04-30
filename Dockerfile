FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    git vim \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /root/ws
CMD ["bash"]