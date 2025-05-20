ARG ros_distro

FROM osrf/ros:${ros_distro}-desktop

RUN apt-get update && apt-get install -y \
    git \
    clang-format \
    ros-${ROS_DISTRO}-ament-clang-format \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/ubuntu/.bashrc