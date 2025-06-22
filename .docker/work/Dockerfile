ARG ros_distro

FROM osrf/ros:${ros_distro}-desktop

RUN apt-get update && apt-get install -y \
    git \
    clangd-18 \
    clang-format-18 \
    clang-tidy-18 \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-launch-pytest \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/ubuntu/.bashrc
