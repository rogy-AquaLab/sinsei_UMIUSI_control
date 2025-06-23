ARG ros_distro=jazzy

FROM osrf/ros:${ros_distro}-desktop

RUN apt-get update && apt-get install -y \
    git \
    clangd-18 \
    clang-format-18 \
    clang-tidy-18 \
    && rm -rf /var/lib/apt/lists/*

RUN echo "%ubuntu ALL=(ALL:ALL) NOPASSWD: ALL" > /etc/sudoers.d/ubuntu

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/ubuntu/.bashrc
