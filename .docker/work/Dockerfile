ARG ros_distro=jazzy

FROM osrf/ros:${ros_distro}-desktop

# Copy scripts temporarily
COPY ../scripts/* /tmp/scripts/
RUN for file in /tmp/scripts/*.sh; do \
    chmod +x "$file"; \
    done

RUN apt-get update && apt-get install -y \
    git \
    clangd-18 \
    clang-format-18 \
    clang-tidy-18 \
    python3-colcon-lcov-result

# Install can-utils
RUN apt-get install -y can-utils

# Install pigpio
RUN /tmp/scripts/install-pigpio.sh

RUN rm -rf /var/lib/apt/lists/*

# Remove temporary scripts
RUN rm -rf /tmp/scripts

RUN echo "%ubuntu ALL=(ALL:ALL) NOPASSWD: ALL" > /etc/sudoers.d/ubuntu

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/ubuntu/.bashrc
