ARG ros_distro=jazzy

FROM osrf/ros:${ros_distro}-desktop
ARG ros_distro
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Copy scripts temporarily
COPY .docker/scripts/* /tmp/scripts/
COPY deps.repos /tmp/deps.repos
COPY package.xml /tmp/package.xml
RUN for file in /tmp/scripts/*.sh; do \
    chmod +x "$file"; \
    done

RUN apt-get update && apt-get install -y \
    git \
    clang \
    clangd-18 \
    clang-format-18 \
    clang-tidy-18 \
    lld \
    lld-18 \
    sccache \
    libgstreamermm-1.0-dev

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
      rosdep init; \
    fi

RUN mkdir -p /home/ubuntu/ros_ws/src/pkg && \
    mv /tmp/package.xml /home/ubuntu/ros_ws/src/pkg/package.xml && \
    vcs import /home/ubuntu/ros_ws/src < /tmp/deps.repos && \
    source /opt/ros/${ros_distro}/setup.bash && \
    rosdep update && \
    rosdep install --from-paths /home/ubuntu/ros_ws/src --ignore-src -y --rosdistro ${ros_distro}

# Install editorconfig-checker
RUN /tmp/scripts/install-ec.sh

RUN rm -rf /var/lib/apt/lists/*

RUN rm -rf /tmp/* /root/.ros

RUN echo "%ubuntu ALL=(ALL:ALL) NOPASSWD: ALL" > /etc/sudoers.d/ubuntu

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/ubuntu/.bashrc
