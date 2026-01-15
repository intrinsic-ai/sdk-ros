FROM ros:jazzy

ENV CCACHE_DIR=/ccache

RUN \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt update; \
    apt install -yq \
        python3-rosinstall-generator \
        ccache

ENV PATH="/usr/lib/ccache:/usr/local/opt/ccache/libexec:$PATH"

RUN --mount=type=cache,target=/ccache/ ccache -s
RUN --mount=type=cache,target=/ccache/ \
    mkdir -p ws/src && \
    cd ws && \
    rosinstall_generator rclcpp --rosdistro jazzy --deps --tar > ./rclcpp.rosinstall && \
    vcs import --input ./rclcpp.rosinstall src && \
    colcon build
RUN --mount=type=cache,target=/ccache/ ccache -s
