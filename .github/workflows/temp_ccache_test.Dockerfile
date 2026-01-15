FROM ros:jazzy

ENV CCACHE_DIR=/ccache
ENV DEBIAN_FRONTEND=noninteractive
ENV RTI_NC_LICENSE_ACCEPTED=yes

RUN \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache && \
    apt update && \
    apt install -yq \
        python3-rosinstall-generator \
        ccache

ENV PATH="/usr/lib/ccache:/usr/local/opt/ccache/libexec:$PATH"

RUN --mount=type=cache,target=/ccache/ ccache -s
RUN \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=cache,target=/ccache/ \
    mkdir -p ws/src && \
    cd ws && \
    rosinstall_generator rclcpp --rosdistro jazzy --deps --tar > ./rclcpp.rosinstall && \
    vcs import --input ./rclcpp.rosinstall src && \
    rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache && \
    apt update && \
    rosdep install --from-paths src --ignore-src --rosdistro jazzy -y && \
    colcon build
RUN --mount=type=cache,target=/ccache/ ccache -s
