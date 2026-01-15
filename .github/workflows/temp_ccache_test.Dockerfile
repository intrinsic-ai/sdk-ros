FROM ros:jazzy

ENV CCACHE_DIR=/ccache

RUN apt update ; apt install -yq \
    python3-rosinstall-generator \
    ccache

ENV PATH="/usr/lib/ccache:/usr/local/opt/ccache/libexec:$PATH"

RUN --mount=type=cache,target=/ccache/ ccache -s
RUN --mount=type=cache,target=/ccache/ \
    mkdir -p src && \
    rosinstall_generator rcpputils --rosdistro jazzy rcpputils.rosinstall && \
    vcs import src < rcpputils.rosinstall && \
    colcon build
RUN --mount=type=cache,target=/ccache/ ccache -s
