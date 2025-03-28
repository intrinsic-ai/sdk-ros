# intrinsic_sdk_cmake build base + user code built and setup to run
ARG TAG=latest
FROM ghcr.io/intrinsic-ai/intrinsic_sdk_cmake:${TAG} as source

# The name of the skill.
ARG SKILL_NAME
# The name of the cmake project/ament package that contains the skill executable.
ARG SKILL_PACKAGE
# The relative path for the skill executable.
ARG SKILL_EXECUTABLE
# The relative path for the skill config.
ARG SKILL_CONFIG
# The reverse domain name for the organization in the asset-id label.
ARG SKILL_ASSET_ID_ORG=com.example

# Colcon workspace for building the user's packages.
# Note use the _workspace suffix to prevent accidental collision with other
# things in opt like ros or intrinsic_sdk_cmake.
ENV SKILL_WORKSPACE=/opt/${SKILL_NAME}_workspace

# Add the user's code to the container.
ADD ./ $SKILL_WORKSPACE/src

FROM source as build

ARG SKILL_PACKAGE

# Install build and run dependencies for the user's packages.
RUN . /opt/intrinsic/intrinsic_sdk_cmake/install/setup.sh \
    && apt-get update \
    && rosdep init || true \
    && rosdep update \
    && cd $SKILL_WORKSPACE \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --default-yes \
        --skip-keys intrinsic_sdk_cmake \
    && rm -rf /var/lib/apt/lists/*

# Build the user's packages.
RUN . /opt/intrinsic/intrinsic_sdk_cmake/install/setup.sh \
    && cd $SKILL_WORKSPACE \
    && colcon build \
        --cmake-args -DBUILD_TESTING=ON \
        --event-handlers console_direct+ console_stderr- \
        --merge-install \
        --executor=sequential \
        --packages-up-to $SKILL_PACKAGE

FROM source as run

ARG SKILL_EXECUTABLE
ARG SKILL_CONFIG
ARG SKILL_ASSET_ID_ORG

# Install run dependencies for user's packages.
RUN . /opt/intrinsic/intrinsic_sdk_cmake/install/setup.sh \
    && apt-get update \
    && rosdep init || true \
    && rosdep update \
    && cd $SKILL_WORKSPACE \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --default-yes \
        --dependency-types exec \
        --skip-keys intrinsic_sdk_cmake \
    && rm -rf /var/lib/apt/lists/*

COPY --from=build $SKILL_WORKSPACE/install $SKILL_WORKSPACE/install

ENV SKILL_EXECUTABLE_ABS=$SKILL_WORKSPACE/install/$SKILL_EXECUTABLE
RUN ls $SKILL_EXECUTABLE_ABS \
    || (echo "Skill executable does not exist '$SKILL_EXECUTABLE_ABS'" \
        && false)
ENV SKILL_CONFIG_ABS=$SKILL_WORKSPACE/install/$SKILL_CONFIG
RUN ls $SKILL_CONFIG_ABS \
    || (echo "Skill executable does not exist '$SKILL_CONFIG_ABS'" \
        && false)

RUN ln -sf $SKILL_EXECUTABLE_ABS /skill_main && \
    ln -sf $SKILL_CONFIG_ABS /skill_config && \
    sed --in-place \
        --expression '$isource "$SKILL_WORKSPACE/install/setup.bash"' \
        /ros_entrypoint.sh \
    && sed --in-place \
        --expression \
            '5 a/opt/intrinsic/intrinsic_sdk_cmake/install/lib/zenoh_bridge_dds/zenoh_bridge_dds \
             -m client \
             -e tcp/zenoh-router.app-intrinsic-base:7447 \
             --no-multicast-scouting &' \
        /ros_entrypoint.sh

LABEL "ai.intrinsic.asset-id"="${SKILL_ASSET_ID_ORG}.${SKILL_NAME}"
LABEL "ai.intrinsic.skill-image-name"="${SKILL_NAME}"

CMD ["/skill_main", "--skill_service_config_filename=/skill_config"]
