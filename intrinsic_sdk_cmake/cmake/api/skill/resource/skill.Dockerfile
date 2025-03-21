# intrinsic_sdk_cmake build base + user code built and setup to run
ARG TAG=latest
FROM ghcr.io/intrinsic-dev/intrinsic_sdk_cmake:${TAG} as source

# The name of the skill.
ARG SKILL_NAME
# The name of the cmake project/ament package that contains the skill executable.
ARG SKILL_PACKAGE
# The relative path for the skill executable.
ARG SKILL_EXECUTABLE
# The relative path for the skill config.
ARG SKILL_CONFIG

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

CMD ["$SKILL_EXECUTABLE_ABS" ]
