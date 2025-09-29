# intrinsic_sdk_cmake build base + user code built and setup to run
ARG REPOSITORY=ghcr.io/intrinsic-ai
ARG TAG=latest
FROM ${REPOSITORY}/intrinsic_sdk_cmake:${TAG} AS source

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
# The relative path for the manifest textproto
ARG SKILL_MANIFEST_TEXTPROTO
# The relative path for the file descriptor set for the manifest protos
ARG SKILL_MANIFEST_FILE_DESCRIPTOR_SET

# Colcon workspace for building the user's packages.
# Note use the _workspace suffix to prevent accidental collision with other
# things in opt like ros or intrinsic_sdk_cmake.
ENV SKILL_WORKSPACE=/opt/${SKILL_NAME}_workspace

# Add the user's code to the container.
ADD ./ $SKILL_WORKSPACE/src

# build stage: build dependencies + build the packages
FROM source AS build

ARG SKILL_PACKAGE

# Install build and run dependencies for the user's packages.
RUN . /opt/intrinsic/intrinsic_sdk_cmake/install/setup.sh \
    && set -x \
    && apt-get update \
    && (rosdep init || true) \
    && rosdep update \
    && cd $SKILL_WORKSPACE \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --default-yes \
        --dependency-types exec \
    && dpkg --get-selections > /user_exec_apt_packages.txt \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --default-yes \
    && rm -rf /var/lib/apt/lists/*

# Build the user's packages.
RUN . /opt/intrinsic/intrinsic_sdk_cmake/install/setup.sh \
    && set -x \
    && cd $SKILL_WORKSPACE \
    && colcon build \
        --cmake-args -DBUILD_TESTING=ON \
        --event-handlers console_direct+ console_stderr- \
        --merge-install \
        --executor=sequential \
        --packages-up-to $SKILL_PACKAGE

# exec_depends stage: capture just the exec depends using the source
FROM ${REPOSITORY}/intrinsic_sdk_cmake_run:${TAG} AS exec_depends

ARG SKILL_NAME
ENV SKILL_WORKSPACE=/opt/${SKILL_NAME}_workspace

COPY --from=source \
    $SKILL_WORKSPACE \
    $SKILL_WORKSPACE

RUN . /opt/intrinsic/intrinsic_sdk_cmake/install/setup.sh \
    && set -x \
    && apt-get update \
    && (rosdep init || true) \
    && rosdep update \
    && cd $SKILL_WORKSPACE \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --default-yes \
        --dependency-types exec \
    && dpkg --get-selections > /user_exec_apt_packages.txt \
    && rm -rf /var/lib/apt/lists/*

# run stage: install exec dependencies + copy install artifacts from build stage
FROM ${REPOSITORY}/intrinsic_sdk_cmake_run:${TAG} AS run

# Re-declare the arguments for this stage
ARG SKILL_EXECUTABLE
ARG SKILL_CONFIG
ARG SKILL_ASSET_ID_ORG
ARG SKILL_MANIFEST_TEXTPROTO
ARG SKILL_MANIFEST_FILE_DESCRIPTOR_SET

ARG SKILL_NAME
ENV SKILL_WORKSPACE=/opt/${SKILL_NAME}_workspace

# Install run dependencies for user's packages.
COPY --from=exec_depends \
    /user_exec_apt_packages.txt \
    /user_exec_apt_packages.txt
RUN set -x \
    && apt-get update \
    && apt-cache dumpavail | dpkg --merge-avail \
    && dpkg --set-selections < /user_exec_apt_packages.txt \
    && apt-get dselect-upgrade -y \
    && rm -rf /var/lib/apt/lists/*

# Copy build artifacts from user's packages.
COPY --from=build $SKILL_WORKSPACE/install $SKILL_WORKSPACE/install

# Ensure skill executable and config file exist.
ENV SKILL_EXECUTABLE_ABS=$SKILL_WORKSPACE/install/$SKILL_EXECUTABLE
RUN ls $SKILL_EXECUTABLE_ABS \
    || (echo "Skill executable does not exist '$SKILL_EXECUTABLE_ABS'" \
        && false)
ENV SKILL_CONFIG_ABS=$SKILL_WORKSPACE/install/$SKILL_CONFIG
RUN ls $SKILL_CONFIG_ABS \
    || (echo "Skill executable does not exist '$SKILL_CONFIG_ABS'" \
        && false)
ENV SKILL_MANIFEST_TEXTPROTO_ABS=${SKILL_WORKSPACE}/install/${SKILL_MANIFEST_TEXTPROTO}
RUN ls $SKILL_MANIFEST_TEXTPROTO_ABS \
    || (echo "Skill manifest textproto does not exist '$SKILL_MANIFEST_TEXTPROTO_ABS'" \
        && false)
ENV SKILL_MANIFEST_FILE_DESCRIPTOR_SET_ABS=${SKILL_WORKSPACE}/install/${SKILL_MANIFEST_FILE_DESCRIPTOR_SET}
RUN ls $SKILL_MANIFEST_FILE_DESCRIPTOR_SET_ABS \
    || (echo "Skill manifest file descriptor set does not exist '$SKILL_MANIFEST_FILE_DESCRIPTOR_SET_ABS'" \
        && false)

# Link skill executable and skill config file into well known locations needed by Flowstate.
# Also ensure the user's workspace is sourced so the skill main can be run correctly.
RUN set -x \
    && mkdir -p /skills \
    && ln -sf $SKILL_EXECUTABLE_ABS /skills/skill_service \
    && ln -sf $SKILL_CONFIG_ABS /skills/skill_service_config.proto.bin \
    && ln -sf $SKILL_MANIFEST_TEXTPROTO_ABS /skills/skill_manifest.textproto \
    && ln -sf $SKILL_MANIFEST_FILE_DESCRIPTOR_SET_ABS /skills/skill_protos.desc \
    && sed --in-place \
        --expression '$isource "$SKILL_WORKSPACE/install/setup.bash"' \
        /ros_entrypoint.sh

# Set some labels used by Flowstate.
LABEL "ai.intrinsic.asset-id"="${SKILL_ASSET_ID_ORG}.${SKILL_NAME}"
LABEL "ai.intrinsic.skill-image-name"="${SKILL_NAME}"

# Execute the skill main by default, but note that Flowstate will likely override this statement.
CMD [ \
    "/skills/skill_service", \
    "--skill_service_config_filename=/skills/skill_service_config.proto.bin"]
