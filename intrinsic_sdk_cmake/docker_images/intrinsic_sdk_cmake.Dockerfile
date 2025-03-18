# This Dockerfile provides an image with intrinsic_sdk_cmake with its build dependencies.
# Use the intrinsic_sdk_cmake_run image if you only want the runtime dependencies.

ARG TAG=latest
FROM ghcr.io/intrinsic-dev/intrinsic_sdk_cmake_source:${TAG} AS source

# build_export_depends stage: source + rosdep install build_export depends
FROM source AS build_export

# Install build_export-like dependencies for the packages in intrinsic_sdk_ros.
# Exclude intrinsic_sdk_ros and intrinsic_sdk for now.
RUN . /opt/ros/jazzy/setup.sh \
	&& apt-get update \
	&& rosdep init || true \
	&& rosdep update \
	&& cd /opt/intrinsic/intrinsic_sdk_cmake \
	&& touch src/intrinsic_sdk_ros/intrinsic_sdk/COLCON_IGNORE \
	&& touch src/intrinsic_sdk_ros/intrinsic_sdk_ros/COLCON_IGNORE \
	&& rosdep install \
		--from-paths src \
		--ignore-src \
		--default-yes \
		--dependency-types buildtool_export build_export exec \
	&& rm -rf /var/lib/apt/lists/*

# Save installed packages for re-install without source in later stage.
# TODO(wjwwood): it would be nice to get the list of packages from rosdep
#   without installing them, but I couldn't figure out how to do that easily.
RUN dpkg --get-selections > /build_export_apt_packages.txt

# build stage: source + rosdep install build depends + packages up to intrinsic_sdk_cmake built
FROM source AS build

# Install standard dependencies for the packages in intrinsic_sdk_ros.
# Exclude intrinsic_sdk_ros and intrinsic_sdk for now.
RUN . /opt/ros/jazzy/setup.sh \
	&& apt-get update \
	&& rosdep init || true \
	&& rosdep update \
	&& cd /opt/intrinsic/intrinsic_sdk_cmake \
	&& touch src/intrinsic_sdk_ros/intrinsic_sdk/COLCON_IGNORE \
	&& touch src/intrinsic_sdk_ros/intrinsic_sdk_ros/COLCON_IGNORE \
	&& rosdep install \
		--from-paths src \
		--ignore-src \
		--default-yes \
	&& rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/jazzy/setup.sh \
	&& cd /opt/intrinsic/intrinsic_sdk_cmake \
	&& touch src/intrinsic_sdk_ros/intrinsic_sdk/COLCON_IGNORE \
	&& touch src/intrinsic_sdk_ros/intrinsic_sdk_ros/COLCON_IGNORE \
	&& colcon build \
		--cmake-args -DBUILD_TESTING=ON \
		--event-handlers console_direct+ console_stderr- \
		--merge-install \
		--executor=sequential

# result stage: base + re-installed build_export depends + copy install artifacts
FROM base AS result

# Get the installed artifacts from the build stage.
COPY --from=build \
	/opt/intrinsic/intrinsic_sdk_cmake/install \
	/opt/intrinsic/intrinsic_sdk_cmake/install

# Get the list of package from the run_deps stage.
COPY --from=dependencies \
	/build_export_apt_packages.txt \
	/build_export_apt_packages.txt

# Re-install the packages from the run_deps stage.
# This avoids having the source code in the final image.
RUN dpkg --set-selections < /build_export_apt_packages.txt \
	&& apt-get dselect-upgrade

CMD ["bash"]
