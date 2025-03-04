# base stage: ros:jazzy + configs
FROM ros:jazzy AS base
# TODO(wjwwood): this is based on the ROS image because we still use
#   ament_cmake, we should move away from that and either vendor ament_cmake or
#   avoid it entirely.

WORKDIR /opt/intrinsic

ENV ROS_HOME=/tmp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# source stage: base + source added
FROM base AS source

ADD ../.. /opt/intrinsic/intrinsic_sdk_cmake/src/intrinsic_sdk_ros

# build stage: source + rosdep install + dependencies built
FROM source AS build

# Install standard dependencies for the packages in intrinsic_sdk_ros.
# Exclude intrinsic_sdk_ros and intrinsic_sdk for now.
RUN . /opt/ros/jazzy/setup.sh \
	&& apt-get update \
	&& rosdep init \
	&& rosdep update \
	&& cd /opt/intrinsic/intrinsic_sdk_cmake \
	&& rosdep install \
		--from-paths src \
		--ignore-src \
		--default-yes \
		--skip-keys "intrinsic_sdk_ros intrinsic_sdk"

RUN . /opt/ros/jazzy/setup.sh \
	&& apt-get update \
	&& apt install -y ros-jazzy-ament-cmake-vendor-package \
	&& cd /opt/intrinsic/intrinsic_sdk_cmake \
	&& colcon build \
		--cmake-args -DBUILD_TESTING=ON \
		--event-handlers console_direct+ console_stderr- \
		--merge-install \
		--packages-up-to intrinsic_sdk_cmake

# run_deps stage: source + rosdep install run dependencies only
FROM source AS run_deps

# Install exec dependencies for the packages in intrinsic_sdk_ros.
# Exclude intrinsic_sdk_ros and intrinsic_sdk for now.
RUN . /opt/ros/jazzy/setup.sh \
	&& apt-get update \
	&& rosdep init \
	&& rosdep update \
	&& cd /opt/intrinsic/intrinsic_sdk_cmake \
	&& rosdep install \
		--from-paths src \
		--ignore-src \
		--default-yes \
		--dependency-types exec \
		--skip-keys "intrinsic_sdk_ros intrinsic_sdk"

# Save installed packages for re-install without source in the run stage.
# TODO(wjwwood): it would be nice to get the list of packages from rosdep
#   without installing them, but I couldn't figure out how to do that easily.
RUN dpkg --get-selections > /apt_packages.txt

# run stage: base + re-installed exec depends + copy install artifacts
FROM base AS run

# Get the installed artifacts from the build stage.
COPY --from=build \
	/opt/intrinsic/intrinsic_sdk_cmake/install \
	/opt/intrinsic/intrinsic_sdk_cmake/install

# Get the list of package from the run_deps stage.
COPY --from=run_deps \
	/apt_packages.txt \
	/apt_packages.txt

# Re-install the packages from the run_deps stage.
# This avoids having the source code in the final image.
RUN dpkg --set-selections < /apt_packages.txt \
	&& apt-get dselect-upgrade

CMD ["bash"]
