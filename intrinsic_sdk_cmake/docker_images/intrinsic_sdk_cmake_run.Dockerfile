# dependencies stage: source + rosdep install run dependencies only
FROM source AS dependencies

# Install exec dependencies for the packages in intrinsic_sdk_ros.
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
		--dependency-types exec \
	&& rm -rf /var/lib/apt/lists/*

# Save installed packages for re-install without source in the run stage.
# TODO(wjwwood): it would be nice to get the list of packages from rosdep
#   without installing them, but I couldn't figure out how to do that easily.
RUN dpkg --get-selections > /exec_apt_packages.txt
