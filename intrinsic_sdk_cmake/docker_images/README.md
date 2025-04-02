# intrinsic_sdk_cmake docker images

These docker images are used as the standard foundation for cmake/ROS based container images that run the Intrinsic Skill or Service that uses intrinsic_sdk_cmake.

The images include:

- intrinsic_sdk_cmake_base: contains the base container image with settings and prerequisites installed
- intrinsic_sdk_cmake: builds off of intrinsic_sdk_cmake_base, contains the built artifacts of intrinsic_sdk_cmake and its build_export dependencies, which makes it appropriate for a user's container that build's their software to depend on
- intrinsic_sdk_cmake_run: contains the built artifacts of intrinsic_sdk_cmake and its run-time dependencies, appropriate for a slimmed down run-time only image for the user

## How to build the images

These are some notes on how to build these images, which are normally hosted on ghcr.io/intrinsic-ai.

You shouldn't try to push to the ghcr.io/intrinsic-ai container registry if you're not a project maintainer, but you might find the following useful for local testing if you're trying to contribute or debug an issue.

### (Optional): setup a local registry for testing

First you need to start a local container registry, the easiest thing is to use podman/docker:

```
$ docker run -d -p 5000:5000 --name registry registry:2.7
```

Then you need to edit `/etc/containers/registries.conf` to contain an entry like:

```toml
[[registry]]
location = "localhost:5000"
prefix = "ghcr.io/intrinsic-ai"
insecure = true
```

Now, when you _pull_ (and only when you pull), if you reference `ghcr.io/intrinsic-ai` it will try to pull the images from your local registry.

### Building Notes

Before the step-by-step guide, some notes to be aware of, you need to map in your local bazel cache folder into the containers with a volume option like this:

```
-v "$HOME/.cache/bazel:/root/.cache/bazel"
```

This will prevent errors where the cache is expected to exist and also it will allow the cache to be reused between runs.

---

Also the images are set up to build the `latest` tag by default, but they take an argument `ARG TAG=latest` if you want to build them for a different, more specific tag.
This doesn't happen with the base image since it doesn't depend on another of our images.

---

Additionally, you should always build the images from the root of the repository, so that the docker images can access the local files from a consistent location.

### Build the base image

First build the base image:

```
# Make sure you're in the root directory of the repository.
$ podman build \
  -t ghcr.io/intrinsic-ai/intrinsic_sdk_cmake_base:latest \
  -f intrinsic_sdk_cmake/docker_images/intrinsic_sdk_cmake_base.Dockerfile \
  .
```

Note the base image doesn't need the bazel cache volume because it doesn't build anything, but it also wouldn't hurt to include it.

#### (Optional) push to local container registry

If you're using the local container registry, you will need to tag it and push it first:

```
$ podman tag \
  ghcr.io/intrinsic-ai/intrinsic_sdk_cmake_base:latest \
  localhost:5000/intrinsic_sdk_cmake_base:latest
...

$ podman push localhost:5000/intrinsic_sdk_cmake_base:latest --tls-verify=false
```

Note that you may need to use `--tls-verify=false` to get it to push to the local registry.

#### Push to the ghcr.io registry

Once you're done testing, or if you're not using the local container registry, then you need to push to ghcr.io:

```
$ podman push ghcr.io/intrinsic-ai/intrinsic_sdk_cmake_base:latest
```

Note you may need to login or authenticate with ghcr.io first and ensure you have permission to push to that project.

### Build the main image

Make sure you have built and pushed the base image before trying to build this image, and if you are iterating on the source code in this repository, make sure you rebuild this image and dependent images to uptake the source code changes.

```
# Make sure you're in the root directory of the repository.
$ podman build \
  -v "$HOME/.cache/bazel:/root/.cache/bazel" \
  --build-arg TAG=latest \
  -t ghcr.io/intrinsic-ai/intrinsic_sdk_cmake:latest \
  -f intrinsic_sdk_cmake/docker_images/intrinsic_sdk_cmake.Dockerfile \
  .
```

Note that if you want to build it off of a different tag of the base image, you'll need to change the `TAG` argument.

#### (Optional) push to local container registry

If you're using the local container registry, you will need to tag it and push it first:

```
$ podman tag \
  ghcr.io/intrinsic-ai/intrinsic_sdk_cmake:latest \
  localhost:5000/intrinsic_sdk_cmake:latest
...

$ podman push localhost:5000/intrinsic_sdk_cmake:latest --tls-verify=false
```

#### Push to the ghcr.io registry

Once you're done testing, or if you're not using the local container registry, then you need to push to ghcr.io:

```
$ podman push ghcr.io/intrinsic-ai/intrinsic_sdk_cmake:latest
```

### Build the run image

This is exactly the same process as building the main `intrinsic_sdk_cmake` image above, just replace the image name and docker file name in each place.
