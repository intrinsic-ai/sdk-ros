# Snapshot Interfaces

This package provided interfaces for retrieving sensor data from cameras
intermittently, as opposed to subscribing to a continuous data streams.

It is intended to be provided either directly by a camera driver node,
or by an "adapter node" that sits "in-between" a pre-existing streaming
camera driver node and a client of this service.

### Pre-existing work

The `polled_camera` package uses a different approach, where image and
`camera_info` topics are still used, but messages are only published on them
when requested by a service call.  This package, in contrast, includes
snapshots of all the requested topics in a single request-response transaction.
This is convenient when there are many topics produced by a multi-sensor
camera.
