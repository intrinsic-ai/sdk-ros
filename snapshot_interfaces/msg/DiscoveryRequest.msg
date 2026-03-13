# This message is currently empty, but it may evolve in the future, and it has
# a bit more semantics when reading code than sending a std_msgs/Empty message.
#
# Because there may be several low-level camera drivers present in the system,
# and it is not known ahead of time which drivers are present, this message is
# sent to request all drivers to discover their cameras.
#
# All camera drivers subscribe to a common topic for this message, such as
# /cameras/discovery_request
#
# When a message arrives on this topic, all camera drivers will attempt to
# discover cameras, and publish a DiscoveryResponse message on a common topic,
# such as /cameras/discovery_responses
#
# Due to the variety of discovery mechanisms of each type of camera, it is
# expected that this will be an asynchronous process, with each camera type
# taking a different amount of time to publish a response.
