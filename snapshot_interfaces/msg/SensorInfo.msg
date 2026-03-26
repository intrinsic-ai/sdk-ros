# The name of the sensor. This is typically something concise like
# "color" or "infrared" or "depth"
string sensor_name

# The topic name where this sensor's data can be found. (This may
# need to incorporate prefixes and suffixes around the sensor name.)
string topic_name

# The sensor type can be one of the following list. This list may grow
# over time, as new sensors are included on devices using this message.
uint32 sensor_type

uint32 IMAGE=1
uint32 POINT_CLOUD=2
uint32 IMU=3
uint32 TEMPERATURE=4
uint32 DEPTH=5
uint32 NORMAL=6

# Each sensor within a camera typically has a static transform from
# a "base link" of the camera to the sensor frame of interest, such
# as the optical center of an image sensor, or the frame of an IMU.
geometry_msgs/TransformStamped camera_t_sensor

# For image sensors, it can be convenient to have a sample CameraInfo
# before calling the Snapshot service.
sensor_msgs/CameraInfo[<=1] info
