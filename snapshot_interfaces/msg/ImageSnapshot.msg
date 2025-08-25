string topic_name                  # Topic where the image was published. The
                                   # CameraInfo message will be captured from
                                   # its sibling topic, "../camera_info"
sensor_msgs/Image image            # The Image message on topic_name
sensor_msgs/CameraInfo camera_info # The message on the sibling camera_info
