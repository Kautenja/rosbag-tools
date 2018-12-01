"""A method to create a camera info message for a ROS bag."""
import rospy
import roslib
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import CameraInfo


def camera_info_msg(stamp: rospy.rostime.Time, dims: tuple) -> CameraInfo:
    """
    Return a camera info message with given information.

    Args:
        stamp: the timestamp to use for the camera info message
        dims: a tuple of the height and width of the images from the camera

    Returns:
        a camera info message

    """
    info = CameraInfo()
    info.header.stamp = stamp
    info.height = dims[0]
    info.width = dims[1]

    return info


# explicitly define the outward facing API of this module
__all__ = [camera_info_msg.__name__]
