"""A method to create a raw image message for a ROS bag."""
import numpy as np
import rospy
import roslib
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image


#TODO: remove the dims argument and use implicitly from the pixels vector
def image_msg(
    pixels: np.ndarray,
    stamp: rospy.rostime.Time,
    dims: tuple,
    encoding: str,
) -> Image:
    """
    Return a ROS image from an RGB NumPy tensor.

    Args:
        pixels: the 3D NumPy tensor to convert to an ROS Image
        stamp: the timestamp to use for the image message
        dims: a tuple of the height and width of the image
        encoding: the encoding to use for the image (e.g., 'rgb8')

    Returns:
        an image with the raw pixel data

    """
    ros_image = Image()
    ros_image.header.stamp = stamp
    ros_image.header.frame_id = "camera"
    ros_image.height = dims[0]
    ros_image.width = dims[1]
    ros_image.encoding = encoding
    ros_image.data = pixels.flatten().tobytes()

    return ros_image


# explicitly define the outward facing API of this module
__all__ = [image_msg.__name__]
