"""A method to create a compressed image message for a ROS bag."""
import numpy as np
import rospy
import roslib
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import CompressedImage


# TODO: fix to not use cv2 (i.e., use Pillow)
def image_compressed_msg(
    pixels: np.ndarray,
    stamp: rospy.rostime.Time,
    compression: str='png',
) -> CompressedImage:
    """
    Return a compressed ROS image from an RGB NumPy tensor.

    Args:
        pixels: the 3D NumPy tensor to convert to a compressed ROS Image
        stamp: the timestamp to use for the image message
        compression: the compression method to use on the raw data

    Returns:
        a compressed image with the compressed pixel data

    """
    raise NotImplementedError

    # ros_image = CompressedImage()
    # ros_image.header.stamp = stamp
    # ros_image.header.frame_id = "camera"
    # ros_image.format = compression
    # # use OpenCV to encode the RGB data
    # success, data = cv2.imencode('.{}'.format(compression), pixels)
    # if not success:
    #     raise ValueError('failed to png compress pixels')
    # ros_image.data = data.tobytes()

    # return ros_image


# explicitly define the outward facing API of this module
__all__ = [image_compressed_msg.__name__]
