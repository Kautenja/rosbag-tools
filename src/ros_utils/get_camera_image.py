"""A method to extract camera data from ROS messages into a NumPy tensors."""
import io
import numpy as np
from PIL import Image


def numpy_image(data: any, dims: tuple) -> np.ndarray:
    """
    Return a NumPy tensor from image data.

    Args:
        data: the image data to convert to a NumPy tensor
        dims: the height and width of the image

    Returns:
        an RGB or RGBA NumPy tensor of the image data

    """
    # try to create an RGBA tensor from the image data
    try:
        return np.array(data, dtype='uint8').reshape((*dims, 4))
    # try to create an RGB tensor from the image data
    except ValueError:
        return np.array(data, dtype='uint8').reshape((*dims, 3))


def get_camera_image(data: bytes, dims: tuple) -> np.ndarray:
    """
    Get an image from binary ROS data.

    Args:
        data: the binary data to extract an image from
        dims: the expected dimensions of the image

    Returns:
        an uncompressed NumPy tensor with the 8-bit RGB pixel data

    """
    try:
        # open the compressed image using Pillow
        with Image.open(io.BytesIO(data)) as rgb_image:
            return numpy_image(rgb_image, dims)
    # if an OS error happens, the image is raw data
    except OSError:
        return numpy_image(list(data), dims)


# explicitly define the outward facing API of this module
__all__ = [
    get_camera_image.__name__,
    numpy_image.__name__,
]
