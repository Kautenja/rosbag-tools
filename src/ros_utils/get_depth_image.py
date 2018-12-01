"""A method to extract FP32 depth data from ROS messages into NumPy tensors."""
import numpy as np
from ..heatmap import heatmap


def get_depth_image(data: bytes, dims: tuple,
    max_depth: float=130.0,
    as_rgb: bool=True,
) -> np.ndarray:
    """
    Get a depth image from binary ROS data.

    Args:
        data: the binary floating point data to extract a depth image from
        dims: the expected dimensions of the image
        max_depth: the max depth measurement in the floating point data
        as_rgb: whether to return RGB (True) or Floating Point (False) values

    Returns:
        an uncompressed NumPy tensor with the 8-bit RGB depth heat-map data

    """
    # get the expected number of bytes
    expected = np.prod(dims) * 4
    # get the actual number of bytes
    actual = len(data)
    # if the expected and actual bytes don't match, raise an error
    if expected != actual:
        msg = 'mismatch between expected {} and actual {} bytes'
        raise ValueError(msg.format(expected, actual))
    # extract the matrix of depths from the memory buffer
    depths = np.frombuffer(data, dtype='float32').reshape(dims)
    # normalize the depths using the L infinity norm
    depths = depths / max_depth
    # if the max depth is greater than 1 print a warning
    if depths.max() > 1:
        print('WARNING: normalized depth falls outside of [0, 1]')
    # clip the depths into [0, 1]
    depths = np.clip(depths, 0, 1)
    # convert the floating point values to 8 bit
    depths = (255 * depths).astype('uint8')
    # if RGB is disabled, return the floating point values
    if not as_rgb:
        return depths
    # create a heat-map from the depths to show as an image
    return heatmap(depths, normalize=False)


# explicitly define the outward facing API of this module
__all__ = [get_depth_image.__name__]
