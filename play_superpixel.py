"""A script to play camera footage with a super pixel segmentation overlay."""
import argparse
import numpy as np
from rosbag import Bag
from skimage.color import rgb2gray
from skimage.filters import sobel
from skimage.segmentation import felzenszwalb
from skimage.segmentation import mark_boundaries
from skimage.segmentation import slic
from skimage.segmentation import watershed
from skimage.transform import resize
from skimage.util import img_as_float
from src.ros_utils import get_camera_dimensions
from src.ros_utils import get_camera_image
from src.ros_utils import get_depth_image
from src.window import Window


def segment(img,
    method: str='felzenszwalb',
    downscale: int=2,
    mark: bool=True
):
    """
    Segment an input image using an iterative method.

    Args:
        img: the input image to segment
        method: the method to use for segmentation
        downscale: the factor to downscale the image by before segmentation
        mark: whether to return a marked image or the image segmentations

    Returns:
        a marked image or tensor of segmentations matching input image shape

    """
    # downscale the input image by a factor of `downscale`
    _img = resize(img, (img.shape[0] / downscale, img.shape[1] / downscale, img.shape[2]),
        anti_aliasing=False,
        mode='symmetric',
        clip=False,
        preserve_range=True,
    ).astype('uint8')
    _img = img_as_float(_img)
    # apply the segmentation algorithm
    if method == 'slic':
        segments = slic(_img, n_segments=1000, compactness=0.1, sigma=1)
    elif method == 'felzenszwalb':
        segments = felzenszwalb(_img, scale=25.0, sigma=0.5, min_size=30)
    elif method == 'watershed':
        segments = watershed(sobel(rgb2gray(_img)), markers=1e3, compactness=1e-4)
    else:
        raise ValueError('unexpected segmentation method: {}'.format(method))
    print(segments.max())
    # restore the segmentations to the original input shape
    segments = resize(segments, img.shape[:2],
        anti_aliasing=False,
        mode='symmetric',
        clip=False,
        preserve_range=True,
    ).astype(segments.dtype)
    # if mark is enabled, return a marked version of the input image
    if mark:
        img = mark_boundaries(img_as_float(img[..., :3]), segments)
        return (255 * img).astype('uint8')
    # otherwise return just the segmentations
    else:
        return segments


def play_superpixel(
    bag_file: Bag,
    camera_info: str,
    camera: str,
    depth: str,
    segmentation: str,
    downscale: int
) -> None:
    """
    Play the camera data in a bag file through a super pixel algorithm.

    Args:
        bag_file: the bag file to play
        camera_info: the topic to use to read metadata about the camera
        camera: the topic to use to read compressed or raw camera data
        depth: the topic to use to read 32-bit floating point depth measures
        segmentation: the algorithm to use for segmentation
        downscale: the factor to downscale the image by before segmentation

    Returns:
        None

    """
    # extract the camera dimensions from the bag
    dims = get_camera_dimensions(bag, camera_info)
    # open a window to stream the data to
    window = Window('{} ({})'.format(bag_file.filename, camera), *dims)
    # iterate over the messages
    for topic, msg, _ in bag.read_messages(topics=[camera, depth]):
        # if topic is camera, unwrap the camera data
        if topic == camera:
            camera_img = get_camera_image(msg.data, dims)
        # if topic is depth, unwrap and calculate the segmentation
        elif topic == depth:
            depth_img = get_depth_image(msg.data, dims)
            # combine the image with the depth channel (Red only)
            img = np.concatenate([camera_img, depth_img[..., 0:1]], axis=-1)
            # segment the image and get a copy of the segmented pixels
            img = segment(img, method=segmentation, downscale=downscale)
            # send the segmented image to the window
            window.show(img)
    # shut down the viewer windows
    window.close()


# ensure this script is running as the main entry point
if __name__ == '__main__':
    # create an argument parser to read arguments from the command line
    PARSER = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # add an argument for the bag file
    PARSER.add_argument('--bag_file', '-b',
        type=str,
        help='The bag file containing the ZED data to play.',
        required=True,
    )
    # add an argument for selecting the camera info topic
    PARSER.add_argument('--camera_info', '-C',
        type=str,
        help='The topic to get information about the camera from.',
        required=False,
        default='/zed/left/camera_info_raw',
    )
    # add an argument for selecting the camera data topic
    PARSER.add_argument('--camera', '-c',
        type=str,
        help='The topic to use to read camera data.',
        required=False,
        default='/zed/left/image_rect_color/compressed',
    )
    # add an argument for selecting the depth data topic
    PARSER.add_argument('--depth', '-d',
        type=str,
        help='The topic to use to read depth data.',
        required=False,
        default='/zed/depth/depth_registered',
    )
    # add an argument for selecting the segmentation method
    PARSER.add_argument('--segmentation_algorithm', '-s',
        type=str,
        help='The algorithm to use for generating segmentations.',
        required=False,
        default='felzenszwalb',
        choices={'slic', 'felzenszwalb', 'watershed'},
    )
    # add an argument for selecting the segmentation method
    PARSER.add_argument('--downscale', '-D',
        type=int,
        help='The factor to downscale images by before segmenting them',
        required=False,
        default=2,
    )
    # get the arguments from the argument parser
    ARGS = PARSER.parse_args()
    # play the bag given in the arguments
    try:
        with Bag(ARGS.bag_file) as bag:
            play_superpixel(bag,
                ARGS.camera_info,
                ARGS.camera,
                ARGS.depth,
                ARGS.segmentation_algorithm,
                ARGS.downscale
            )
    except KeyboardInterrupt:
        pass


# explicitly define the outward facing API of this module
__all__ = [play_superpixel.__name__]
