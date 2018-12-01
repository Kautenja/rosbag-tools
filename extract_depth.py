"""A script to extract depths from a bag file."""
import argparse
import glob
import os
import numpy as np
from rosbag import Bag
from tqdm import tqdm
from src.ros_utils import get_camera_dimensions
from src.ros_utils import get_depth_image


def extract_depth(
    bag_file: Bag,
    rgb_directory: str,
    camera_info: str,
    depth: str
) -> None:
    """
    Extract depth data from a bag file.

    Args:
        bag_file: the bag file to play
        rgb_directory: the directory to find RGB image to match depths to
        camera_info: the topic to use to read metadata about the camera
        depth: the topic to use to read 32-bit floating point depth measures

    Returns:
        None

    """
    # extract the camera dimensions from the bag
    dims = get_camera_dimensions(bag, camera_info)
    # get the images from a glob
    images = glob.glob(os.path.join(rgb_directory, 'X', 'data', '*.png'))
    # convert the images to numbers
    path_to_int = lambda x: int(os.path.basename(x).replace('.png', ''))
    images = sorted([path_to_int(image) for image in images])
    # create the output directory
    output_dir = os.path.join(rgb_directory, 'D', 'data')
    try:
        os.makedirs(output_dir)
    except FileExistsError:
        pass
    # iterate over the messages
    progress = tqdm(total=len(images))
    for _, msg, time in bag_file.read_messages(topics=depth):
        # if there are no more images left, break out of the loop
        if not len(images):
            break
        # if the time is less than the current image, continue
        if int(str(time)) < images[0]:
            continue
        # update the progress bar
        progress.update(1)
        # get the depth image
        img = get_depth_image(msg.data, dims, as_rgb=False)
        # save the depth image to disk
        output_file = os.path.join(output_dir, '{}-{}.npz'.format(images[0], time))
        np.savez_compressed(output_file, y=img)
        # remove the first item from the list of times
        images.pop(0)
    # close the progress bar
    progress.close()


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
    PARSER.add_argument('--camera_info', '-c',
        type=str,
        help='The topic to get information about the camera from.',
        required=False,
        default='/zed/left/camera_info_raw',
    )
    # add an argument for selecting the depth data topic
    PARSER.add_argument('--depth', '-d',
        type=str,
        help='The topic to use to read depth data.',
        required=False,
        default='/zed/depth/depth_registered',
    )
    # add an argument for selecting the RGB image directory
    PARSER.add_argument('--rgb_directory', '-r',
        type=str,
        help='The topic to use to read depth data.',
        required=True,
    )
    # get the arguments from the argument parser
    ARGS = PARSER.parse_args()
    # play the bag given in the arguments
    try:
        with Bag(ARGS.bag_file) as bag:
            extract_depth(bag, ARGS.rgb_directory, ARGS.camera_info, ARGS.depth)
    except KeyboardInterrupt:
        pass


# explicitly define the outward facing API of this module
__all__ = [extract_depth.__name__]
