"""A script to play raw ZED footage."""
import argparse
from rosbag import Bag
from src.ros_utils import get_camera_dimensions
from src.ros_utils import get_camera_image
from src.ros_utils import get_depth_image
from src.window import Window


def play_depth(bag_file: Bag, camera_info: str, camera: str, depth: str) -> None:
    """
    Play the ZED data in a bag file.

    Args:
        bag_file: the bag file to play
        camera_info: the topic to use to read metadata about the camera
        camera: the topic to use to read compressed or raw camera data
        depth: the topic to use to read 32-bit floating point depth measures

    Returns:
        None

    """
    # extract the camera dimensions from the bag
    dims = get_camera_dimensions(bag, camera_info)
    # open windows to stream the camera and depth image data to
    camera_window = Window('{} ({})'.format(bag_file.filename, camera), *dims)
    depth_window = Window('{} ({})'.format(bag_file.filename, depth), *dims)
    # iterate over the messages
    for topic, msg, _ in bag.read_messages(topics=[camera, depth]):
        # if topic is depth, unwrap and send to the depth window
        if topic == depth:
            img = get_depth_image(msg.data, dims)
            depth_window.show(img)
        # if topic is camera, unwrap and send to the camera window
        elif topic == camera:
            img = get_camera_image(msg.data, dims)
            camera_window.show(img)
    # shut down the viewer windows
    camera_window.close()
    depth_window.close()


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
    # get the arguments from the argument parser
    ARGS = PARSER.parse_args()
    # play the bag given in the arguments
    try:
        with Bag(ARGS.bag_file) as bag:
            play_depth(bag, ARGS.camera_info, ARGS.camera, ARGS.depth)
    except KeyboardInterrupt:
        pass


# explicitly define the outward facing API of this module
__all__ = [play_depth.__name__]
