"""A script to play raw ZED footage."""
import argparse
from rosbag import Bag
from src.ros_utils import get_camera_image
from src.ros_utils import get_depth_image
from src.window import Window


def play_depth(bag_file: Bag, camera: str, depth: str) -> None:
    """
    Play the ZED data in a bag file.

    Args:
        bag_file: the bag file to play
        camera: the topic to use to read compressed or raw camera data
        depth: the topic to use to read 32-bit floating point depth measures

    Returns:
        None

    """
    # open windows to stream the camera and depth image data to
    camera_window = None
    depth_window = None
    # iterate over the messages
    for topic, msg, _ in bag.read_messages(topics=[camera, depth]):
        # if topic is depth, unwrap and send to the depth window
        if topic == depth:
            # if the depth window is not setup yet, open it
            if depth_window is None:
                # create a title for the window
                title = '{} ({})'.format(bag_file.filename, depth)
                # initialize the window
                depth_window = Window(title, msg.height, msg.width)
            # get a depth image from the data and dimensions
            img = get_depth_image(msg.data, depth_window.shape)
            # show the image on the depth window
            depth_window.show(img)
        # if topic is camera, unwrap and send to the camera window
        elif topic == camera:
            # if the camera window is not setup yet, open it
            if camera_window is None:
                # create a title for the window
                title = '{} ({})'.format(bag_file.filename, camera)
                # initialize the window
                camera_window = Window(title, msg.height, msg.width)
            # get an image from the data and dimensions
            img = get_camera_image(msg.data, camera_window.shape)
            # show the image on the camera window
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
    # add an argument for selecting the camera data topic
    PARSER.add_argument('--camera', '-c',
        type=str,
        help='The topic to use to read camera data.',
        required=False,
        default='/zed/left/image_rect_color',
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
            play_depth(bag, ARGS.camera, ARGS.depth)
    except KeyboardInterrupt:
        pass


# explicitly define the outward facing API of this module
__all__ = [play_depth.__name__]
