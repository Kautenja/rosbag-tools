"""A script to play raw image footage."""
import argparse
from rosbag import Bag
from tqdm import tqdm
from src.ros_utils import get_camera_image
from src.window import Window


def play_images(bag_file: Bag, topics: list) -> None:
    """
    Play the data in a bag file.

    Args:
        bag_file: the bag file to play
        topics: the list of topics to play

    Returns:
        None

    """
    # open windows to stream the camera and a priori image data to
    windows = {topic: None for topic in topics}
    # iterate over the messages
    progress = tqdm(total=bag_file.get_message_count(topic_filters=topics))
    for topic, msg, time in bag_file.read_messages(topics=topics):
        # if topic is camera, unwrap and send to the camera window
        if topic in topics:
            # update the progress bar with an iteration
            progress.update(1)
            # update the progress with a post fix
            progress.set_postfix(time=time)
            # if the camera window isn't open, open it
            if windows[topic] is None:
                title = '{} ({})'.format(bag_file.filename, topic)
                windows[topic] = Window(title, msg.height, msg.width)
            # get the pixels of the camera image and display them
            img = get_camera_image(msg.data, windows[topic].shape)
            if msg.encoding == 'bgr8':
                img = img[..., ::-1]
            windows[topic].show(img[..., :3])

    # shut down the viewer windows
    for window in windows.values():
        if window is not None:
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
    # add an argument for the camera topics
    PARSER.add_argument('--topics', '-t',
        type=str,
        nargs='+',
        help='The raw image topics to play.',
        required=True,
    )
    try:
        # get the arguments from the argument parser
        ARGS = PARSER.parse_args()
        # open the bag file in a content manager
        with Bag(ARGS.bag_file, 'r') as BAG_FILE:
            # play the bag with the given camera topics
            play_images(BAG_FILE, ARGS.topics)
    except KeyboardInterrupt:
        pass


# explicitly define the outward facing API of this module
__all__ = [play_images.__name__]
