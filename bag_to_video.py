"""A script to convert a ROS bag image topic to a video file."""
import argparse
import os
import sys
from rosbag import Bag
from tqdm import tqdm
from src.ros_utils import get_camera_image
# hacky fix to remove ROS from python path to import a different cv2
del sys.path[1]
import cv2


def bag_to_video(
    input_file: Bag,
    output_file: str,
    topic: str,
    fps: float=30,
    codec: str='MJPG',
) -> None:
    """
    Convert a ROS bag with image topic to a video file.

    Args:
        input_file: the bag file to get image data from
        output_file: the path to an output video file to create
        topic: the topic to read image data from
        fps: the frame rate of the video
        codec: the codec to use when outputting to the video file

    Returns:
        None

    """
    # create an empty reference for the output video file
    video = None
    # get the total number of frames to write
    total = input_file.get_message_count(topic_filters=topic)
    # get an iterator for the topic with the frame data
    iterator = input_file.read_messages(topics=topic)
    # iterate over the image messages of the given topic
    for _, msg, _ in tqdm(iterator, total=total):
        # open the video file if it isn't open
        if video is None:
            # create the video codec
            codec = cv2.VideoWriter_fourcc(*codec)
            # open the output video file
            cv_dims = (msg.width, msg.height)
            video = cv2.VideoWriter(output_file, codec, fps, cv_dims)
        # read the image data into a NumPy tensor
        img = get_camera_image(msg.data, (msg.height, msg.width))
        # write the image to the video file
        video.write(img)

    # if the video file is open, close it
    if video is not None:
        video.release()

    # read the start time of the video from the bag file
    start = input_file.get_start_time()
    # set the access and modification times for the video file
    os.utime(output_file, (start, start))


# ensure this script is running as the main entry point
if __name__ == '__main__':
    # create an argument parser to read arguments from the command line
    PARSER = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # add an argument for the video file
    PARSER.add_argument('--bag_file', '-b',
        type=str,
        help='The name of the ROSbag file to create.',
        required=True,
    )
    # add an argument for the video file
    PARSER.add_argument('--video_file', '-v',
        type=str,
        help='The output video file to create .',
        required=True,
    )
    # add an argument for the topic to get image data from
    PARSER.add_argument('--topic', '-t',
        type=str,
        help='The topic to read raw image data from.',
        required=True,
    )
    # add an argument for the topics to subscribe to
    PARSER.add_argument('--fps', '-f',
        type=int,
        help='The frame-rate of the video file to create.',
        required=False,
        default=30,
    )
    # add an argument for the topics to subscribe to
    PARSER.add_argument('--codec', '-c',
        type=str,
        help='The codec of the video file to create.',
        required=False,
        default='MJPG',
    )
    # get the arguments from the argument parser
    ARGS = PARSER.parse_args()
    # iterate over all the frames in the video
    try:
        # open the input bag file
        with Bag(ARGS.bag_file, 'r') as bag:
            # convert the bag file to a video file with the command line args
            bag_to_video(bag, ARGS.video_file, ARGS.topic, ARGS.fps, ARGS.codec)
    except KeyboardInterrupt:
        pass


# explicitly define the outward facing API of this module
__all__ = [bag_to_video.__name__]
