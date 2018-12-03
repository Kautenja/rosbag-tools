"""A script to convert a video file to a ROS bag."""
import argparse
import os
import sys
import rospy
from rosbag import Bag
from tqdm import tqdm
from src.ros_utils import camera_info_msg
from src.ros_utils import image_compressed_msg
from src.ros_utils import image_msg
# hacky fix to remove ROS from python path to import a different cv2
del sys.path[1]
import cv2


# the default encoding when working with OpenCV streams
ENCODING = 'bgr8'


def video_to_bag(
    video_path: str,
    output_file: Bag,
    topics: set={'/image_raw', '/image_raw/compressed'},
    compression: str='png',
    base: str=None,
    time_zone: int=6,
    frame_skip: int=1,
) -> None:
    """
    Convert a video file to a ROS bag file.

    Args:
        video_path: the path to the video to convert to a bag
        output_file: the bag file to publish the camera topics to
        topics: the camera topics to publish
        compression: the compression to use as either 'png' or 'jpeg'
        base: the base for the camera topics to publish (i.e., '/foo')
        time_zone: the hour offset to adjust for UTC time zone
        frame_skip: the number of frames to skip between output messages

    Returns:
        None

    """
    # open the video file
    video = cv2.VideoCapture(video_path)
    # read the start time of the video from the file
    start = os.path.getmtime(video_path) + time_zone * 3600
    # get the dimensions of frames in the video
    height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
    width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    dims = height, width
    # set the base end point to an empty string if it's None
    base = '' if base is None else '{}'.format(base)
    # get the number of frames in the video
    frame_count = int(video.get(cv2.CAP_PROP_frame_count))
    # write the camera metadata to the bag if compression is in the topics
    if '/image_raw/compressed' in topics:
        ros_stamp = rospy.rostime.Time.from_seconds(start)
        topic = '{}/camera_info'.format(base)
        msg = camera_info_msg(ros_stamp, dims)
        output_file.write(topic, msg, ros_stamp)
    # write the images to the bag
    for idx in tqdm(range(frame_count), unit='frame'):
        # get the frame's point in time relative to the origin frame
        sec = video.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
        # create a timestamp for this frame
        ros_stamp = rospy.rostime.Time.from_seconds(start + sec)
        # read an image and the flag for a next image being available
        _, image = video.read()
        # skip the frame if it's not aligned with the skip
        if idx % frame_skip != 0:
            continue
        # write a raw image if the argument is set
        if '/image_raw' in topics:
            topic = '{}/image_raw'.format(base)
            msg = image_msg(image, ros_stamp, dims, ENCODING)
            output_file.write(topic, msg, ros_stamp)
        # write a compressed image if the argument is set
        if '/image_raw/compressed' in topics:
            topic = '{}/image_raw/compressed'.format(base)
            msg = image_compressed_msg(image, ros_stamp, compression)
            output_file.write(topic, msg, ros_stamp)
    # release the video file
    video.release()


# ensure this script is running as the main entry point
if __name__ == '__main__':
    # create an argument parser to read arguments from the command line
    PARSER = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # add an argument for the video file
    PARSER.add_argument('--video_file', '-v',
        type=str,
        help='The video file to convert to a ROS bag (.bag).',
        required=True,
    )
    # add an argument for the video file
    PARSER.add_argument('--bag_file', '-b',
        type=str,
        help='The name of the ROSbag file to create.',
        required=True,
    )
    # add an argument for the topics to subscribe to
    PARSER.add_argument('--topics', '-t',
        type=str,
        help='The topics to subscribe to.',
        required=False,
        nargs='+',
        default=['/image_raw', '/image_raw/compressed'],
    )
    # add an argument for the format of compression
    PARSER.add_argument('--compression', '-c',
        type=str,
        help='The type of image compression to use.',
        required=False,
        default='png',
        choices={'png', 'jpeg'}
    )
    # add an argument for the topics to subscribe to
    PARSER.add_argument('--base', '-B',
        type=str,
        help='the base endpoint to attach the data as.',
        required=False,
        default=None,
    )
    # add an argument for the topics to subscribe to
    PARSER.add_argument('--time_zone', '-T',
        type=int,
        help='the adjustment for the time zone in hours.',
        required=False,
        default=6,
    )
    # add an argument for the topics to subscribe to
    PARSER.add_argument('--frame_skip', '-f',
        type=int,
        help='the number of frames to skip when writing to the bag.',
        required=False,
        default=1,
    )
    # get the arguments from the argument parser
    ARGS = PARSER.parse_args()
    # iterate over all the frames in the video
    try:
        with Bag(ARGS.bag_file, 'w') as bag_file:
            video_to_bag(
                ARGS.video_file,
                bag_file,
                ARGS.topics,
                ARGS.compression,
                ARGS.base,
                ARGS.time_zone,
                ARGS.frame_skip,
            )
    except KeyboardInterrupt:
        pass


# explicitly define the outward facing API of this module
__all__ = [video_to_bag.__name__]
