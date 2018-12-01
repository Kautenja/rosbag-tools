"""Shift a topic forward or backward in time."""
import argparse
import rospy
from rosbag import Bag
from tqdm import tqdm


def shift(
    input_file: Bag,
    output_file: Bag,
    shift_topic: list,
    shift_sec: float,
) -> None:
    """
    Shift a topic in a bag forward or backwards in time.

    Args:
        input_file: the input bag to stream from
        output_file: the output bag to write data to
        topic: the topic to shift in time
        shift_sec: the amount of time to shift by

    Returns:
        None

    """
    # create a progress bar for iterating over the messages in the bag
    with tqdm(total=input_file.get_message_count()) as prog:
        # iterate over the messages in this input bag
        for topic, msg, time in input_file:
            # if the topic is the sentinel topic, increase the count
            if topic == shift_topic:
                # adjust time by the shift value
                time += rospy.Duration(shift_sec)
                # update the message header with the new time stamp
                msg.header.stamp = time
            # update the progress bar with a single iteration
            prog.update(1)
            # write the message
            output_file.write(topic, msg, time)


# ensure this script is running as the main entry point
if __name__ == '__main__':
    # create an argument parser to read arguments from the command line
    PARSER = argparse.ArgumentParser(description=__doc__)
    # add an argument for the output bag to create
    PARSER.add_argument('--output_bag', '-o',
        type=str,
        help='The path to an output bag file to write to.',
        required=True,
    )
    # add an argument for the input bag to clip
    PARSER.add_argument('--input_bag', '-i',
        type=str,
        help='The path to an input bag to clip.',
        required=True,
    )
    # add an argument for the topic to shift in time
    PARSER.add_argument('--topic', '-t',
        type=str,
        help='The topic to shift in time.',
        required=True,
    )
    # add an argument for the amount of time to shift
    PARSER.add_argument('--shift', '-s',
        type=float,
        help='The amount of time to shift the topic by.',
        required=True,
    )
    # get the arguments from the argument parser
    ARGS = PARSER.parse_args()
    try:
        # open the input bag with an automatically closing context
        with Bag(ARGS.input_bag, 'r') as input_bag:
            # open the output bag in an automatically closing context
            with Bag(ARGS.output_bag, 'w') as output_bag:
                # stream the input bag to the output bag
                shift(input_bag, output_bag, ARGS.topic, ARGS.shift)
    except KeyboardInterrupt:
        pass


# explicitly define the outward facing API of this module
__all__ = [shift.__name__]
