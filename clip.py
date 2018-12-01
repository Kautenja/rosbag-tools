"""Clip a segment from a ROS bag."""
import argparse
from tqdm import tqdm
from rosbag import Bag


def clip(
    input_file: Bag,
    output_file: Bag,
    sentinel_topic: list,
    start: int,
    stop: int,
) -> None:
    """
    Clip a bag using a range of messages for a given topic.

    Args:
        input_file: the input bag to stream from
        output_file: the output bag to write data to
        sentinel_topic: the topic to watch for starting and stopping the clip
        start: the starting message for the clip
        stop: the stopping message for the clip

    Returns:
        None

    """
    # initialize a counter for the sentinel topic messages
    count = 0
    # create a progress bar for iterating over the messages in the bag
    with tqdm(total=stop - start, unit='sentinel') as prog:
        # iterate over the messages in this input bag
        for topic, msg, time in input_file:
            # if the topic is the sentinel topic, increase the count
            if topic == sentinel_topic:
                count += 1
            # if the count hasn't reached the starting point, continue
            if count < start:
                continue
            # if the count has exceeded the stopping point, break
            if count >= stop:
                break
            # update the progress bar with a single iteration
            if topic == sentinel_topic:
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
    # add an argument for the topic to use to start and stop the clip
    PARSER.add_argument('--topic', '-t',
        type=str,
        help='The topic to signify the start and stop of the clip.',
        required=True,
    )
    # add an argument for the topic to use to start and stop the clip
    PARSER.add_argument('--range', '-r',
        type=int,
        nargs=2,
        help='The starting and stopping points for clipping.',
        required=True,
    )
    try:
        # get the arguments from the argument parser
        ARGS = PARSER.parse_args()
        # open the input bag with an automatically closing context
        with Bag(ARGS.input_bag, 'r') as input_bag:
            # open the output bag in an automatically closing context
            with Bag(ARGS.output_bag, 'w') as output_bag:
                # stream the input bag to the output bag
                clip(input_bag, output_bag, ARGS.topic, *ARGS.range)
    except KeyboardInterrupt:
        pass


# explicitly define the outward facing API of this module
__all__ = [clip.__name__]
