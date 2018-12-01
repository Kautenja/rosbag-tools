"""Dump messages from a bag."""
import argparse
from rosbag import Bag
from tqdm import tqdm


def dump(input_file: Bag, topics: list, output_file: 'file'=None) -> None:
    """
    Dump messages from a bag.

    Args:
        input_file: the input bag to dump topics from
        topics: the topics to dump
        output_file: an optional file to dump to

    Returns:
        None

    """
    # create a progress bar for iterating over the messages in the bag
    with tqdm(total=input_file.get_message_count(topic_filters=topics)) as prog:
        # iterate over the messages in this input bag
        for topic, msg, _ in input_file.read_messages(topics=topics):
            # update the progress bar with a single iteration
            prog.update(1)
            # create the line to print
            line = '{} {}\n\n'.format(topic, msg)
            # print the line to the terminal
            print(line)
            # if there is an output file, write the line to it
            if output_file is not None:
                output_file.write(line)


# ensure this script is running as the main entry point
if __name__ == '__main__':
    # create an argument parser to read arguments from the command line
    PARSER = argparse.ArgumentParser(description=__doc__)
    # add an argument for the input bag to read data from
    PARSER.add_argument('--input_bag', '-i',
        type=str,
        help='The input bag file to read from',
    )
    # add an argument for the topics to dump
    PARSER.add_argument('--topics', '-t',
        type=str,
        nargs='+',
        help='A list of input bag files',
    )
    # add an argument for the optional output file
    PARSER.add_argument('--output_file', '-o',
        type=str,
        help='An optional output file to dump to instead of the command line.',
        default=None,
        required=False,
    )
    try:
        # get the arguments from the argument parser
        ARGS = PARSER.parse_args()
        # open the input bag with an automatically closing context
        with Bag(ARGS.input_bag, 'r') as input_bag:
            # if there is an output file path, open the file
            if ARGS.output_file is not None:
                ARGS.output_file = open(ARGS.output_file, 'w')
            # stream the input bag to the output bag
            dump(input_bag, ARGS.topics, ARGS.output_file)
            # if there was an output file, close it
            if ARGS.output_file is not None:
                ARGS.output_file.close()
    except KeyboardInterrupt:
        pass


# explicitly define the outward facing API of this module
__all__ = [dump.__name__]
