"""Merge one or more bag files and filter topics."""
import argparse
from fnmatch import fnmatchcase
from rosbag import Bag
from tqdm import tqdm


def stream(input_file: Bag, output_file: Bag, topics: list) -> None:
    """
    Stream data from an input bag to an output bag.

    Args:
        input_file: the input bag to stream from
        output_file: the output bag to write data to
        topics: a list of the topics to include

    Returns:
        None

    """
    included = 0
    skipped = 0
    # create a progress bar for iterating over the messages in the bag
    with tqdm(total=input_file.get_message_count(), unit='message') as prog:
        # iterate over the messages in this input bag
        for topic, msg, time in input_file:
            # check for matches between the topics filter and this topic
            if any(fnmatchcase(topic, pattern) for pattern in topics):
                # write this message to the output bag
                output_file.write(topic, msg, time)
                # increment the counter of included messages
                included += 1
            else:
                # increment the counter of excluded messages
                skipped += 1
            # update the progress bar with a single iteration
            prog.update(1)
            # update the progress bar post fix text with statistics
            prog.set_postfix(included=included, skipped=skipped)


# ensure this script is running as the main entry point
if __name__ == '__main__':
    # create an argument parser to read arguments from the command line
    PARSER = argparse.ArgumentParser(description=__doc__)
    # add an argument for the output bag to create
    PARSER.add_argument('--output_bag', '-o',
        type=str,
        help='The output bag file to write to',
    )
    # add an argument for the sequence of input bags
    PARSER.add_argument('--input_bags', '-i',
        type=str,
        nargs='+',
        help='A list of input bag files',
    )
    # add an argument for the topics to filter
    PARSER.add_argument('--topics', '-t',
        type=str,
        nargs='*',
        help='A sequence of topics to include from the input bags.',
        default=['*'],
        required=False,
    )
    try:
        # get the arguments from the argument parser
        ARGS = PARSER.parse_args()
        # open the output bag in an automatically closing context
        with Bag(ARGS.output_bag, 'w') as output_bag:
            # iterate over the input files
            for filename in tqdm(ARGS.input_bags, unit='bag'):
                # open the input bag with an automatically closing context
                with Bag(filename, 'r') as input_bag:
                    # stream the input bag to the output bag
                    stream(input_bag, output_bag, ARGS.topics)
    except KeyboardInterrupt:
        pass


# explicitly define the outward facing API of this module
__all__ = [stream.__name__]
