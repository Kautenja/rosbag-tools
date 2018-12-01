"""Predict images from input directory to output directory."""
import argparse
import ast
import os
import numpy as np
import pandas as pd
import rospy
from keras.models import load_model
from PIL import Image
from rosbag import Bag
from skimage.transform import resize
from std_msgs.msg import String
from tqdm import tqdm
from src.ros_utils import get_camera_image
from src.ros_utils import image_msg
from src.window import Window


def seed(value: int) -> None:
    """
    Seed the RNG with a given value.

    Args:
        value: the value to seed the RNG with

    Returns:
        None

    """
    import random
    random.seed(ARGS.seed)
    import numpy as np
    np.random.seed(ARGS.seed)
    import tensorflow as tf
    tf.set_random_seed(ARGS.seed)


def read_rgb_map(metadata: str) -> tuple:
    """
    Return an RGB map and a vectorized method to map codes to RGB tuples.

    Args:
        metadata: the path to the metadata file to load

    Returns:
        a tuple of:
        - a dictionary mapping discrete codes to RGB tuples
        - a vectorized method for mapping discrete codes to RGB tuples

    """
    # load the metadata table using pandas
    metadata = pd.read_csv(metadata)
    # extract the RGB map from the table using the code and draw columns
    rgb_map = metadata[['code', 'rgb_draw']]
    # set the code as the index and convert the table to a dictionary
    rgb_map = rgb_map.set_index('code').to_dict()['rgb_draw']
    # evaluate each RGB tuple string to parse the data
    rgb_map = {k: ast.literal_eval(v) for (k, v) in rgb_map.items()}
    # vectorize the getter of rgb_map
    unmap_rgb = np.vectorize(rgb_map.get)

    return rgb_map, unmap_rgb


def semantic_segment(
    metadata: str,
    input_bag: Bag,
    model: 'keras.models.Model',
    predict: str,
    output_bag: Bag=None,
    output_dir: str=None,
    base: str=None,
    num_samples: int=200,
    encoding: str='rgb',
) -> None:
    """
    Predict a stream of images from an input ROSbag.

    Args:
        metadata: the metadata about the semantic segmentations from the model
        input_bag: the input bag to predict targets from a topic
        model: the semantic segmentation model to use to make predictions
        predict: the topic to get a priori estimates from
        output_bag: the output bag to write the a priori estimates to
        output_dir: the output directory to write image pairs to
        base: the base-name for the prediction image topic
        num_samples: the number of image pairs to sample for output directory
        encoding: the encoding for the images to write

    Returns:
        None

    """
    # create the base endpoint for the topics
    base = '' if base is None else '{}'.format(base)

    # setup the output directories
    if output_dir is not None:
        x_dir = os.path.join(output_dir, 'X', 'data')
        if not os.path.isdir(x_dir):
            os.makedirs(x_dir)
        y_dir = os.path.join(output_dir, 'y', 'data')
        if not os.path.isdir(y_dir):
            os.makedirs(y_dir)

    # read the RGB map and vectorized method from the metadata file
    rgb_map, unmap_rgb = read_rgb_map(metadata)

    # write the color map metadata to the output bag
    if output_bag is not None:
        ros_stamp = rospy.rostime.Time(input_bag.get_start_time())
        msg = String(repr(rgb_map))
        output_bag.write('{}/rgb_map'.format(rgb_map), msg, ros_stamp)

    # open a Window to play the video
    x_window = Window('img', model.input_shape[1], model.input_shape[2])
    y_window = Window('sem-seg', model.output_shape[1], model.output_shape[2])
    # create a progress bar for iterating over the messages in the bag
    total_messages = input_bag.get_message_count(topic_filters=predict)
    with tqdm(total=total_messages, unit='message') as prog:
        # iterate over the messages in this input bag
        for _, msg, time in input_bag.read_messages(topics=predict):
            # update the progress bar with a single iteration
            prog.update(1)
            if np.random.random() > num_samples / total_messages:
                continue
            # create a tensor from the raw pixel data
            pixels = get_camera_image(msg.data, (msg.height, msg.width))[..., :3]
            # flip the BGR image to RGB
            if encoding == 'bgr':
                pixels = pixels[..., ::-1]
            # resize the pixels to the shape of the model
            _pixels = resize(pixels, model.input_shape[1:],
                anti_aliasing=False,
                mode='symmetric',
                clip=False,
                preserve_range=True,
            ).astype('uint8')
            # pass the frame through the model
            y_pred = model.predict(_pixels[None, ...])[0]
            y_pred = np.stack(unmap_rgb(y_pred.argmax(axis=-1)), axis=-1)
            y_pred = y_pred.astype('uint8')
            # show the pixels on the windows
            x_window.show(_pixels)
            y_window.show(y_pred)
            # create an Image message and write it to the output ROSbag
            if output_bag is not None:
                msg = image_msg(y_pred, msg.header.stamp, y_pred.shape[:2], 'rgb8')
                output_bag.write('{}/image_raw'.format(base), msg, msg.header.stamp)
            # sample a number and write the image pair to disk
            if output_dir is not None:
                x_file = os.path.join(x_dir, '{}.png'.format(time))
                Image.fromarray(pixels).save(x_file)
                y_file = os.path.join(y_dir, '{}.png'.format(time))
                y_pred = resize(y_pred, pixels.shape[:2],
                    anti_aliasing=False,
                    mode='symmetric',
                    clip=False,
                    preserve_range=True,
                ).astype('uint8')
                Image.fromarray(y_pred).save(y_file)


# ensure this script is running as the main entry point
if __name__ == '__main__':
    # create an argument parser to read arguments from the command line
    PARSER = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # add an argument for the metadata file
    PARSER.add_argument('--metadata', '-M',
        type=str,
        help='The model to use to predict the data.',
        required=True,
    )
    # add an argument for the input bag file
    PARSER.add_argument('--input_bag', '-i',
        type=str,
        help='The bag file with the data to predict.',
        required=True,
    )
    # add an argument for the model file
    PARSER.add_argument('--model', '-m',
        type=str,
        help='The model to use to predict the data.',
        required=True,
    )
    # add an argument for the topic to predict values from
    PARSER.add_argument('--predict', '-p',
        type=str,
        help='The name of the topic to use to predict values from.',
        required=True,
    )
    # add an argument for the output bag file to create
    PARSER.add_argument('--output_bag',
        type=str,
        help='The bag to output data to.',
        required=False,
        default=None,
    )
    # add an argument for the output directory to create
    PARSER.add_argument('--output_dir',
        type=str,
        help='The directory to output data to.',
        required=False,
        default=None,
    )
    # add an argument for the topics to subscribe to
    PARSER.add_argument('--base', '-B',
        type=str,
        help='the base endpoint to attach the data as.',
        required=False,
        default=None,
    )
    # add an argument for the encoding to use
    PARSER.add_argument('--num_samples', '-N',
        type=int,
        help='the number of samples to produce.',
        required=False,
        default=200,
    )
    # add an argument for the encoding to use
    PARSER.add_argument('--encoding', '-e',
        type=str,
        help='the encoding of the input images.',
        required=False,
        choices={'rgb', 'bgr'},
        default='rgb',
    )
    # add an argument for the encoding to use
    PARSER.add_argument('--seed', '-s',
        type=int,
        help='the random number seed to use.',
        required=False,
        default=1,
    )
    try:
        # get the arguments from the argument parser
        ARGS = PARSER.parse_args()
        # seed the random number generator before any other code runs
        seed(ARGS.seed)
        # open the input bag to read camera data from
        with Bag(ARGS.input_bag, 'r') as IN_BAG:
            # create the output bag to write predicted masks to
            if ARGS.output_bag is not None:
                ARGS.output_bag = Bag(ARGS.output_bag, 'w')
            # predict the stream of data
            semantic_segment(
                ARGS.metadata,
                IN_BAG,
                load_model(ARGS.model),
                ARGS.predict,
                ARGS.output_bag,
                ARGS.output_dir,
                ARGS.base,
                ARGS.num_samples,
                ARGS.encoding,
            )
            # close the output bag if there is one
            if ARGS.output_bag is not None:
                ARGS.output_bag.close()
    except KeyboardInterrupt:
        pass


# explicitly define the outward facing API of this module
__all__ = [semantic_segment.__name__]
