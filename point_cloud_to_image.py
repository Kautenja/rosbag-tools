"""A script to play raw ZED footage."""
import argparse
import numpy as np
from rosbag import Bag
from skimage.transform import resize
from tqdm import tqdm
from sensor_msgs import point_cloud2
from src.window import Window


AZIMUTH_MIN = np.radians(0)
AZIMUTH_MAX = np.radians(180)
ELEVATION_MIN = np.radians(-15.0)
ELEVATION_MAX = np.radians(15.0)


X_MIN = 0
X_MAX = 130
Y_MIN = -10
Y_MAX = 10
Z_MIN = -5
Z_MAX = 10


X_SHAPE = int((X_MAX - X_MIN))
Y_SHAPE = int((Y_MAX - Y_MIN))
Z_SHAPE = int((Z_MAX - Z_MIN))
SHAPE = (Y_SHAPE, Z_SHAPE)


def pointcloud2_to_image(
    input_file: Bag,
    output_file: Bag,
    point_cloud: str,
    camera: str
) -> None:
    """
    Play PointCloud2 data in one bag file into another as image data.

    Args:
        input_file: the input bag to read data from
        output_file: the output bag to write data to
        points_topic: the topic to read PointCloud2 data from
        camera_topic: the topic to write camera data to

    Returns:
        None

    """
    window = Window('{} ({})'.format(output_file.filename, camera), 352, 480)
    # the total number of point clouds
    total = input_file.get_message_count(topic_filters=point_cloud)
    # create a progress bar
    progress = tqdm(total=total, unit='cloud')
    # iterate over all the PointCloud2 messages
    for _, msg, time in input_file.read_messages(topics=point_cloud):
        # update the progress bar for this iteration
        progress.update(1)
        # print a warning if there are NaN values in the message
        if not msg.is_dense:
            print('WARNING: message t={} has NaN values'.format(time))
        # read the point generator from the message with x,y,z data
        points = point_cloud2.read_points(msg, field_names=list('xyz'))
        # convert the points generator to a vector with shape (N, 3)
        points = np.array(list(points))
        # convert the point cloud from Cartesian to spherical coordinates
        radius = np.sqrt(np.sum(points**2, axis=-1))
        azimuth = np.arctan2(points[..., 0], points[..., 2])
        elevation = np.arcsin(points[..., 1] / radius)
        # find the points where the azimuth and elevation are within range
        a_idx = (AZIMUTH_MIN < azimuth) * (azimuth < AZIMUTH_MAX)
        e_idx = (ELEVATION_MIN < elevation) * (elevation < ELEVATION_MAX)
        points = points[a_idx * e_idx]
        # find the points within the camera bounds
        x_idx = (X_MIN < points[:, 0]) * (points[:, 0] < X_MAX)
        y_idx = (Y_MIN < points[:, 1]) * (points[:, 1] < Y_MAX)
        z_idx = (Z_MIN < points[:, 2]) * (points[:, 2] < Z_MAX)
        points = points[x_idx * y_idx * z_idx]
        # move the points into a normalized frame
        points[:, 0] = points[:, 0] - X_MIN
        points[:, 1] = points[:, 1] - Y_MIN
        points[:, 2] = points[:, 2] - Z_MIN
        # expand the points into a larger space and truncate via flooring
        points = np.floor(points)
        # create an image for this set of points
        image = np.zeros(SHAPE)
        # set the depth dimension (Y) in the image using the (X, Z) coordinates
        image[points[:, 1].astype(int), points[:, 2].astype(int)] = points[:, 0]
        # transpose and flip the image into the correct orientation
        image = np.transpose(image)
        image = np.flip(image, axis=0)
        # convert the image to an 8-bit encoding
        image = (255 * image / X_SHAPE).astype('uint8')
        # convert the 8-bit encoding to RGB and send to the windshield
        image = np.stack(3 * [image**2], axis=-1)

        image = resize(image, (352, 480),
            anti_aliasing=False,
            mode='symmetric',
            clip=False,
            preserve_range=True,
        ).astype(image.dtype)

        window.show(image)

    # close the progress bar
    progress.close()
    window.close()


if __name__ == '__main__':
    # create an argument parser to read arguments from the command line
    PARSER = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # add an argument for the bag file
    PARSER.add_argument('--input_bag', '-i',
        type=str,
        help='The bag file containing the point cloud data to read.',
        required=True,
    )
    PARSER.add_argument('--output_bag', '-o',
        type=str,
        help='The name of the bag file to write output data to.',
        required=True,
    )
    # add an argument for reading the point cloud data
    PARSER.add_argument('--point_cloud', '-p',
        type=str,
        help='The topic containing point cloud data to read.',
        required=False,
        default='/velodyne_points',
    )
    # add an argument for selecting the camera data topic
    PARSER.add_argument('--camera', '-c',
        type=str,
        help='The topic to use to write as camera pixels.',
        required=False,
        default='/points_pixels',
    )


    # get the arguments from the argument parser
    ARGS = PARSER.parse_args()
    # play the bag given in the arguments
    try:
        # open the input bag
        INPUT_BAG = Bag(ARGS.input_bag, 'r')
        # open the output bag
        OUTPUT_BAG = Bag(ARGS.output_bag, 'w')
        # transform input bag point cloud data into image data in the output bag
        pointcloud2_to_image(INPUT_BAG, OUTPUT_BAG, ARGS.point_cloud, ARGS.camera)
        # close the bag files
        OUTPUT_BAG.close()
        INPUT_BAG.close()
    except KeyboardInterrupt:
        pass


# explicitly define the outward facing API of this module
__all__ = [pointcloud2_to_image.__name__]
