"""A method to extract camera dimensions from a ROS bag."""


def get_camera_dimensions(bag: 'rosbag.Bag', metadata_topic: str) -> str:
    """
    Return the dimensions for the camera topic in the given bag.

    Args:
        bag: the bag file to read messages from
        metadata_topic: the topic to get the camera properties from

    Returns:
        a tuple with the dimensions of the camera

    """
    # extract the camera dimensions from the bag
    dims = None
    for _, cd_msg, _ in bag.read_messages(topics=metadata_topic):
        # setup dims of the first pass by
        if dims is None:
            dims = cd_msg.height, cd_msg.width
        # make sure the dimensions never change after the first reading
        elif dims != (cd_msg.height, cd_msg.width):
            print('WARNING: camera dimensions changed mid-test')

    return dims


# explicitly define the outward facing API of this module
__all__ = [get_camera_dimensions.__name__]
