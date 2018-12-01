# Python ROSBag Tools

Use `python <script> --help` to print documentation for each
of the following scripts.

## Clip

`clip.py` extracts clips from a bag file using a sentinel topic and range
of message indexes.

## Decompress

`decompress.py` decompresses compressed image data to raw image data.

## Dump

`dump.py` dumps messages to the terminal and optionally to disk

## Extract Depth

`extract_depth.py` extracts depth measures to NumPy compressed tensors.

## Merge

`merge.py` both merges bag files and filters topics.

## Play Depth

`play_depth.py` plays image and depth data to pyglet windows.

## Play Images

`play_images.py` plays raw image data to pyglet windows

## Play Super Pixel

`play_super_pixel.py` plays raw image data through a super pixel segmentation
algorithm to a pyglet window.

## Semantic Segment

`semantic_segment.py` produces samples from a semantic segmentation model using
an image topic in a ROS bag.

## Shift

`shift.py` shifts a topic in an ROS bag by an offset in seconds.

## Video To Bag

`video_to_bag.py` converts a video file to a ROS bag with image topics.
