#!/usr/bin/env python
# Copyright 2016 Massachusetts Institute of Technology
# This code has been extracted from this link: https://gist.github.com/wngreene/835cda68ddd9c5416defce876a4d7dd9

"""Extract images from a rosbag.
"""

import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("--bag_file", "-b", type=str, help="Input ROS bag.", required=True)
    parser.add_argument("--output_dir", "-o", type=str, help="Output directory.", required=True)
    parser.add_argument("--image_topic", "-t", type=str, help="Image topic.", required=True)
    args = parser.parse_args()

    print("Extract images from {} on topic {} into {}".format(args.bag_file, args.image_topic, args.output_dir))

    abs_bag_path = os.path.abspath(args.bag_file)
    output_path = os.path.abspath(args.output_dir)
    if not os.path.exists(abs_bag_path):
        print("ERROR: The given filepath does not exists! Aborting.")
        return -1

    final_output_path = os.path.join(output_path, os.path.basename(abs_bag_path).split(".")[0])
    try:
        os.makedirs(final_output_path, exist_ok=True)
    except PermissionError as e:
        print("ERROR: No permissions to write in {}. Aborting.".format(final_output_path))
        return -1

    bag = rosbag.Bag(abs_bag_path, "r")
    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        cv2.imwrite(os.path.join(final_output_path, "frame{:06d}.png".format(count)), cv_img)
        print("Wrote image {:06d}".format(count))
        count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()