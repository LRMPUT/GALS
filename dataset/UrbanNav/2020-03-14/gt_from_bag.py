#!/usr/bin/env python
# -*- coding: utf-8 -*-


import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract GT data from ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("data_topic", help="Data topic.")

    args = parser.parse_args()
    bag = rosbag.Bag(args.bag_file, "r")
    count = 0
    with open("gt_from_bag.csv", 'w') as out:
        pass

    for topic, msg, t in bag.read_messages(topics=[args.data_topic]):
        with open("gt_from_bag.csv", 'a') as out:
             out.write( '{:d},{:d},{:f},{:f},{:f}'.format(msg.header.gps_week,msg.header.gps_week_seconds/100,msg.latitude,msg.longitude,msg.altitude))
             out.write('\n')
    bag.close()

    return

if __name__ == '__main__':
    main()
    print("Finished")
