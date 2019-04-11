#!/usr/bin/python

import rosbag
import glob
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
import sys
import math

# check for correct usage
if len(sys.argv) < 3:
    sys.exit("Usage: " + sys.argv[0] + " folder/bag topic_name [target_framerate]")

# load parameters
prefix = sys.argv[1]
topic_name = sys.argv[2]
if len(sys.argv) == 4:
  target_framerate = float(sys.argv[3])
else:
  target_framerate = -1.

# Use a CvBridge to convert ROS images to OpenCV images so they can be saved.
bridge = CvBridge()

# get all bag files in that folder
if prefix.endswith(".bag"):
    bag_files = [prefix]
else:
    bag_files = glob.glob(prefix + "/*.bag")

def get_filename(i):
  return folder_name + "/frame_" + str(i).zfill(5)  + ".png"

for bag_file in bag_files:
    print "Processing file: " + bag_file
    if target_framerate > 0.:
      print "Targeting framerate of " + str(target_framerate)

    # folder name is without the .bag
    folder_name = bag_file[:-4]

    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    i = 0
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == topic_name:
                try:
                    image_type = msg.encoding
                    cv_image = bridge.imgmsg_to_cv2(msg, image_type)
                except CvBridgeError, e:
                    print e

                if target_framerate > 0.:
                  if i==0:
                    first_timestamp = msg.header.stamp
                    cv2.imwrite(get_filename(i), cv_image)
                    i = i + 1
                  else:
                    diff = msg.header.stamp - first_timestamp
                    last_timestamp = msg.header.stamp

                    while i/target_framerate < diff.to_sec():
                      cv2.imwrite(get_filename(i), cv_image)
                      i = i + 1

                else:
                  cv2.imwrite(get_filename(i), cv_image)
                  i = i + 1

    print "Wrote " + str(i) + " images into " + folder_name
    if target_framerate > 0.:
      print "Target framerate for dataset of " + str(round(diff.to_sec(), 2)) + "s gives " + str(int(math.ceil(diff.to_sec()*target_framerate))) + " frames"
