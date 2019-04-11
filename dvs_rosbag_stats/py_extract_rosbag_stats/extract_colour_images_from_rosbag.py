#!/usr/bin/python
import sys
print(sys.path)
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")  # do this until update ros python to 3
import rosbag
import glob
import cv2
import os
import sys
import math


# check for correct usage
# if len(sys.argv) < 3:
#     sys.exit("Usage: " + sys.argv[0] + " folder/bag topic_name_dvs topic_name_frame")

# load parameters
# prefix = sys.argv[1]
# topic_name_dvs = sys.argv[2]
# topic_name_frame = sys.argv[3]

prefix = "/home/cedric/Documents/phd/data/rosbags/color_dataset_evaluation/good"
topic_name_dvs = "/dvs/events"
topic_name_frame = "/dvs/image_color"
# get all bag files in that folder
if prefix.endswith(".bag"):
    bag_files = [prefix]
else:
    bag_files = glob.glob(prefix + "/*.bag")

stats = dict()

for bag_file in bag_files:
    print("Processing file: " + bag_file)

    # sequence name is without the .bag
    sequence_name = bag_file[:-4]

    i = 0
    eventCount = 0
    frameCount = 0
    firstMsg = True
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == topic_name_dvs:
                "collect event stats"
                eventCount += len(msg.events)
            if topic == topic_name_frame:
                "collect frame stats"
                frameCount += 1
                if firstMsg:
                    startTime = msg.header.stamp.to_sec()
                    firstMsg = False
                endTime = msg.header.stamp.to_sec()
                print(endTime - startTime)
                break
    stats[sequence_name]["duration"] = endTime - startTime
    stats[sequence_name]["eventCount"] = eventCount
    stats[sequence_name]["frameCount"] = frameCount
    break


for sequence_name, stat in stats.items():
   print(sequence_name)
   print(stat)




