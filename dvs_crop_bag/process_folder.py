# -*- coding: utf-8 -*-

# !/usr/bin/python
"""

"""

from os.path import join
import os
import subprocess
from glob import glob
import random
import yaml
import argparse
import numpy as np


def build_run_command(path_to_input_bag, num_msgs, num_skip):
    # "rosrun dvs_crop_bag dvs_crop_bag path_to_input num_msgs"
    rosrun_cmd = ['rosrun',
                  'dvs_crop_bag',
                  'dvs_crop_bag',
                  path_to_input_bag,
                  str(num_msgs),
                  str(num_skip)]

    return rosrun_cmd


if __name__ == "__main__":

    # Parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder",
                        required=True,
                        type=str,
                        help="directory of rosbags")
    parser.add_argument("--num_msgs",
                        required=True,
                        type=int,
                        help="number of messages to write")
    parser.add_argument("--num_skip",
                        required=False,
                        type=int,
                        default=0,
                        help="number of messages to skip")
    args = parser.parse_args()

    for file in os.listdir(args.folder):
        if file.endswith(".bag"):
            # print(file)
            # append the arguments to the rosrun command
            rosrun_cmd = build_run_command(os.path.join(args.folder, file), args.num_msgs, args.num_skip)
            # print(rosrun_cmd)
            print(subprocess.check_output(rosrun_cmd))

    # now summarise output txt files

    # num_events_list = list()
    # num_frames_list = list()
    # duration_list = list()
    # stats_dir = os.path.join(args.folder, "stats")
    # for file in os.listdir(stats_dir):
    #     if file.endswith(".txt"):
    #         num_events, num_frames, duration = np.loadtxt(os.path.join(stats_dir, file), delimiter=',')
    #         num_events_list.append(num_events)
    #         num_frames_list.append(num_frames)
    #         duration_list.append(duration)
    #
    # print(np.sum(num_events_list), np.sum(num_frames_list), np.sum(duration_list))

