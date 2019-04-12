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


def build_run_command(path_to_input_bag, height, width):
    # "rosrun dvs_frame_filter dvs_frame_filter path_to_input"
    if height == 0 or width == 0:
        rosrun_cmd = ['rosrun',
                      'dvs_frame_filter',
                      'dvs_frame_filter',
                      path_to_input_bag]
    else:
    # "rosrun dvs_frame_filter dvs_frame_filter path_to_input height width"
        rosrun_cmd = ['rosrun',
                      'dvs_frame_filter',
                      'dvs_frame_filter',
                      path_to_input_bag,
                      height,
                      width]

    return rosrun_cmd


if __name__ == "__main__":

    # Parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder",
                        required=True,
                        type=str,
                        help="directory of rosbags")
    parser.add_argument("--height",
                        required=False,
                        type=int,
                        help="height")
    parser.add_argument("--width",
                        required=False,
                        type=int,
                        default=0,
                        help="width")
    args = parser.parse_args()

    for file in os.listdir(args.folder):
        if file.endswith(".bag"):
            # print(file)
            # append the arguments to the rosrun command
            rosrun_cmd = build_run_command(os.path.join(args.folder, file), args.height, args.width)
            # print rosrun_cmd
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

