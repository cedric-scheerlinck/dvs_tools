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


def build_run_command(path_to_input_bag, time_intervals):
    # "rosrun dvs_crop_time dvs_crop_time path_to_input"
    rosrun_cmd = ['rosrun',
                  'dvs_crop_time',
                  'dvs_crop_time',
                  path_to_input_bag,
                  f'--t1s={time_intervals[0]}',
                  f'--t1e={time_intervals[1]}',
                  f'--t2s={time_intervals[2]}',
                  f'--t2e={time_intervals[3]}'] 

    return rosrun_cmd


if __name__ == "__main__":

    # Parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder",
                        required=True,
                        type=str,
                        help="directory of rosbags")
    parser.add_argument("--config_path",
                        required=True,
                        type=str,
                        help="path to times.txt config file")
    args = parser.parse_args()

    with open(args.config_path, 'r') as f:
        for line in f.readlines():
            argList = line.rstrip().split(' ')
            rosrun_cmd = build_run_command(os.path.join(args.folder, argList[0]), argList[1:])
            print(subprocess.check_output(rosrun_cmd))

