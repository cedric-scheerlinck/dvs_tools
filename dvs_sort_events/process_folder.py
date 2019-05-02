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


def build_run_command(path_to_input_bag, max_events, max_duration_ms):
    # "rosrun dvs_sort_events sort path_to_input --max_events=n --max_duration_ms=t"
    rosrun_cmd = ['rosrun',
                  'dvs_sort_events',
                  'sort',
                  path_to_input_bag,
                  f'--max_events={max_events}',
                  f'--max_duration_ms={max_duration_ms}'] 

    return rosrun_cmd


if __name__ == "__main__":

    # Parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder",
                        required=True,
                        type=str,
                        help="directory of rosbags")
    parser.add_argument("--max_events",
                        required=True,
                        type=str,
                        help="Maximum number of events per output message")
    parser.add_argument("--max_duration_ms",
                        required=True,
                        type=str,
                        help="Maximum duration of output message")
    args = parser.parse_args()

    for file in os.listdir(args.folder):
        if file.endswith(".bag"):
            rosrun_cmd = build_run_command(os.path.join(args.folder, file), args.max_events, args.max_duration_ms)
            print(subprocess.check_output(rosrun_cmd))
