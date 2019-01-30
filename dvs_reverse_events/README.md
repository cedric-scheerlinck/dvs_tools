# DVS Reverse Events
Sometimes, you have a whacky feeling and you just want to reverse the timestamps of all events in a rosbag.
This package allows you to do so.

Usage:

        rosrun dvs_reverse_events reverse_events path_to_input.bag

A new rosbag will be written, name ```input.bag.reversed```. It is an exact copy of the original bag, except hot pixel events have been removed.

## Installation

Assumes you have already set up your workspace (see [here](https://github.com/cedric-scheerlinck/dvs_image_reconstruction) for more details).

Clone this repository into your catkin workspace:

    cd ~/catkin_ws/src/
    git clone https://github.com/cedric-scheerlinck/dvs_reverse_events.git

Clone dependencies:

    vcs-import < dvs_reverse_events/dependencies.yaml

Build

    catkin build dvs_reverse_events
    source ~/catkin_ws/devel/setup.bash
