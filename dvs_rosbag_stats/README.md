# DVS Hot Pixel Filter

Description.

Usage:

        rosrun dvs_rosbag_stats dvs_rosbag_stats path_to_input.bag
        
## Installation

Assumes you have already set up your workspace (see [here](https://github.com/cedric-scheerlinck/dvs_image_reconstruction) for more details).

Build

    catkin build dvs_rosbag_stats
    source ~/catkin_ws/devel/setup.bash 
    
## Statistics

Saves output statistics to ```./stats/<bag_name>.txt```.

