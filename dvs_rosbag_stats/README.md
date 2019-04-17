# DVS Rosbag Stats

Description.

### Usage:
#### Single .bag file:

        rosrun dvs_rosbag_stats dvs_rosbag_stats path_to_input.bag
      
#### Process folder (used to process an entire folder of .bag files):

        python process_folder.py --folder <bag/folder/>
        
## Installation

Assumes you have already set up your workspace (see [here](https://github.com/cedric-scheerlinck/dvs_image_reconstruction) for more details).

Build

    catkin build dvs_rosbag_stats
    source ~/catkin_ws/devel/setup.bash 
    
## Statistics

Saves output statistics to ```<bag_folder>/stats/<bag_name>.txt```.
